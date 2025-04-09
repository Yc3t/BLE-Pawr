import serial
import struct
import time
from pymongo import MongoClient
import datetime
import collections # For deque
import pymongo  # Added explicit import

# --- Configuration ---
SERIAL_PORT = 'COM77'  
BAUD_RATE = 115200
TIMEOUT = 0.1 # Shorter timeout for non-blocking reads, but main loop handles waiting

MONGO_URI = "mongodb://localhost:27017/" # Default local MongoDB instance
DB_NAME = "pawr_sensor_data"
COLLECTION_NAME = "collector_readings2"

# --- Data Format Constants (matching central.c) ---
UART_MAGIC_HEADER_INT = 0x55555555
UART_MAGIC_HEADER_BYTES = UART_MAGIC_HEADER_INT.to_bytes(4, byteorder='little')
MAX_SENSORS = 6        # Maximum number of sensors (updated from 3 to 6)
MAX_MESSAGES_PER_SENSOR = 10  # Each sensor reports up to 10 messages

# Calculate minimum buffer size (header only)
MIN_BUFFER_SIZE = 12  # 4 (magic) + 4 (seq) + 4 (num_sensors)

# Calculate maximum possible packet size (worst case scenario)
# Header (12) + All sensors (6 * (3 + 10*4)) = 12 + 6*(3 + 40) = 12 + 6*43 = 12 + 258 = 270 bytes
MAX_PACKET_SIZE = MIN_BUFFER_SIZE + (MAX_SENSORS * (3 + (MAX_MESSAGES_PER_SENSOR * 4)))

def connect_to_serial(port, baud, timeout):
    """Attempts to connect to the specified serial port."""
    try:
        # Use a shorter timeout for read, but rely on main loop logic for waiting
        ser = serial.Serial(port, baud, timeout=timeout)
        print(f"Successfully connected to {port} at {baud} baud.")
        return ser
    except serial.SerialException as e:
        print(f"Error connecting to serial port {port}: {e}")
        return None
    except Exception as e:
        print(f"An unexpected error occurred during serial connection: {e}")
        return None

def connect_to_mongodb(uri, db_name, collection_name):
    """Attempts to connect to MongoDB and get the collection."""
    try:
        client = MongoClient(uri, serverSelectionTimeoutMS=5000) # Add timeout
        # The ismaster command is cheap and does not require auth.
        client.admin.command('ismaster')
        print("Successfully connected to MongoDB.")
        db = client[db_name]
        collection = db[collection_name]
        print(f"Using database '{db_name}' and collection '{collection_name}'.")
        return collection
    except pymongo.errors.ServerSelectionTimeoutError as e:
         print(f"Error connecting to MongoDB (Timeout): {e}")
         return None
    except pymongo.errors.ConnectionFailure as e:
        print(f"Error connecting to MongoDB: {e}")
        return None
    except Exception as e:
        print(f"An unexpected error occurred during MongoDB connection: {e}")
        return None

def calculate_packet_size(buffer_bytes):
    """Calculates the total packet size based on the header information."""
    if len(buffer_bytes) < MIN_BUFFER_SIZE:
        return None  # Not enough data to determine size
    
    try:
        # Unpack number of sensors
        num_sensors = struct.unpack_from('<I', buffer_bytes, 8)[0]
        
        if num_sensors > MAX_SENSORS * 2:  # Use a reasonable upper limit
            print(f"Error: Invalid num_sensors value ({num_sensors}), likely corrupted data")
            return None
            
        # Initial size for header
        total_size = MIN_BUFFER_SIZE
        
        # We need to read at least the metadata for each sensor
        # to calculate the full packet size
        min_sensor_metadata_size = num_sensors * 3  # 1 (ID) + 1 (RSSI) + 1 (n_messages)
        
        if len(buffer_bytes) < MIN_BUFFER_SIZE + min_sensor_metadata_size:
            # Not enough data yet to read all sensor metadata
            return None
        
        # Calculate size by reading each sensor's data
        offset = MIN_BUFFER_SIZE
        for i in range(num_sensors):
            if offset + 3 > len(buffer_bytes):
                # Not enough data to read sensor metadata
                return None
                
            # Read number of messages for this sensor
            sensor_id = buffer_bytes[offset]      # First byte is sensor ID
            rssi = buffer_bytes[offset + 1]       # Second byte is RSSI (signed)
            num_messages = buffer_bytes[offset + 2]  # Third byte is message count
            
            if num_messages > MAX_MESSAGES_PER_SENSOR * 2:  # Use a reasonable upper limit
                print(f"Error: Sensor {sensor_id} claims {num_messages} messages, likely corrupted data")
                return None
            
            # Size for this sensor: 3 bytes metadata + (4 bytes per message)
            sensor_size = 3 + (num_messages * 4)
            total_size += sensor_size
            
            offset += 3  # Move to payload data
            
            # Skip payload data for size calculation
            offset += (num_messages * 4)
            
        # Sanity check - if calculated size is too large, it might be corrupted data
        if total_size > MAX_PACKET_SIZE * 2:
            print(f"Warning: Calculated packet size ({total_size}) exceeds expected maximum ({MAX_PACKET_SIZE})")
            
        return total_size
        
    except Exception as e:
        print(f"Error calculating packet size: {e}")
        return None

def parse_uart_data(data_bytes):
    """Parses the variable-length binary data received from the central device."""
    if len(data_bytes) < MIN_BUFFER_SIZE:
        print(f"Error: Data too short to be valid packet ({len(data_bytes)} bytes)")
        return None
    
    try:
        offset = 0
        # Validate header bytes
        magic_read = data_bytes[offset:offset+4]
        if magic_read != UART_MAGIC_HEADER_BYTES:
            print(f"Error: Invalid magic header: {magic_read.hex()} != {UART_MAGIC_HEADER_BYTES.hex()}")
            return None
        offset += 4

        # Unpack sequence number and number of sensors
        sequence_number, num_sensors = struct.unpack_from('<II', data_bytes, offset)
        offset += 8  # 4 bytes for seq + 4 bytes for num_sensors
        
        print(f"Parsing packet: seq={sequence_number}, sensors={num_sensors}")

        if num_sensors > MAX_SENSORS:
            print(f"Warning: Packet claims {num_sensors} sensors, max expected is {MAX_SENSORS}")
            # We'll try to parse all claimed sensors, but be cautious
            
        sensors_parsed = []
        for i in range(num_sensors):
            # Read sensor metadata
            if offset + 3 > len(data_bytes):
                print(f"Error: Buffer too short at sensor {i+1}/{num_sensors} metadata (offset {offset}, buffer length {len(data_bytes)})")
                return None
                
            sensor_id, rssi, num_msgs = struct.unpack_from('<BbB', data_bytes, offset)
            offset += 3
            
            if num_msgs > MAX_MESSAGES_PER_SENSOR:
                print(f"Warning: Sensor {sensor_id} claims {num_msgs} messages, max expected is {MAX_MESSAGES_PER_SENSOR}")
                num_msgs = MAX_MESSAGES_PER_SENSOR  # Cap at reasonable number
            
            # Ensure we have enough data for payloads
            payload_size = num_msgs * 4  # 4 bytes per message
            if offset + payload_size > len(data_bytes):
                print(f"Error: Buffer too short for sensor {sensor_id} payloads (need {payload_size} bytes at offset {offset}, have {len(data_bytes)-offset})")
                return None
                
            # Extract all payloads for this sensor
            payloads = []
            for j in range(num_msgs):
                payload = struct.unpack_from('<I', data_bytes, offset)[0]
                payloads.append(payload)
                offset += 4
            
            sensors_parsed.append({
                "sensor_id": sensor_id,
                "rssi": rssi,
                "message_count": num_msgs,
                "payloads": payloads,
                "payloads_hex": [f"0x{p:08X}" for p in payloads]  # Hex representation for readability
            })

        # Create MongoDB document
        document = {
            "timestamp": datetime.datetime.now(datetime.timezone.utc),
            "sequence_number": sequence_number,
            "num_sensors": num_sensors,
            "sensors": sensors_parsed,
            "total_bytes": len(data_bytes)  # Add packet size for debugging
        }
        return document

    except struct.error as e:
        print(f"Error unpacking data at offset {offset}: {e}")
        return None
    except IndexError as e:
        print(f"Error: Index out of bounds during parsing (offset {offset}): {e}")
        return None
    except Exception as e:
        print(f"An unexpected error during parsing: {e}")
        return None

def main():
    """Main loop to read UART, parse data, and store in MongoDB."""
    ser = connect_to_serial(SERIAL_PORT, BAUD_RATE, TIMEOUT)
    if ser is None:
        return

    collection = connect_to_mongodb(MONGO_URI, DB_NAME, COLLECTION_NAME)
    if collection is None:
        ser.close()
        return

    print(f"\n--- Listening on {SERIAL_PORT} for sensor data (Magic Header: {UART_MAGIC_HEADER_BYTES.hex()}) ---")
    print(f"Expecting variable-length packets with up to {MAX_SENSORS} sensors, {MAX_MESSAGES_PER_SENSOR} messages each")
    print(f"Maximum possible packet size: {MAX_PACKET_SIZE} bytes")

    # Use a deque for efficient adding/removing from both ends
    read_buffer = collections.deque(maxlen=MAX_PACKET_SIZE * 2)  # Increase buffer size to handle larger packets

    try:
        while True:
            # Read available data without blocking indefinitely
            bytes_waiting = ser.in_waiting
            if bytes_waiting > 0:
                new_data = ser.read(bytes_waiting)
                if new_data:
                    read_buffer.extend(new_data)
                    print(f"Read {len(new_data)} bytes, buffer size: {len(read_buffer)}")

            # Try to find a packet in the buffer
            buffer_bytes = bytes(read_buffer)
            header_index = buffer_bytes.find(UART_MAGIC_HEADER_BYTES)

            if header_index != -1:
                # Found header, check if we have enough data to determine packet size
                if len(buffer_bytes) >= header_index + MIN_BUFFER_SIZE:
                    # Get packet bytes starting from header
                    potential_packet = buffer_bytes[header_index:]
                    
                    # Calculate full packet size
                    packet_size = calculate_packet_size(potential_packet)
                    
                    if packet_size is not None and len(potential_packet) >= packet_size:
                        # We have a complete packet
                        packet_bytes = potential_packet[:packet_size]
                        print(f"Found complete packet: {packet_size} bytes")
                        
                        # Parse and store the packet
                        parsed_document = parse_uart_data(packet_bytes)
                        if parsed_document:
                            try:
                                result = collection.insert_one(parsed_document)
                                print(f"Successfully stored data. Doc ID: {result.inserted_id}, Seq: {parsed_document['sequence_number']}")
                                
                                # Print sensor information
                                print("Sensor Summary:")
                                for sensor in parsed_document["sensors"]:
                                    print(f"  Sensor {sensor['sensor_id']}: RSSI={sensor['rssi']}, " +
                                          f"Messages={sensor['message_count']}")
                                    # Limit displayed payloads if too many
                                    if sensor['message_count'] > 3:
                                        print(f"    First 3 of {sensor['message_count']} messages:")
                                        for i in range(3):
                                            print(f"    Msg {i+1}: {sensor['payloads_hex'][i]}")
                                    else:
                                        for i, payload in enumerate(sensor['payloads_hex']):
                                            print(f"    Msg {i+1}: {payload}")
                                
                            except pymongo.errors.PyMongoError as e:
                                print(f"Error inserting into MongoDB: {e}")
                            except Exception as e:
                                print(f"Unexpected error during DB insertion: {e}")
                        
                        # Consume the processed part from the buffer
                        for _ in range(header_index + packet_size):
                            if read_buffer:
                                read_buffer.popleft()
                        print(f"Buffer size after processing: {len(read_buffer)}")
                    else:
                        # Need more data or couldn't determine size
                        if packet_size is None:
                            print(f"Found header at index {header_index}, waiting for more data to determine size")
                        else:
                            print(f"Found header at index {header_index}, need {packet_size} bytes, have {len(potential_packet)}")
                            
                        # If we have too much unconsumed data before the header, discard it
                        if header_index > 100:  # Arbitrary threshold
                            for _ in range(header_index):
                                if read_buffer:
                                    read_buffer.popleft()
                            print(f"Discarded {header_index} bytes before header")

            # Prevent excessive looping if no data is coming
            if not bytes_waiting:
                time.sleep(0.1)  # Small delay when idle

    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Exiting program.")
    except Exception as e:
        print(f"\nAn unexpected error occurred in the main loop: {e}")
    finally:
        if ser and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    main()