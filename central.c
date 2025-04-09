#include <zephyr/sys/byteorder.h> // Needed for handling Little Endian data
#include <zephyr/bluetooth/att.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/drivers/sensor.h>
#include <dk_buttons_and_leds.h>

// UNCOMMENT for low power mode
//#define LOW_POWER_MODE

#ifdef LOW_POWER_MODE
#define PA_INTERVAL 0x320 // 1 second
#define SUBEVENT_INTERVAL 0xF0 // 300 ms  
#else
#define PA_INTERVAL 0xFF // 318.75 ms
#define SUBEVENT_INTERVAL 0x50 // 100 ms
#endif

#define NUM_SENSORS 3

// Sensor variables
static uint32_t sensor_fixed_values[NUM_SENSORS] = {0};

// 9 bytes: 1 byte for length, 1 byte for type, 2 bytes for company ID, 1 byte for device ID, 4 bytes for fixed payload
#define RESPONSE_DATA_SIZE 9

// We will be assigning each sensor node a subevent and response slot
// We will have a maximum of 3 sensor nodes
#define NUM_RSP_SLOTS 1
#define NUM_SUBEVENTS NUM_SENSORS
#define PACKET_SIZE   5
#define NAME_LEN      30

// Device Discovery Definitions
#define NOVEL_BITS_COMPANY_ID 0x08D3

// PAwR command definitions
#define PAWR_CMD_FIXED_PAYLOAD 0x01
#define PAWR_CMD_SLEEP_REQUEST 0x02

// Define a structure to track sensor data collection
typedef struct {
    bool data_received;        // Whether we've received data from this sensor AT LEAST ONCE this cycle
    uint8_t response_count;    // Number of responses received this cycle (up to 10)
    uint32_t last_heard_time;  // Last time we heard from this sensor
    uint8_t device_id;         // Device ID of the sensor
} sensor_tracker_t;

// Tracker array for all sensors
static sensor_tracker_t sensor_trackers[NUM_SENSORS] = {0};

// Timeout for waiting for all sensors (in ms)
#define SENSOR_COLLECTION_TIMEOUT_MS 30000 // Use 30 seconds for scanning (increased from 10 seconds)

// Flag to indicate if we're in data collection phase or sleep command phase
static enum {
    PHASE_DATA_COLLECTION,  // Collecting data from all sensors
    PHASE_SLEEP_COMMAND     // Sending sleep commands to sensors
} operation_phase = PHASE_DATA_COLLECTION;

// Add central state machine - moved here before connected_cb for proper scope
enum central_state {
    STATE_SCANNING,       // Actively scanning for sensors
    STATE_SLEEP_PENDING,  // Finished scanning, sending sleep commands
    STATE_SLEEPING        // In sleep period, ignoring new sensors
};

// Current state of the central device - moved here before connected_cb
static enum central_state current_state = STATE_SCANNING;

// Index of the next sensor to send sleep command to
static int next_sleep_cmd_idx = 0;

// A delay between sending sleep commands to different sensors
#define SLEEP_CMD_INTERVAL_MS 500

// Sleep duration command - all sensors will be told to sleep for this time
#define SENSOR_SLEEP_DURATION_S 60 // Use 60 seconds (1 minute) for sleep

// Forward declarations for all functions that are used before being defined
static void cmd_reset_timeout(struct k_timer *timer);
static void collection_timeout_handler(struct k_timer *timer);
static void send_next_sleep_cmd(struct k_timer *timer);
static bool all_sensors_reported(void);
static void init_sensor_trackers(void);

// Add a timer to reset the command back to fixed payload
K_TIMER_DEFINE(cmd_reset_timer, cmd_reset_timeout, NULL);

// Timer for sensor collection timeout
K_TIMER_DEFINE(collection_timer, collection_timeout_handler, NULL);

// Timer for sending sleep commands
K_TIMER_DEFINE(sleep_cmd_timer, send_next_sleep_cmd, NULL);

typedef struct adv_data
{
    // Device Name
    char name[NAME_LEN];

    /* data */
    bool novelbits_id_present;
    uint8_t data[30];

} custom_adv_data_t;

// Initialize the currently connected device ID to 0xFF (Invalid ID)
static uint8_t currently_connected_device_id = 0xFF;

// PAwR Definitions
static uint8_t current_pawr_command = PAWR_CMD_FIXED_PAYLOAD;

static K_SEM_DEFINE(sem_connected, 0, 1);
static K_SEM_DEFINE(sem_discovered, 0, 1);
static K_SEM_DEFINE(sem_written, 0, 1);
static K_SEM_DEFINE(sem_disconnected, 0, 1);

static struct bt_uuid_128 pawr_char_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1));
static uint16_t pawr_attr_handle;
static const struct bt_le_per_adv_param per_adv_params = {
    .interval_min = PA_INTERVAL,
    .interval_max = PA_INTERVAL,
    .options = 0, // No options
    .num_subevents = NUM_SUBEVENTS, // 3
    .subevent_interval = SUBEVENT_INTERVAL,
    .response_slot_delay = 0x5, // 6.25 ms
    .response_slot_spacing = 0xFA, // 31.25 ms
    .num_response_slots = NUM_RSP_SLOTS, // 1
};

#define DEVICE_NAME "Weather_Station"
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME)-1)

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN)
};

static struct bt_le_per_adv_subevent_data_params subevent_data_params[NUM_SUBEVENTS];
static struct net_buf_simple bufs[NUM_SUBEVENTS];
static uint8_t backing_store[NUM_SUBEVENTS][PACKET_SIZE];

BUILD_ASSERT(ARRAY_SIZE(bufs) == ARRAY_SIZE(subevent_data_params));
BUILD_ASSERT(ARRAY_SIZE(backing_store) == ARRAY_SIZE(subevent_data_params));

uint32_t quick_ieee11073_from_float(float temperature)
{
    uint8_t  exponent = 0xFE; //Exponent is -2
    uint32_t mantissa = (uint32_t)(temperature * 100);

    return (((uint32_t)exponent) << 24) | mantissa;
}

static struct bt_conn *central_conn;
static struct bt_conn *peripheral_conn;

static void request_cb(struct bt_le_ext_adv *adv, const struct bt_le_per_adv_data_request *request);
static void response_cb(struct bt_le_ext_adv *adv, struct bt_le_per_adv_response_info *info,
             struct net_buf_simple *buf);

static const struct bt_le_ext_adv_cb adv_cb = {
    .pawr_data_request = request_cb,
    .pawr_response = response_cb,
};

void connected_cb(struct bt_conn *conn, uint8_t err)
{

    printk("Connected (err 0x%02X)\n", err);

    // Success case
    if (err == 0) {
        if (conn != central_conn)
        {
            // Connected as a Peripheral
            printk("Connected as a Peripheral\n");
            peripheral_conn = bt_conn_ref(conn);
            
            // Turn on LED1 to indicate peripheral connection
            dk_set_led_on(DK_LED1);
        } else {
            // Connected as Central - give the semaphore to continue the flow
            printk("Connected as Central to device ID: %d\n", currently_connected_device_id);
            
            // Since connection is successful, we'll give the semaphore later in remote_info_available
            // This ensures the BLE stack has time to complete connection setup
        }
    }
    // Failure Case
    else
    {
        printk("Connection failed with error 0x%02X\n", err);
        
        if (conn == central_conn) {
            bt_conn_unref(central_conn);
            central_conn = NULL;
            
            // If connection fails, we should restart scanning
            printk("Central connection failed, will restart scan in next cycle\n");
            
            // Simply return to scanning state
            current_state = STATE_SCANNING;
        }
        else if (conn == peripheral_conn) {
            bt_conn_unref(peripheral_conn);
            peripheral_conn = NULL;
            
            // Turn off LED1 if peripheral connection failed
            dk_set_led_off(DK_LED1);
        }
    }
}

void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected (reason 0x%02X)\n", reason);

    if (conn == central_conn) {
        bt_conn_unref(central_conn);
        central_conn = NULL;
        k_sem_give(&sem_disconnected);
        currently_connected_device_id = 0xFF;
        
        // Add a small delay after disconnection 
        // to ensure BT controller is in a clean state
        k_sleep(K_MSEC(50));
    }
    else if (conn == peripheral_conn) {
        bt_conn_unref(peripheral_conn);
        peripheral_conn = NULL;
        
        // Turn off LED1 when peripheral disconnects
        dk_set_led_off(DK_LED1);
    }
}

void remote_info_available_cb(struct bt_conn *conn, struct bt_conn_remote_info *remote_info)
{
    /* Need to wait for remote info before initiating PAST  -- only as a Central*/

    if (conn == central_conn) {
        printk("Remote info available for central connection\n");
               
        // Small delay to ensure stack is ready
        k_sleep(K_MSEC(50));
        
        // Now we can proceed with PAST and other operations
        k_sem_give(&sem_connected);
    }
}

BT_CONN_CB_DEFINE(conn_cb) = {
    .connected = connected_cb,
    .disconnected = disconnected_cb,
    .remote_info_available = remote_info_available_cb,
};

static bool data_cb(struct bt_data *data, void *user_data)
{
    custom_adv_data_t *adv_data_struct = user_data;
    uint8_t len;

    switch (data->type) {
    case BT_DATA_NAME_SHORTENED:
    case BT_DATA_NAME_COMPLETE:
        len = MIN(data->data_len, NAME_LEN - 1);
        memcpy(adv_data_struct->name, data->data, len);
        adv_data_struct->name[len] = '\0';
        return true;

    case BT_DATA_MANUFACTURER_DATA:
        if (data->data_len < 3) {
            return true;
        }

        if (sys_get_le16(&(data->data[0])) != NOVEL_BITS_COMPANY_ID) {
            return true;
        }

        printk("Found Novel Bits Company ID\n");

        adv_data_struct->novelbits_id_present = true;
        memcpy(adv_data_struct->data, &data->data[0], data->data_len);

        return false;
    default:
        return true;
    }
}

// Timer for the full sleep cycle
K_TIMER_DEFINE(sleep_cycle_timer, NULL, NULL);

// Forward declare scan_param
static const struct bt_le_scan_param scan_param = {
    .type     = BT_LE_SCAN_TYPE_ACTIVE,
    .interval = BT_GAP_SCAN_FAST_INTERVAL,
    .window   = BT_GAP_SCAN_FAST_WINDOW,
    .options  = BT_LE_SCAN_OPT_CODED | BT_LE_SCAN_OPT_NO_1M
};

// Forward declare device_found
static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
             struct net_buf_simple *ad);

// Add the missing struct definition
struct pawr_timing {
    uint8_t subevent;
    uint8_t response_slot;
} __packed;

// Add missing function definitions
static void init_bufs(void)
{
    for (size_t i = 0; i < ARRAY_SIZE(backing_store); i++) {
        backing_store[i][0] = ARRAY_SIZE(backing_store[i]) - 1;
        backing_store[i][1] = BT_DATA_MANUFACTURER_DATA;
        backing_store[i][2] = (NOVEL_BITS_COMPANY_ID & 0xFF); /* Novel Bits */
        backing_store[i][3] = ((NOVEL_BITS_COMPANY_ID >> 8) & 0xFF);

        net_buf_simple_init_with_data(&bufs[i], &backing_store[i],
                          ARRAY_SIZE(backing_store[i]));
    }
}

static uint8_t discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                 struct bt_gatt_discover_params *params)
{
    struct bt_gatt_chrc *chrc;
    char str[BT_UUID_STR_LEN];

    printk("Discovery: attr %p\n", attr);

    if (!attr) {
        return BT_GATT_ITER_STOP;
    }

    chrc = (struct bt_gatt_chrc *)attr->user_data;

    bt_uuid_to_str(chrc->uuid, str, sizeof(str));
    printk("UUID %s\n", str);

    if (!bt_uuid_cmp(chrc->uuid, &pawr_char_uuid.uuid)) {
        pawr_attr_handle = chrc->value_handle;

        printk("Characteristic handle: %d\n", pawr_attr_handle);

        k_sem_give(&sem_discovered);
    }

    return BT_GATT_ITER_STOP;
}

static void write_func(struct bt_conn *conn, uint8_t err, struct bt_gatt_write_params *params)
{
    if (err) {
        printk("Write failed (err %d)\n", err);
        return;
    }

    k_sem_give(&sem_written);
}

#define USER_BUTTON DK_BTN1_MSK

static void button_changed(uint32_t button_state, uint32_t has_changed)
{
    if (has_changed & USER_BUTTON) {
        uint32_t user_button_state = button_state & USER_BUTTON;

        if (user_button_state) {
            printk("Button pressed\n");
            // No need to toggle commands now as we only have one fixed payload
        }
    }
}

static int init_button(void)
{
    int err;

    err = dk_buttons_init(button_changed);
    if (err) {
        printk("Cannot init buttons (err: %d)\n", err);
    }

    return err;
}

// Reset the command to fixed payload request
static void cmd_reset_timeout(struct k_timer *timer)
{
    current_pawr_command = PAWR_CMD_FIXED_PAYLOAD;
    printk("Reset command to FIXED_PAYLOAD\n");
}

// Initialize the sensor trackers
static void init_sensor_trackers(void)
{
    // For simplicity, we pre-populate the device IDs
    // In a real implementation, you might discover these dynamically
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensor_trackers[i].device_id = i + 1;  // Device IDs are 1-based
        sensor_trackers[i].data_received = false;
        sensor_trackers[i].response_count = 0; // Reset response count
        sensor_trackers[i].last_heard_time = 0;
    }
    
    // Start in data collection phase
    operation_phase = PHASE_DATA_COLLECTION;
    
    // Start the collection timer
    k_timer_start(&collection_timer, K_MSEC(SENSOR_COLLECTION_TIMEOUT_MS), K_NO_WAIT);
}

// Add restart_scan_handler forward declaration
static void restart_scan_handler(struct k_work *work);

// Define the work item
K_WORK_DELAYABLE_DEFINE(restart_scan_work, restart_scan_handler);

// Update collection_timeout_handler to transition to SLEEP_PENDING state
static void collection_timeout_handler(struct k_timer *timer)
{
    printk("Sensor collection timeout reached (30 seconds). Moving to sleep command phase.\n");
    
    // Switch to sleep command phase
    operation_phase = PHASE_SLEEP_COMMAND;
    current_state = STATE_SLEEP_PENDING;
    
    // Reset index for sleep commands
    next_sleep_cmd_idx = 0;
    
    // Start sending sleep commands
    k_timer_start(&sleep_cmd_timer, K_NO_WAIT, K_NO_WAIT);
}

// Update send_next_sleep_cmd to transition to SLEEPING state
static void send_next_sleep_cmd(struct k_timer *timer)
{
    bool cmd_sent = false;
    
    // Find the next sensor that has reported data
    while (next_sleep_cmd_idx < NUM_SENSORS && !cmd_sent) {
        if (sensor_trackers[next_sleep_cmd_idx].data_received) {
            uint8_t device_id = sensor_trackers[next_sleep_cmd_idx].device_id;
            
            printk("Sending sleep command to sensor node #%d\n", device_id);
            
            // Change command to sleep request
            current_pawr_command = PAWR_CMD_SLEEP_REQUEST;
            
            // Reset command after a short delay
            k_timer_start(&cmd_reset_timer, K_MSEC(300), K_NO_WAIT);
            
            cmd_sent = true;
        }
        
        next_sleep_cmd_idx++;
    }
    
    // If we've sent a command, schedule the next one with a short delay
    if (cmd_sent && next_sleep_cmd_idx < NUM_SENSORS) {
        k_timer_start(&sleep_cmd_timer, K_MSEC(SLEEP_CMD_INTERVAL_MS), K_NO_WAIT);
    } else {
        // Record the exact time we finish sending all sleep commands
        // This will be our reference for calculating the true sleep duration
        uint32_t sleep_start_time = k_uptime_get_32();
        
        // We're done sending sleep commands, enter full sleep mode
        printk("All sleep commands sent at timestamp %u ms. Entering sleep period for %d seconds.\n", 
               sleep_start_time, SENSOR_SLEEP_DURATION_S);
        
        // Enter sleeping state - we'll ignore new sensors during this time
        current_state = STATE_SLEEPING;
        
        // Start a timer for the sleep period
        k_timer_start(&sleep_cycle_timer, K_SECONDS(SENSOR_SLEEP_DURATION_S), K_NO_WAIT);
        
        // Calculate a more accurate wake time to ensure sync with sensors
        // We want to wake up slightly before the sensors do, so we can be ready to scan
        // when they start advertising
        uint32_t elapsed_ms = k_uptime_get_32() - sleep_start_time;
        uint32_t adjusted_sleep_ms = (SENSOR_SLEEP_DURATION_S * 1000) - elapsed_ms;
        
        // Ensure we don't schedule with negative time
        if (adjusted_sleep_ms > (SENSOR_SLEEP_DURATION_S * 1000)) {
            adjusted_sleep_ms = (SENSOR_SLEEP_DURATION_S * 1000);
        }
        
        // Wake up 500ms before the sensors should wake up
        if (adjusted_sleep_ms > 500) {
            adjusted_sleep_ms -= 500;
        }
        
        printk("Scheduling wake-up in %u ms (adjusted for command transmission time)\n", 
               adjusted_sleep_ms);
        
        // When sleep period is over, we'll start scanning again
        k_work_schedule(&restart_scan_work, K_MSEC(adjusted_sleep_ms));
    }
}

// Improve the restart_scan_handler function for better synchronization
static void restart_scan_handler(struct k_work *work)
{
    int err;
    
    printk("Sleep period of %d seconds completed. Preparing to restart scan cycle...\n", 
           SENSOR_SLEEP_DURATION_S);
    
    // Reset all tracker data for next cycle
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensor_trackers[i].data_received = false;
        sensor_trackers[i].response_count = 0; // Reset response count for the new cycle
    }
    
    // Return to data collection phase
    operation_phase = PHASE_DATA_COLLECTION;
    current_state = STATE_SCANNING;
    
    // Add a delay BEFORE starting the scan to allow sensors time to wake up and start advertising
    // This is critical for synchronization - sensors may need time to initialize their advertisers
    printk("Waiting 1 second for sensors to initialize advertising...\n");
    k_sleep(K_SECONDS(1));
    
    printk("Starting collection timer for %d ms...\n", SENSOR_COLLECTION_TIMEOUT_MS);
    // Start the collection timer for the next cycle with a slightly longer timeout
    // to account for the extra initialization time
    k_timer_start(&collection_timer, K_MSEC(SENSOR_COLLECTION_TIMEOUT_MS + 1000), K_NO_WAIT);
    
    // First, attempt to stop any ongoing scan (ignore errors)
    bt_le_scan_stop();
    
    // Add a small delay to ensure BT controller has time to reset
    k_sleep(K_MSEC(100));
    
    // Start scanning for devices with retry mechanism
    int retry_count = 0;
    const int max_retries = 3;
    bool scan_started = false;
    
    while (!scan_started && retry_count < max_retries) {
        err = bt_le_scan_start(&scan_param, device_found);
    if (err) {
            retry_count++;
            printk("Scanning failed to start (err %d), retry %d of %d\n", 
                   err, retry_count, max_retries);
            
            // Wait a bit longer between retries
            k_sleep(K_MSEC(200 * retry_count));
            
            // If this is the last retry, we'll continue with the cycle anyway
            if (retry_count >= max_retries) {
                printk("Failed to restart scanning after multiple attempts\n");
                
                // Even though scanning failed, we'll continue with the cycle
                // The collection timeout will eventually trigger and move to sleep phase
                scan_started = true;
            }
        } else {
            printk("Started new scan cycle to collect sensor data\n");
            scan_started = true;
        }
    }
}

// Update response_cb to track sensors only in scanning state and limit to 10 responses
static void response_cb(struct bt_le_ext_adv *adv, struct bt_le_per_adv_response_info *info,
             struct net_buf_simple *buf)
{
    if (buf) {
        // Turn on LED2 to indicate data reception
        dk_set_led_on(DK_LED2);

        printk("\nReceived Response in Subevent #%d, Response Slot #%d [Data length = %d bytes]\n", info->subevent, info->response_slot, buf->len);

        // Only process responses when in scanning state
        if (current_state == STATE_SCANNING) {
            // Validate the data format first
            if ((buf->len == RESPONSE_DATA_SIZE+1)
                && (buf->data[0] == RESPONSE_DATA_SIZE) 
                && buf->data[1] == 0xFF // Manufacturer Specific Data type (0xFF)
                && (buf->data[2] == (NOVEL_BITS_COMPANY_ID & 0xFF)) && (buf->data[3] == (NOVEL_BITS_COMPANY_ID >> 8)))
            {
                uint8_t device_id = buf->data[4];
                uint32_t fixed_payload = *(uint32_t *)&(buf->data[5]);

                // Check if sensor ID is valid
                if (device_id > 0 && device_id <= NUM_SENSORS) {
                    int idx = device_id - 1;  // Convert to 0-based index

                    // Check if we have already received 10 responses from this sensor
                    if (sensor_trackers[idx].response_count >= 10) {
                        printk("Sensor #%d already reported 10 times. Ignoring further data this cycle.\n", device_id);
                    } else {
                        // Increment response count
                        sensor_trackers[idx].response_count++;

                        printk("\t---- SENSOR NODE #%d (Response %d/10) ----\n\tFixed Payload: 0x%08X\n", 
                               device_id, sensor_trackers[idx].response_count, fixed_payload);
                        
                        // Store the received value (can be overwritten by subsequent responses)
                        sensor_fixed_values[idx] = fixed_payload;

                        // Mark data as received (at least once)
                        sensor_trackers[idx].data_received = true; 
                        sensor_trackers[idx].device_id = device_id; // Ensure device ID is set
                        sensor_trackers[idx].last_heard_time = k_uptime_get_32();
                        
                        printk("Marked sensor #%d as received (response %d/10)\n", 
                               device_id, sensor_trackers[idx].response_count);
                        
                        // If we've heard from all sensors (10 times each) and we're in data collection phase,
                        // move to sleep command phase
                        if (operation_phase == PHASE_DATA_COLLECTION && all_sensors_reported()) {
                            printk("All sensors have reported 10 times each. Moving to sleep command phase.\n");
                            
                            // Cancel the collection timeout timer
                            k_timer_stop(&collection_timer);
                            
                            // Switch to sleep command phase
                            operation_phase = PHASE_SLEEP_COMMAND;
                            current_state = STATE_SLEEP_PENDING;
                            
                            // Reset index for sleep commands
                            next_sleep_cmd_idx = 0;
                            
                            // Start sending sleep commands with a short delay
                            k_timer_start(&sleep_cmd_timer, K_MSEC(500), K_NO_WAIT);
                        }
                    }
                } else {
                    printk("Invalid device ID %d received\n", device_id);
                }
            }
            else
            {
                printk("Invalid data format received\n");
            }
        }

        // After processing data, turn off LED2
        k_sleep(K_MSEC(50));  // Brief flash
        dk_set_led_off(DK_LED2);
    }
    else 
    {
        printk("Failed to receive response: subevent %d, slot %d\n", info->subevent,
               info->response_slot);
    }
}

// Check if all sensors have reported 10 TIMES (not just once)
static bool all_sensors_reported(void)
{
    bool all_reported = true;
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        // If this sensor is expected (device_id is set) and hasn't reported data 10 times
        if (sensor_trackers[i].device_id != 0 && sensor_trackers[i].response_count < 10) {
            all_reported = false;
            break;
        }
    }
    
    return all_reported;
}

// Update device_found to more robustly handle scan stop
static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
             struct net_buf_simple *ad)
{
    int err;
    char addr_str[BT_ADDR_LE_STR_LEN];
    custom_adv_data_t device_ad_data;

    // Skip device processing if we're not in scanning state
    if (current_state != STATE_SCANNING) {
        return;
    }

    // If we already have a connection, don't try to establish another one
    if (central_conn) {
        return;
    }

    // Print out the advertisement type for debugging
    printk("Advertisement type: 0x%02X\n", type);

    /* Accept both regular and extended advertising */
    if (type != BT_GAP_ADV_TYPE_ADV_IND && 
        type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND &&
        type != BT_GAP_ADV_TYPE_EXT_ADV) {
        return;
    }

    (void)memset(&device_ad_data, 0, sizeof(device_ad_data));
    bt_data_parse(ad, data_cb, &device_ad_data);

    if (!device_ad_data.novelbits_id_present) {
        return;
    }

    printk("Device found: %s (RSSI %d)\n", device_ad_data.name, rssi);
    printk("Manufacturer specific data [Novel Bits]. ID = 0x%02X\n", device_ad_data.data[2]);

    if (device_ad_data.data[2] > NUM_SENSORS) {
        printk("Invalid device ID\n");
        return;
    }

    // Try to stop scanning, but handle failure gracefully
    err = bt_le_scan_stop();
    if (err) {
        printk("Failed to stop scanning (err %d), will try again\n", err);
        
        // Try briefly waiting and stopping again
        k_sleep(K_MSEC(50));
        err = bt_le_scan_stop();
        
    if (err) {
            printk("Still failed to stop scanning (err %d), continuing anyway\n", err);
            // We'll continue despite the error
        }
    }

    // Convert the address to string for logging
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

    // Create extended advertising connection parameters with Coded PHY
    struct bt_conn_le_create_param *conn_params = 
        BT_CONN_LE_CREATE_PARAM(BT_CONN_LE_OPT_CODED | BT_CONN_LE_OPT_NO_1M,
                               BT_GAP_SCAN_FAST_INTERVAL,
                               BT_GAP_SCAN_FAST_INTERVAL);

    err = bt_conn_le_create(addr, conn_params,
                BT_LE_CONN_PARAM_DEFAULT,
                &central_conn);
    if (err) {
        printk("Create conn to %s failed (%d)\n", addr_str, err);
        
        // If connection creation fails, we should restart scanning
        // but with a small delay to allow the controller to reset
        k_sleep(K_MSEC(100));
        err = bt_le_scan_start(&scan_param, device_found);
        if (err) {
            printk("Failed to restart scanning after connection failure (err %d)\n", err);
        }
    } else {
        // Successfully initiated connection
        currently_connected_device_id = device_ad_data.data[2];
        printk("Connecting to device ID %d at %s\n", 
               currently_connected_device_id, addr_str);
    }
}

// GATT Definitions

// Strings for the User Descriptions
#define SENSOR_1_DATA_DESCRIPTION "Sensor 1 Data"
#define SENSOR_2_DATA_DESCRIPTION "Sensor 2 Data"
#define SENSOR_3_DATA_DESCRIPTION "Sensor 3 Data"

static ssize_t read_sensor_1_data(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &sensor_fixed_values[0], sizeof(uint32_t));
}

static ssize_t read_sensor_2_data(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &sensor_fixed_values[1], sizeof(uint32_t));
}

static ssize_t read_sensor_3_data(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &sensor_fixed_values[2], sizeof(uint32_t));
}

// Format for the fixed data characteristic
static const struct bt_gatt_cpf fixed_data_cpf = {
    .format = 0x08, // uint32
    .unit = 0x2700,  // unitless
};

// Service UUID: 12345678-1234-5678-1234-56789abcdef2
static struct bt_uuid_128 ws_service_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef2));

// ----- Characteristic UUIDs -----
// Sensor 1 Data: 12345678-1234-5678-1234-56789abcdef3
static struct bt_uuid_128 ws_sensor_1_data_char_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef3));

// Sensor 2 Data: 12345678-1234-5678-1234-56789abcdef4
static struct bt_uuid_128 ws_sensor_2_data_char_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef4));

// Sensor 3 Data: 12345678-1234-5678-1234-56789abcdef5
static struct bt_uuid_128 ws_sensor_3_data_char_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef5));

// --- Weather Station Service ---
// The handles are in the following sequence:
// [0] Weather Station Service
// [1] Sensor 1 Data characteristic declaration
// [2] Sensor 1 Data characteristic value
// [3] Sensor 1 Data characteristic client configuration descriptor (CCC)
// [4] Sensor 1 Data characteristic presentation format descriptor (CPF)
// [5] Sensor 1 Data characteristic user description (CUD)
// [6] Sensor 2 Data characteristic declaration
// [7] Sensor 2 Data characteristic value
// [8] Sensor 2 Data characteristic client configuration descriptor (CCC)
// [9] Sensor 2 Data characteristic presentation format descriptor (CPF)
// [10] Sensor 2 Data characteristic user description (CUD)
// [11] Sensor 3 Data characteristic declaration
// [12] Sensor 3 Data characteristic value
// [13] Sensor 3 Data characteristic client configuration descriptor (CCC)
// [14] Sensor 3 Data characteristic presentation format descriptor (CPF)
// [15] Sensor 3 Data characteristic user description (CUD)
BT_GATT_SERVICE_DEFINE(
    ws_svc,
    
    // Simple Service
    BT_GATT_PRIMARY_SERVICE(&ws_service_uuid.uuid),

    // Sensor 1 Data Characteristic  [1,2]
    BT_GATT_CHARACTERISTIC(&ws_sensor_1_data_char_uuid.uuid,
                    BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
                    BT_GATT_PERM_READ,
                    read_sensor_1_data,
                    NULL,
                    &sensor_fixed_values[0]),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CPF(&fixed_data_cpf),
    BT_GATT_CUD(SENSOR_1_DATA_DESCRIPTION, BT_GATT_PERM_READ),

    // Sensor 2 Data Characteristic  [6,7]
    BT_GATT_CHARACTERISTIC(&ws_sensor_2_data_char_uuid.uuid,
                    BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
                    BT_GATT_PERM_READ,
                    read_sensor_2_data,
                    NULL,
                    &sensor_fixed_values[1]),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),    
    BT_GATT_CPF(&fixed_data_cpf),
    BT_GATT_CUD(SENSOR_2_DATA_DESCRIPTION, BT_GATT_PERM_READ),

    // Sensor 3 Data Characteristic [11, 12]
    BT_GATT_CHARACTERISTIC(&ws_sensor_3_data_char_uuid.uuid,
                    BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
                    BT_GATT_PERM_READ,
                    read_sensor_3_data,
                    NULL,
                    &sensor_fixed_values[2]),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CPF(&fixed_data_cpf),
    BT_GATT_CUD(SENSOR_3_DATA_DESCRIPTION, BT_GATT_PERM_READ),
);

// Response data format
struct response_data {
    uint8_t len;
    uint8_t type;
    uint16_t company_id;
    uint8_t device_id;
    uint32_t fixed_payload;  // Fixed 31-bit payload (actually using 32 bits)
} __packed;

// Add request_cb to ignore sensors during sleep period
static void request_cb(struct bt_le_ext_adv *adv, const struct bt_le_per_adv_data_request *request)
{
    int err;
    uint8_t to_send;
    struct net_buf_simple *buf;

    // Ignore requests if we're in sleep mode
    if (current_state == STATE_SLEEPING) {
        return;
    }

    to_send = MIN(request->count, ARRAY_SIZE(subevent_data_params));

    for (size_t i = 0; i < to_send; i++) {
        buf = &bufs[i];
        buf->data[buf->len - 1] = current_pawr_command;

        subevent_data_params[i].subevent =
            (request->start + i) % per_adv_params.num_subevents;
        subevent_data_params[i].response_slot_start = 0;
        subevent_data_params[i].response_slot_count = NUM_RSP_SLOTS;
        subevent_data_params[i].data = buf;
    }

    err = bt_le_per_adv_set_subevent_data(adv, to_send, subevent_data_params);
    if (err) {
        printk("Failed to set subevent data (err %d)\n", err);
    }
}

int main(void)
{
    int err;
    struct bt_le_ext_adv *pawr_adv;
    struct bt_gatt_discover_params discover_params;
    struct bt_gatt_write_params write_params;
    struct pawr_timing sync_config = {0};  // Initialize with zeros

    init_bufs();
    init_button();
    init_sensor_trackers();  // Initialize sensor tracking
    
    // Initialize LEDs
    err = dk_leds_init();
    if (err) {
        printk("LEDs init failed (err: %d)\n", err);
    }
    
    // Turn off all LEDs initially
    dk_set_leds(0x00);

    printk("Starting Periodic Advertising Demo\n");

    /* Initialize the Bluetooth Subsystem */
    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return 0;
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err && err != -EALREADY) {
        printk("Advertising failed to start (err %d) @ %d\n", err, __LINE__);

        return 0;
    }

    /* Create a non-connectable non-scannable advertising set */
    err = bt_le_ext_adv_create(BT_LE_EXT_ADV_NCONN, &adv_cb, &pawr_adv);
    if (err) {
        printk("Failed to create advertising set (err %d)\n", err);
        return 0;
    }

    /* Set periodic advertising parameters */
    err = bt_le_per_adv_set_param(pawr_adv, &per_adv_params);
    if (err) {
        printk("Failed to set periodic advertising parameters (err %d)\n", err);
        return 0;
    }

    /* Enable Periodic Advertising */
    err = bt_le_per_adv_start(pawr_adv);
    if (err) {
        printk("Failed to enable periodic advertising (err %d)\n", err);
        return 0;
    }

    printk("Start Periodic Advertising\n");
    err = bt_le_ext_adv_start(pawr_adv, BT_LE_EXT_ADV_START_DEFAULT);
    if (err) {
        printk("Failed to start extended advertising (err %d)\n", err);
        return 0;
    }
    
    // Set initial state to scanning and start the collection timer
    current_state = STATE_SCANNING;
    k_timer_start(&collection_timer, K_MSEC(SENSOR_COLLECTION_TIMEOUT_MS), K_NO_WAIT);

    printk("----------- Starting main cycle -----------\n");
    printk("Scanning period: %d ms, Sleep period: %d s\n", 
           SENSOR_COLLECTION_TIMEOUT_MS, SENSOR_SLEEP_DURATION_S);

    while (1) {
        // Only scan for devices when in scanning state
        if (current_state == STATE_SCANNING) {
            // Try to stop any existing scan first
            bt_le_scan_stop();
            
            // Small delay to ensure controller is ready
            k_sleep(K_MSEC(50));
            
            // Log the start of scanning phase with timestamp
            printk("\n*** SCAN PHASE START at %u ms ***\n", k_uptime_get_32());
            
            // Start with a retry mechanism in case of error
            int retry_count = 0;
            bool scan_started = false;
            
            while (!scan_started && retry_count < 3) {
        err = bt_le_scan_start(&scan_param, device_found);
        if (err) {
                    retry_count++;
                    printk("Scanning failed to start (err %d), retry %d of 3\n", err, retry_count);
                    k_sleep(K_MSEC(100 * retry_count));
                } else {
                    scan_started = true;
                    printk("Scanning successfully started for %d ms collection period\n", 
                           SENSOR_COLLECTION_TIMEOUT_MS);
                }
            }
            
            // If we couldn't start scanning after retries, we'll wait and try again in the next cycle
            if (!scan_started) {
                printk("Failed to start scanning after multiple attempts, will retry in next cycle\n");
                // Move directly to sleep phase
                current_state = STATE_SLEEPING;
                k_timer_start(&sleep_cycle_timer, K_SECONDS(SENSOR_SLEEP_DURATION_S), K_NO_WAIT);
                k_work_schedule(&restart_scan_work, K_SECONDS(SENSOR_SLEEP_DURATION_S));
                continue;
            }
            
            // Periodically check if we've found any sensors during the scan period
            int check_count = 0;
            const int max_checks = 10;
            bool any_sensor_found = false;
            
            // Check every second for up to 10 seconds (if no device connects first)
            while (check_count < max_checks && !any_sensor_found && central_conn == NULL) {
                // Check if we've found any sensor so far
                for (int i = 0; i < NUM_SENSORS; i++) {
                    if (sensor_trackers[i].data_received) {
                        any_sensor_found = true;
                        printk("Found sensor #%d during periodic check\n", i+1);
                        break;
                    }
                }
                
                if (!any_sensor_found) {
                    check_count++;
                    printk("No sensors found yet, check %d of %d\n", check_count, max_checks);
                    k_sleep(K_SECONDS(1));
                }
            }
            
            // Wait for connection or timeout
            k_sem_take(&sem_connected, check_count < max_checks ? K_FOREVER : K_NO_WAIT);

        err = bt_le_per_adv_set_info_transfer(pawr_adv, central_conn, 0);
        if (err) {
            printk("Failed to send PAST (err %d)\n", err);
        }
        else // PAST sent successfully
        {
            printk("PAST sent\n");
            
            discover_params.uuid = &pawr_char_uuid.uuid;
            discover_params.func = discover_func;
            discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
            discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
            discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
            err = bt_gatt_discover(central_conn, &discover_params);
            if (err) {
                printk("Discovery failed (err %d)\n", err);
            }
            else // Discovery started successfully
            {
                printk("Discovery started\n");

                err = k_sem_take(&sem_discovered, K_SECONDS(10));
                if (err) {
                    printk("Timed out during GATT discovery\n");
                }
                else // Discovery completed successfully
                {
                    if (currently_connected_device_id == 0xFF) {
                        printk("No device ID found\n");
                    }
                    else // Device ID found
                    {
                        printk("Device ID: %d --> PAST sent for subevent %d and response slot %d\n",
                                currently_connected_device_id,
                                sync_config.subevent,
                                sync_config.response_slot);

                        // Give the device time to process the PAST and sync before writing the PAwR config
                        k_msleep(2*per_adv_params.interval_min);        

                        // Assign the subevent and response slot to the sync_config
                        // [Note: each node will be assigned a subevent based on their device ID. Response slot will be 0 for all nodes]
                        sync_config.subevent = currently_connected_device_id-1;
                        sync_config.response_slot = 0;

                        write_params.func = write_func;
                        write_params.handle = pawr_attr_handle;
                        write_params.offset = 0;
                        write_params.data = &sync_config;
                        write_params.length = sizeof(sync_config);

                        err = bt_gatt_write(central_conn, &write_params);
                        if (err) {
                            printk("Write failed (err %d)\n", err);
                        }
                        else // Write started successfully
                        {
                            printk("Write started\n");

                            err = k_sem_take(&sem_written, K_SECONDS(10));
                            if (err) {
                                printk("Timed out during GATT write\n");
                            }
                            else // Write completed successfully
                            {
                                printk("PAwR config written to device %d, disconnecting\n", currently_connected_device_id);
                            }
                        }
                    }
                }
            }
        }

        // Disconnect from the device
        err = bt_conn_disconnect(central_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        if (err) {
            printk("Failed to disconnect (err %d)\n", err);
            return 0;
        }

        printk("Disconnected\n");
            k_sem_take(&sem_disconnected, K_SECONDS(30));
        } else {
            // If not in scanning state, we're either sending sleep commands or sleeping
            // Just wait a bit before checking again
            k_sleep(K_MSEC(500));
        }
    }

    printk("SHOULD NEVER REACH HERE\n");

    return 0;
}