#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/sys/util.h>
#include <dk_buttons_and_leds.h>
#include <stdbool.h> // Include for bool type

#include "device_id.h"
#include "ble_interface.h"
#include "sensor_node_sm.h"
// #include "sensor_interface.h" // Remove this include

// Remove old sensor data dependency
// extern sensor_data_t sensor_data;
// Add fixed payload dependency
extern uint32_t fixed_payload;

// PAwR Request Type Definitions
// Remove old commands
// #define PAWR_CMD_REQUEST_TEMP 0x01
// #define PAWR_CMD_REQUEST_HUMIDITY 0x02
// Add command expected by collector
#define PAWR_CMD_FIXED_PAYLOAD 0x01
#define PAWR_CMD_SLEEP_REQUEST 0x02

static struct bt_conn *default_conn;
static struct bt_le_per_adv_sync_transfer_param past_param;
static struct bt_le_per_adv_sync *default_sync;

static void sync_cb(struct bt_le_per_adv_sync *sync, struct bt_le_per_adv_sync_synced_info *info);
static void term_cb(struct bt_le_per_adv_sync *sync, const struct bt_le_per_adv_sync_term_info *info);
static void recv_cb(struct bt_le_per_adv_sync *sync, const struct bt_le_per_adv_sync_recv_info *info,
                    struct net_buf_simple *buf);

static struct bt_le_per_adv_sync_cb sync_callbacks = {
    .synced = sync_cb,
    .term = term_cb,
    .recv = recv_cb,
};

static struct __packed {
    uint8_t subevent;
    uint8_t response_slot;
} pawr_timing;

// Update response size for fixed payload structure
// 9 bytes: 1 byte len, 1 byte type, 2 bytes company ID, 1 byte device ID, 1 byte command, 4 bytes fixed payload
#define RESPONSE_DATA_SIZE 9

// Update response data format
struct response_data {
    uint8_t len;
    uint8_t type;
    uint16_t company_id;
    uint8_t device_id;
    uint8_t command; // Keep command field
    // Replace sensor reading with fixed payload
    // int sensor_reading;      // 4 bytes
    // uint32_t padding;        // Additional 4 bytes to match the original 8-byte sensor value size
    uint32_t fixed_payload;  // Fixed 32-bit payload
} __packed;

// Company ID for Novel Bits (used for the manufacturer data)
#define NOVEL_BITS_COMPANY_ID 0x08D3

// Advertising data - Update to match what central.c is looking for
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_NAME_COMPLETE, 'N', 'o', 'v', 'e', 'l', ' ', 'B', 'i', 't', 's'),
    // Critical change: Format manufacturer data exactly as central expects
    BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA, 
                 (NOVEL_BITS_COMPANY_ID & 0xFF),        // LSB first
                 ((NOVEL_BITS_COMPANY_ID >> 8) & 0xFF), // MSB second
                 DEVICE_ID)                             // Device ID third
};

// Flag to signal the main loop to sleep
static volatile bool sleep_requested = false;
// Flag to signal the main loop that Tx was successful
static volatile bool tx_success_signal = false;
// Flag to indicate if the device is in the sleep period (checked by recv_cb)
static volatile bool is_sleeping = false;
// Sleep duration requested by the central
static volatile uint32_t requested_sleep_duration_s = 10; // Default 10 seconds

// Add this to your global variables
static struct bt_le_ext_adv *adv;

// Add to your declarations for the callbacks
static void request_cb(struct bt_le_ext_adv *adv, const struct bt_le_per_adv_data_request *request);
static void response_cb(struct bt_le_ext_adv *adv, struct bt_le_per_adv_response_info *info,
             struct net_buf_simple *buf);

// Define the advertisement callbacks like in peripheral.c
static const struct bt_le_ext_adv_cb adv_cb = {
    .pawr_data_request = request_cb,
    .pawr_response = response_cb,
};

// Add these stub callback implementations (you can leave them empty)
static void request_cb(struct bt_le_ext_adv *adv, const struct bt_le_per_adv_data_request *request)
{
    // Not needed for peripheral role
}

static void response_cb(struct bt_le_ext_adv *adv, struct bt_le_per_adv_response_info *info,
             struct net_buf_simple *buf)
{
    // Not needed for peripheral role
}

void ble_start_advertising(void)
{
    int err;

    // Create proper advertising parameters for a connectable advertisement
    struct bt_le_adv_param param =
        BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_CONNECTABLE |
                     BT_LE_ADV_OPT_EXT_ADV |
                     BT_LE_ADV_OPT_CODED,
                     BT_GAP_ADV_FAST_INT_MIN_2,
                     BT_GAP_ADV_FAST_INT_MAX_2,
                     NULL);

    // Create the advertising set if it doesn't exist
    if (!adv) {
        // Pass adv_cb to register the callbacks
        err = bt_le_ext_adv_create(&param, &adv_cb, &adv);
        if (err) {
            printk("Failed to create advertiser set (err %d)\n", err);
            return;
        }
        
        // Set the advertising data
        err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
        if (err) {
            printk("Failed to set advertising data (err %d)\n", err);
            return;
        }
        
        printk("Created extended advertising set successfully\n");
    }

    // Start advertising
    err = bt_le_ext_adv_start(adv, NULL);
    if (err) {
        printk("Advertising failed to start (err %d)\n", err);
    } else {
        printk("Started advertising with Coded PHY\n");
    }
}

static void sync_cb(struct bt_le_per_adv_sync *sync, struct bt_le_per_adv_sync_synced_info *info)
{
    char le_addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));
    printk("Synced to %s with %d subevents\n", le_addr, info->num_subevents);

    default_sync = sync;

    int err;
    uint8_t subevents[1];
    struct bt_le_per_adv_sync_subevent_params params;   

    params.properties = 0;
    params.num_subevents = 1;
    params.subevents = subevents;
    subevents[0] = pawr_timing.subevent;

    printk("Setting subevent to sync to: %d\n", pawr_timing.subevent);
    printk("Setting response slot to: %d\n", pawr_timing.response_slot);

    err = bt_le_per_adv_sync_subevent(sync, &params);
    if (err) {
        printk("Failed to set subevents to sync to (err %d)\n", err);
    }    

    sensor_node_sm_set_state(Synced);
}

static void term_cb(struct bt_le_per_adv_sync *sync,
            const struct bt_le_per_adv_sync_term_info *info)
{
    char le_addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

    printk("Sync terminated (reason %d)\n", info->reason);

    // Go to NotSynced State
    sensor_node_sm_set_state(NotSynced);
}

static bool parse_ad_field(struct bt_data *data, void *user_data)
{
    uint8_t *request_command = ((uint8_t *)user_data);

    if (data->type == BT_DATA_MANUFACTURER_DATA)
    {
        uint16_t company_id = (data->data[1] << 8) | data->data[0];
        *request_command = data->data[2];

        if (company_id != NOVEL_BITS_COMPANY_ID)
        {
            printk("Found Unexpected Company ID: 0x%04X\n", company_id);
        }
    }
    return true;
}

// Function to set response data
int bt_le_per_adv_set_response_data(struct bt_le_per_adv_sync *per_adv_sync,
                    const struct bt_le_per_adv_response_params *params,
                    const struct net_buf_simple *data);

// Response parameters
static struct bt_le_per_adv_response_params rsp_params;

// Response buffer - adjust size
NET_BUF_SIMPLE_DEFINE_STATIC(rsp_buf, RESPONSE_DATA_SIZE+1);

static void recv_cb(struct bt_le_per_adv_sync *sync,
            const struct bt_le_per_adv_sync_recv_info *info, struct net_buf_simple *buf)
{
    int err;

    // --- Check sleep state FIRST ---
    if (is_sleeping) {
        // If sleeping, ignore the request entirely.
        //printk("Sleeping, ignoring PAwR request.\n"); // Optional: can be noisy
        return;
    }
    // --- End sleep check ---

    if (buf && buf->len) {
        uint8_t request_command;
        struct response_data rsp_data;

        if (info->subevent != pawr_timing.subevent)
        {
            printk("Received indication for subevent %d, but expected subevent %d\n", info->subevent, pawr_timing.subevent);

            // Sync to the correct subevent
            uint8_t subevents[1];
            struct bt_le_per_adv_sync_subevent_params params;
            params.properties = 0;
            params.num_subevents = 1;
            params.subevents = subevents;
            subevents[0] = pawr_timing.subevent;

            err = bt_le_per_adv_sync_subevent(sync, &params);
            if (err) {
                printk("Failed to set subevents to sync to (err %d)\n", err);
            }
            return;
        }

        printk("\n\n[SENSOR NODE: #%d] Indication: subevent %d, responding in slot %d\n", 
                DEVICE_ID,
                info->subevent,
                pawr_timing.response_slot);
        bt_data_parse(buf, parse_ad_field, &request_command);

        // Send back fixed payload based on the request type
        net_buf_simple_reset(&rsp_buf);

        // Set the response data common fields
        rsp_data.len = RESPONSE_DATA_SIZE;
        rsp_data.type = BT_DATA_MANUFACTURER_DATA;
        /* Novel Bits */
        rsp_data.company_id = NOVEL_BITS_COMPANY_ID;
        rsp_data.device_id = DEVICE_ID;
        rsp_data.command = request_command; // Echo back the received command

        // Check for command type (Fixed Payload)
        if (request_command == PAWR_CMD_FIXED_PAYLOAD)
        {
            printk("\tReceived request for fixed payload.\n\tResponding with Payload = 0x%08X\n",
                   fixed_payload);
            // Set the fixed payload
            rsp_data.fixed_payload = fixed_payload;
        }
        // Handle sleep request command
        else if (request_command == PAWR_CMD_SLEEP_REQUEST)
        {
            printk("\tReceived sleep request command!\n");
            // Set the fixed payload (still send the payload)
            rsp_data.fixed_payload = fixed_payload;
            // Set the sleep request flag
            sleep_requested = true;
            
            // Extract sleep duration if provided in the manufacturer data payload
            // We'll use a simplified approach here, where the command byte also carries the sleep duration
            // In a more complex scenario, you might want to encode this in the payload
            uint8_t sleep_duration_byte = 0;
            if (buf->len >= 4) { // Check if there's enough data
                sleep_duration_byte = buf->data[3]; // Use the fourth byte for sleep duration
            }
            
            // If the duration byte is valid (not 0), use it; otherwise use default
            if (sleep_duration_byte > 0) {
                requested_sleep_duration_s = sleep_duration_byte;
                printk("\tSleep duration requested: %d seconds\n", requested_sleep_duration_s);
            } else {
                printk("\tUsing default sleep duration: %d seconds\n", requested_sleep_duration_s);
            }
        }
        // Remove old temperature/humidity logic
        // else if (request_command == PAWR_CMD_REQUEST_HUMIDITY)
        // {
        //     printk("\tReceived request for humidity data.\n\tResponding with Humidity = %d %%\n",
        //            sensor_data.humidity);
        //     rsp_data.sensor_reading = sensor_data.humidity;
        // }
        else
        {
            printk("Received unknown request: 0x%02X\n", request_command);
            return; // Don't respond to unknown commands
        }

        // Assign response data to response buffer
        // Ensure the size matches the updated struct
        net_buf_simple_add_mem(&rsp_buf, &rsp_data, sizeof(rsp_data));

        rsp_params.request_event = info->periodic_event_counter;
        rsp_params.request_subevent = info->subevent;

        /* Respond in current subevent and assigned response slot */
        rsp_params.response_subevent = info->subevent;
        rsp_params.response_slot = pawr_timing.response_slot;

        // Turn on LED2 to indicate data sending
        dk_set_led_on(DK_LED2);

        err = bt_le_per_adv_set_response_data(sync, &rsp_params, &rsp_buf);

        if (!err) {
             printk("Response sent successfully. Signaling main loop for sleep.\n");
             k_sleep(K_MSEC(50)); // Keep brief flash
             dk_set_led_off(DK_LED2);
             // Signal the main loop that Tx was successful
             tx_success_signal = true;
        } else {
            printk("Failed to send response (err %d)\n", err);
            dk_set_led_off(DK_LED2);
        }
    } else if (buf) {
        printk("Received empty indication: subevent %d\n", info->subevent);
    } else {
        printk("Failed to receive indication: subevent %d\n", info->subevent);
    }
}

// Function for main loop to check if Tx was successful
bool ble_check_and_clear_tx_success_signal(void)
{
    bool ret = tx_success_signal;
    if (ret) {
        tx_success_signal = false; // Clear the flag after checking
    }
    return ret;
}

// Function for main loop to check if sleep was requested
bool ble_check_and_clear_sleep_request(void)
{
    bool ret = sleep_requested;
    if (ret) {
        sleep_requested = false; // Clear the flag after checking
    }
    return ret;
}

// Function to get the requested sleep duration
uint32_t ble_get_sleep_duration(void)
{
    return requested_sleep_duration_s;
}

// Function called by main loop to enter the sleep period
void ble_enter_sleep_period(uint32_t duration_s)
{
    printk("Entering sleep period for %d seconds...\n", duration_s);
    is_sleeping = true;
    k_sleep(K_SECONDS(duration_s));
    is_sleeping = false;
    printk("... Woke up from sleep period.\n");
}

static struct bt_uuid_128 pawr_svc_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0));
static struct bt_uuid_128 pawr_char_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1));

static ssize_t write_timing(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                uint16_t len, uint16_t offset, uint8_t flags)
{
    if (offset) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    if (len != sizeof(pawr_timing)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }  

    memcpy(&pawr_timing, buf, len);

    printk("New timing: subevent %d, response slot %d\n", pawr_timing.subevent,
           pawr_timing.response_slot);    

    return len;
}

BT_GATT_SERVICE_DEFINE(pawr_svc, BT_GATT_PRIMARY_SERVICE(&pawr_svc_uuid.uuid),
               BT_GATT_CHARACTERISTIC(&pawr_char_uuid.uuid, BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_WRITE, NULL, write_timing,
                          &pawr_timing));

void connected(struct bt_conn *conn, uint8_t err)
{
    printk("Connected (err 0x%02X)\n", err);

    if (err) {
        default_conn = NULL;
        return;
    }

    // Turn on LED1 to indicate connection
    dk_set_led_on(DK_LED1);

    default_conn = bt_conn_ref(conn);
}

void disconnected(struct bt_conn *conn, uint8_t reason)
{
    bt_conn_unref(default_conn);
    default_conn = NULL;

    // Turn off LED1 when disconnected
    dk_set_led_off(DK_LED1);

    printk("Disconnected (reason 0x%02X)\n", reason);
}

BT_CONN_CB_DEFINE(conn_cb) = {
    .connected = connected,
    .disconnected = disconnected,
};

// Register callbacks for periodic adv sync
void ble_register_periodic_adv_sync_callbacks(void)
{
    int err;

    bt_le_per_adv_sync_cb_register(&sync_callbacks);

    // Subscribe to PAST events
    past_param.skip = 1;
    past_param.timeout = 1000; /* 10 seconds */
    past_param.options = BT_LE_PER_ADV_SYNC_TRANSFER_OPT_NONE;
    err = bt_le_per_adv_sync_transfer_subscribe(NULL, &past_param);
    if (err) {
        printk("PAST subscribe failed (err %d)\n", err);
        return;
    }   
    printk("Subscribed to PAST events\n");
}

void ble_delete_per_adv_sync(void)
{
    int err = bt_le_per_adv_sync_delete(default_sync);
    if (err) {
        printk("Failed to delete sync (err %d)\n", err);
    }
    default_sync = NULL;
}

void ble_init(void)
{
    int err;

    sleep_requested = false; // Ensure flag is clear on init
    tx_success_signal = false; // Ensure flags are clear on init
    is_sleeping = false;

    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }
    
    // Add a small delay to ensure BT is fully initialized
    k_sleep(K_MSEC(100));
}
