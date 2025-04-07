/*
 * Copyright (c) 2024 Novel Bits, LLC
 *
 * SPDX-License-Identifier: MIT License
 */

#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include "app/device_id.h"
#include "app/sensor_node_sm.h"
#include "app/ble_interface.h"

// Define delay for main loop
#define LOOP_DELAY_MS 1000 // Regular loop delay
#define POST_TX_SLEEP_S 10 // Sleep duration after tx

// Sensor data - Replace with fixed payload
// extern sensor_data_t sensor_data;
uint32_t fixed_payload;

int main(void)
{
    int err;
    
    // Initialize USB
    err = usb_enable(NULL);
    if (err) {
        printk("USB init failed (err: %d)\n", err);
    } else {
        printk("USB initialized successfully\n");
    }
    
    printk("Initializing Simplified Beacon [Device #%d]\n", DEVICE_ID);

    // Initialize LEDs
    err = dk_leds_init();
    if (err) {
        printk("LEDs init failed (err: %d)\n", err);
    }
    
    // Turn off all LEDs initially
    dk_set_leds(0x00);

    // Initialize with fixed sensor data - Replace with fixed payload initialization
    // sensor_capture_data();
    fixed_payload = 0xAABBCC00 | DEVICE_ID; // Example fixed payload
    printk("Fixed payload set to: 0x%08X\n", fixed_payload);

    // Initialize BLE
    ble_init();

    // Register callbacks for periodic adv sync
    ble_register_periodic_adv_sync_callbacks();

    // Initialize State Machine
    sensor_node_sm_init();

    do {
        // Run the state machine
        if (sensor_node_sm_run())
        {
            /* handle return code and terminate state machine */
            printk("Error running state machine\n");
            break;
        }

        // Check for sleep requests from central
        if (ble_check_and_clear_sleep_request()) {
            // Enter the sleep period when requested by central
            printk("Sleep requested by central\n");
            ble_enter_sleep_period(POST_TX_SLEEP_S);
        } 
        // Check if data was successfully transmitted but no sleep command
        else if (ble_check_and_clear_tx_success_signal()) {
            // No need to sleep if not requested by central
            printk("Data transmitted successfully, no sleep requested\n");
        } 
        else {
            // Normal operation delay
            k_msleep(LOOP_DELAY_MS);
        }

    } while (true);

    return 0;
}
