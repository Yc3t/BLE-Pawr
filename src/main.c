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
#define LOOP_DELAY_MS 100 // Shorter regular loop delay for more responsive sleep detection
#define LED_ACTIVE_INDICATOR DK_LED3

// Sensor data - Replace with fixed payload
uint32_t fixed_payload;

// Add the handle_sleep_request function declaration
extern void handle_sleep_request(void);

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
    
    // Turn on LED3 to indicate active (not sleeping) state
    dk_set_led_on(LED_ACTIVE_INDICATOR);

    // Initialize with fixed sensor data - Replace with fixed payload initialization
    fixed_payload = 0xAABBCC00 | DEVICE_ID; // Example fixed payload
    printk("Fixed payload set to: 0x%08X\n", fixed_payload);

    // Initialize BLE
    ble_init();

    // Register callbacks for periodic adv sync
    ble_register_periodic_adv_sync_callbacks();

    // Initialize State Machine
    sensor_node_sm_init();

    printk("Starting main loop - waiting for connection and data requests\n");

    do {
        // Run the state machine
        if (sensor_node_sm_run())
        {
            /* handle return code and terminate state machine */
            printk("Error running state machine\n");
            break;
        }

        // Check for sleep requests from central and handle them
        handle_sleep_request();
        
        // Check if data was successfully transmitted
        if (ble_check_and_clear_tx_success_signal()) {
            // Indicate success with a brief LED flash
            dk_set_led_on(DK_LED2);
            k_sleep(K_MSEC(50));
            dk_set_led_off(DK_LED2);
            
            printk("Data transmitted successfully\n");
        }
        
        // Short delay for main loop
        k_msleep(LOOP_DELAY_MS);

    } while (true);

    return 0;
}
