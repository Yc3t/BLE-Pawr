/*
 * Copyright (c) 2024 Novel Bits, LLC
 *
 * SPDX-License-Identifier: MIT License
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "sensor_tracker.h"

// Tracker array for all sensors
static sensor_tracker_t sensor_trackers[NUM_SENSORS] = {0};

// Array to store the last received data from each sensor
static uint32_t sensor_data[NUM_SENSORS] = {0};

// Initialize the sensor trackers
void sensor_tracker_init(void)
{
    // For simplicity, we pre-populate the device IDs
    // In a real implementation, you might discover these dynamically
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensor_trackers[i].device_id = i + 1;  // Device IDs are 1-based
        sensor_trackers[i].data_received = false;
        sensor_trackers[i].last_heard_time = 0;
        sensor_data[i] = 0;
    }
    
    printk("Sensor tracker initialized for %d sensors\n", NUM_SENSORS);
}

// Check if all sensors have reported their data
bool sensor_tracker_all_reported(void)
{
    bool all_reported = true;
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        // If this sensor is active and hasn't reported data, we're not done
        if (sensor_trackers[i].device_id != 0 && !sensor_trackers[i].data_received) {
            all_reported = false;
            break;
        }
    }
    
    return all_reported;
}

// Mark a sensor as having reported data
void sensor_tracker_set_data_received(uint8_t device_id, uint32_t data)
{
    if (device_id > 0 && device_id <= NUM_SENSORS) {
        int idx = device_id - 1;  // Convert to 0-based index
        sensor_trackers[idx].device_id = device_id;
        sensor_trackers[idx].data_received = true;
        sensor_trackers[idx].last_heard_time = k_uptime_get_32();
        sensor_data[idx] = data;
        
        printk("Marked sensor #%d as received with data: 0x%08X\n", device_id, data);
    } else {
        printk("Invalid device ID: %d\n", device_id);
    }
}

// Reset data received flags for all sensors
void sensor_tracker_reset_flags(void)
{
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensor_trackers[i].data_received = false;
    }
    
    printk("All sensor data flags reset\n");
}

// Get sensor data for a specific device
uint32_t sensor_tracker_get_data(uint8_t device_id)
{
    if (device_id > 0 && device_id <= NUM_SENSORS) {
        return sensor_data[device_id - 1];
    }
    
    return 0;
}

// Get number of configured sensors
uint8_t sensor_tracker_get_count(void)
{
    return NUM_SENSORS;
} 