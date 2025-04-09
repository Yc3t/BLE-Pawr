/*
 * Copyright (c) 2024 Novel Bits, LLC
 *
 * SPDX-License-Identifier: MIT License
 */

#ifndef SENSOR_TRACKER_H
#define SENSOR_TRACKER_H

#include <zephyr/kernel.h>
#include <stdbool.h>

// Number of sensors to track
#define NUM_SENSORS 3

// Timeout for waiting for all sensors (in ms)
#define SENSOR_COLLECTION_TIMEOUT_MS 5000

// Sleep duration for sensors (in seconds)
#define SENSOR_SLEEP_DURATION_S 10

// Define a structure to track sensor data collection
typedef struct {
    bool data_received;        // Whether we've received data from this sensor
    uint32_t last_heard_time;  // Last time we heard from this sensor
    uint8_t device_id;         // Device ID of the sensor
} sensor_tracker_t;

// Initialize the sensor trackers
void sensor_tracker_init(void);

// Check if all sensors have reported their data
bool sensor_tracker_all_reported(void);

// Mark a sensor as having reported data
void sensor_tracker_set_data_received(uint8_t device_id, uint32_t data);

// Reset data received flags for all sensors
void sensor_tracker_reset_flags(void);

// Get sensor data for a specific device
uint32_t sensor_tracker_get_data(uint8_t device_id);

// Get number of configured sensors
uint8_t sensor_tracker_get_count(void);

#endif /* SENSOR_TRACKER_H */ 