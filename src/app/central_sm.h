/*
 * Copyright (c) 2024 Novel Bits, LLC
 *
 * SPDX-License-Identifier: MIT License
 */

#ifndef CENTRAL_SM_H
#define CENTRAL_SM_H

#include <zephyr/kernel.h>

// Define the central states
typedef enum {
    CENTRAL_STATE_SCANNING,    // Actively scanning for sensor nodes
    CENTRAL_STATE_SLEEPING,    // Sensors are sleeping as commanded
    CENTRAL_STATE_WAITING      // Waiting for a condition or timeout
} central_state_t;

// Initialize the central state machine
void central_sm_init(void);

// Get the current state of the central state machine
central_state_t central_sm_get_state(void);

// Set the state of the central state machine
void central_sm_set_state(central_state_t state);

// Run the state machine logic
int central_sm_run(void);

// Check if a timeout has occurred in the current state
bool central_sm_is_timed_out(void);

// Set timeout for the current state
void central_sm_set_timeout(uint32_t timeout_ms);

#endif /* CENTRAL_SM_H */ 