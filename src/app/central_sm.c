/*
 * Copyright (c) 2024 Novel Bits, LLC
 *
 * SPDX-License-Identifier: MIT License
 */

#include <zephyr/kernel.h>
#include <dk_buttons_and_leds.h>
#include "central_sm.h"

// Current state of the central state machine
static central_state_t current_state = CENTRAL_STATE_SCANNING;

// Timer for state timeouts
K_TIMER_DEFINE(state_timer, NULL, NULL);

// Initialize the central state machine
void central_sm_init(void)
{
    // Start in scanning state
    current_state = CENTRAL_STATE_SCANNING;
    
    // Indicate scanning state with LED3
    dk_set_led_on(DK_LED3);
    dk_set_led_off(DK_LED4);
}

// Get the current state of the central state machine
central_state_t central_sm_get_state(void)
{
    return current_state;
}

// Set the state of the central state machine
void central_sm_set_state(central_state_t state)
{
    // Only change state if it's different
    if (current_state != state) {
        current_state = state;
        
        // Update LEDs based on state
        switch (state) {
            case CENTRAL_STATE_SCANNING:
                // Scanning state: LED3 on, LED4 off
                dk_set_led_on(DK_LED3);
                dk_set_led_off(DK_LED4);
                printk("Central State: SCANNING\n");
                break;
                
            case CENTRAL_STATE_SLEEPING:
                // Sleeping state: LED3 off, LED4 on
                dk_set_led_off(DK_LED3);
                dk_set_led_on(DK_LED4);
                printk("Central State: SLEEPING\n");
                break;
                
            case CENTRAL_STATE_WAITING:
                // Waiting state: Both LED3 and LED4 on
                dk_set_led_on(DK_LED3);
                dk_set_led_on(DK_LED4);
                printk("Central State: WAITING\n");
                break;
                
            default:
                break;
        }
    }
}

// Run the state machine logic
int central_sm_run(void)
{
    // The actual state logic will be handled in central.c
    // This function is just a placeholder for any state-specific processing
    return 0;
}

// Check if a timeout has occurred in the current state
bool central_sm_is_timed_out(void)
{
    return k_timer_status_get(&state_timer) > 0;
}

// Set timeout for the current state
void central_sm_set_timeout(uint32_t timeout_ms)
{
    k_timer_start(&state_timer, K_MSEC(timeout_ms), K_NO_WAIT);
} 