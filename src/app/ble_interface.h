#ifndef BLE_INTERFACE_H_
#define BLE_INTERFACE_H_

#include <stdbool.h> // Include for bool type

void ble_init(void);

void ble_start_advertising(void);

// Register callbacks for periodic adv sync
void ble_register_periodic_adv_sync_callbacks(void);

void ble_delete_per_adv_sync(void);

bool ble_check_and_clear_sleep_request(void);

// Function to get the requested sleep duration
uint32_t ble_get_sleep_duration(void);

// Function to check and clear the TX success signal
bool ble_check_and_clear_tx_success_signal(void);

// Function to enter sleep period
void ble_enter_sleep_period(uint32_t duration_s);

// Function to handle sleep requests in a synchronized way
void handle_sleep_request(void);

#endif