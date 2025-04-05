#ifndef BLE_INTERFACE_H_
#define BLE_INTERFACE_H_

#include <stdbool.h> // Include for bool type

void ble_init(void);

void ble_start_advertising(void);

// Register callbacks for periodic adv sync
void ble_register_periodic_adv_sync_callbacks(void);

void ble_delete_per_adv_sync(void);

bool ble_check_and_clear_sleep_request(void);

#endif