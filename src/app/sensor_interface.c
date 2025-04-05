#include "sensor_interface.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

// Define fixed sensor data
sensor_data_t sensor_data;

void sensor_capture_data(void)
{
    // Set fixed temperature value (20°C)
    sensor_data.temperature = 20;
    
    // Set fixed humidity value (50%)
    sensor_data.humidity = 50;
    
    printk("Fixed sensor data: Temp=%d°C, Humidity=%d%%\n", 
           sensor_data.temperature, sensor_data.humidity);
}