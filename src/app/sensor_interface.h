#ifndef SENSOR_INTERFACE_H
#define SENSOR_INTERFACE_H

typedef struct sensor_data_s
{
    // Fixed temperature value (no longer from actual sensor)
    int temperature;
    
    // Fixed humidity value (no longer from actual sensor)
    int humidity;
} sensor_data_t;

void sensor_capture_data(void);

#endif