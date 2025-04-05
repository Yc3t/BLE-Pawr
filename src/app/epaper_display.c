#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "epaper_display.h"

void epaper_display_update_sensor_data(void)
{
    // Dummy function - display not used in simplified version
    printk("Display update skipped (disabled in simplified version)\n");
}

void epaper_display_init(void)
{
    // Dummy function - display not used in simplified version
    printk("Display disabled in simplified version\n");
}