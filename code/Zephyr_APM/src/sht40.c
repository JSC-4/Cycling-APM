#include <zephyr/kernel.h>
#include <zephyr.h>     /* Give access to k_msleep() */
#include <device.h>     /* Gives access to device_get_bindings() */
#include <devicetree.h> /* For DT macros */
#include <zephyr/logging/log.h>
#include <drivers/i2c.h>
#include <stdio.h>

#include "sht40.h"

bool sht40_read(const struct device *dev_i2c, sht40_data *data)
{
    int ret;
    uint8_t sht40_buffer[6] = {0};
	float t_ticks, rh_ticks = 0;
    uint8_t test[1] = {0xFD};

    ret = i2c_write(dev_i2c, test, 1, SHT40_ADDR);
    if (ret != 0)
    {
    	printf("Failed to write I2C device address (err %i)\n", ret);
    	return true;
    }

    k_msleep(10);

    ret = i2c_read(dev_i2c, sht40_buffer, 6, SHT40_ADDR);
    if (ret != 0)
    {
    	printf("Failed to read I2C device address (err %i)\n", ret);
    	return true;
    }

    t_ticks = (uint16_t)sht40_buffer[0] * 256 + (uint16_t)sht40_buffer[1];
    rh_ticks = (uint16_t)sht40_buffer[3] * 256 + (uint16_t)sht40_buffer[4];

    data->temperature = -45 + 175 * t_ticks / 65535;
    data->humidity = -6 + 125 * rh_ticks / 65535;

    return false;
}
