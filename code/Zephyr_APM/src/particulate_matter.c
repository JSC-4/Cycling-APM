#include <zephyr/kernel.h>
#include <zephyr.h>     /* Give access to k_msleep() */
#include <device.h>     /* Gives access to device_get_bindings() */
#include <devicetree.h> /* For DT macros */
#include <zephyr/logging/log.h>
#include <drivers/i2c.h>

#include "particulate_matter.h"

bool pm_read(const struct device *dev_i2c, pm_data *data)
{
    int ret;
    uint8_t pm_buffer[32] = {0};


    ret = i2c_write_read(dev_i2c, PM_ADDR, PM_SC1, 0, &pm_buffer, 32);
    if (ret != 0)
    {
        // printf("Failed to write/read I2C device address (err %i)\n", ret);
        return true;
    }

    // Check that start byte is correct!
    if (pm_buffer[0] != 0x42)
    {
        return true;
    }

    uint16_t buffer_u16[12];
    for (uint8_t i = 0; i < 12; i++)
    {
        buffer_u16[i] = pm_buffer[2 + i * 2 + 1];
        buffer_u16[i] += (pm_buffer[2 + i * 2] << 8);
    }

    data->pm10_standard = buffer_u16[0];
    data->pm25_standard = buffer_u16[1];
    data->pm100_standard = buffer_u16[2];
    data->pm10_env = buffer_u16[3];
    data->pm25_env = buffer_u16[4];
    data->pm100_env = buffer_u16[5];
    data->particles_03um = buffer_u16[6]; 
    data->particles_05um =  buffer_u16[7];
    data->particles_10um =  buffer_u16[8];
    data->particles_25um =  buffer_u16[9];
    data->particles_50um =  buffer_u16[10];
    data->particles_100um =  buffer_u16[11];

    return false;
}
