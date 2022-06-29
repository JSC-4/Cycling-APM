#include <zephyr/kernel.h>
#include <zephyr.h>     /* Give access to k_msleep() */
#include <device.h>     /* Gives access to device_get_bindings() */
#include <devicetree.h> /* For DT macros */
#include <zephyr/logging/log.h>
#include <drivers/i2c.h>

#include <stdio.h>
#include <zephyr/timing/timing.h>

#include "dfr_no2_co.h"

bool getGasData(const struct device *dev_i2c, dfr_data *dfr_data)
{
    getSensorData(dev_i2c, dfr_data);

    dfr_data->no2 = getNitrogenDioxide(dfr_data);
    dfr_data->co = getCarbonMonoxide(dfr_data);

    return false;
}

float getNitrogenDioxide(dfr_data *dfr_data)
{
    if (dfr_data->RS_R0_OX_data < 1.1)
        return 0;
    float nitrogendioxide = (dfr_data->RS_R0_OX_data - 0.045) / 6.13;
    if (nitrogendioxide < 0.1)
        return 0.0;
    if (nitrogendioxide > 10.0)
        return 10.0;
    return nitrogendioxide;
}

float getCarbonMonoxide(dfr_data *dfr_data)
{
    if (dfr_data->RS_R0_RED_data > 0.425)
        return 0.0;
    float co = (0.425 - dfr_data->RS_R0_RED_data) / 0.000405;
    if (co > 1000.0)
        return 1000.0;
    if (co < 1.0)
        return 0.0;
    return co;
}

bool dfrWakeUp(const struct device *dev_i2c)
{
    int ret = 0;
    uint8_t mode[1] = {0x00};
    uint8_t regData[2] = {POWER_MODE_REGISTER, WAKE_UP_MODE};

    printf("Checking sleep mode\n");

    ret = i2c_write(dev_i2c, regData, 1, MICS_ADDRESS_3);
    if (ret != 0)
    {
        printf("Failed to write I2C device address (err %i)\n", ret);
        return true;
    }

    ret = i2c_read(dev_i2c, mode, 1, MICS_ADDRESS_3);
    if (ret != 0)
    {
        printf("Failed to write I2C device address (err %i)\n", ret);
        return true;
    }

    /* Device is sleeping*/
    if (mode[0] == 0)
    {
        ret = i2c_write(dev_i2c, regData, 2, MICS_ADDRESS_3);
        if (ret != 0)
        {
            printf("Failed to write I2C device address SLEEP (err %i)\n", ret);
            return true;
        }
        printf("Wake sensor up\n");
        k_msleep(100);
    }
    else
    {
        printf("Device is woken up\n");
    }
    return 0;
}

int16_t getSensorData(const struct device *dev_i2c, dfr_data *dfr_data)
{
    int ret = 0;
    int wb[1] = {OX_REGISTER_HIGH};
    uint8_t recv_data[20] = {0x00};

    ret = i2c_write_read(dev_i2c, MICS_ADDRESS_3, wb, 1, recv_data, 6);
    if (ret != 0)
    {
        printf("Failed to write I2C device address for getting sensor data (err %i)\n", ret);
        return ERROR;
    }

    dfr_data->oxData = (((uint16_t)recv_data[0] << 8) + (uint16_t)recv_data[1]);
    dfr_data->redData = (((uint16_t)recv_data[2] << 8) + (uint16_t)recv_data[3]);
    dfr_data->powerData = (((uint16_t)recv_data[4] << 8) + (uint16_t)recv_data[5]);

    dfr_data->r0_ox = dfr_data->powerData - dfr_data->oxData;
    dfr_data->r0_red = dfr_data->powerData - dfr_data->redData;
    // printf("__r0_ox = %d, __r0_red = %d, powerData = %d, oxData = %d, redData = %d\n", dfr_data->r0_ox, dfr_data->r0_red, dfr_data->powerData, dfr_data->oxData, dfr_data->redData);

    dfr_data->RS_R0_RED_data = (float)(dfr_data->powerData - dfr_data->redData) / (float)dfr_data->r0_red;
    dfr_data->RS_R0_OX_data = (float)(dfr_data->powerData - dfr_data->oxData) / (float)dfr_data->r0_ox;

    return 0;
}

bool dfrWarmUpTime(const struct device *dev_i2c, dfr_data *dfr_data)
{
    static timing_t start_time, current_time;
    uint64_t delayTime = 60000000000;
    uint64_t total_cycles;
    uint64_t total_ns;
    static uint8_t flag = 0;

    if (flag == 0)
    {
        flag = 1;
        current_time = 0;
        timing_init();
        timing_start();
        start_time = timing_counter_get();
    }

    current_time = timing_counter_get();

    total_cycles = timing_cycles_get(&start_time, &current_time);
    total_ns = timing_cycles_to_ns(total_cycles);
    // printf("Elapsed Time = %llu\n", total_ns);
    if (total_ns < delayTime)
    {
        return false;
    }

    flag = 0;
    timing_stop();

    if (getSensorData(dev_i2c, dfr_data) == ERROR)
    {
        return false;
    }

    return true;
}