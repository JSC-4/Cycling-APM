#include "dfr_no2_co.h"             // Include header file for dfr sensor

HAL_StatusTypeDef drf_initialise(dfr_data *dev, I2C_HandleTypeDef *i2cHandle) {

	/* Set up the I2C structure */
	dev->i2cHandle = i2cHandle;

	/* Initialise values to zero */
	dev->no2 = 0;
	dev->co = 0;
	dev->r0_ox = 0;
	dev->r0_red = 0;
	dev->oxData = 0;
	dev->redData = 0;
	dev->powerData = 0;
	dev->RS_R0_RED_data = 0;
	dev->RS_R0_OX_data = 0;

	return HAL_OK;
}

/* Take a measurement, and get the readings for NO2 and CO */
HAL_StatusTypeDef getGasData(dfr_data *data)
{
    getSensorData(data);

    data->no2 = getNitrogenDioxide(data);
    data->co = getCarbonMonoxide(data);

    return HAL_OK;
}

/* Convert  measurement for NO2*/
float getNitrogenDioxide(dfr_data *data)
{
    if (data->RS_R0_OX_data < 1.1)
        return 0;
    float nitrogendioxide = (data->RS_R0_OX_data - 0.045) / 6.13;
    if (nitrogendioxide < 0.1)
        return 0.0;
    if (nitrogendioxide > 10.0)
        return 10.0;
    return nitrogendioxide;
}

/* Convert measurement for CO*/
float getCarbonMonoxide(dfr_data *data)
{
    if (data->RS_R0_RED_data > 0.425)
        return 0.0;
    float co = (0.425 - data->RS_R0_RED_data) / 0.000405;
    if (co > 1000.0)
        return 1000.0;
    if (co < 1.0)
        return 0.0;
    return co;
}

/* Check if device is in sleep state */
HAL_StatusTypeDef dfrWakeUp(dfr_data *data)
{
    int ret = 0;
    uint8_t mode[1] = {0x00};
    uint8_t regData[2] = {POWER_MODE_REGISTER, WAKE_UP_MODE};

//    printf("Checking sleep mode\n");

    /* Set the address pointer to power mode register */
    ret = i2c_write(dev_i2c, regData, 1, MICS_ADDRESS_3);
    if (ret != 0)
    {
        printf("Failed to write I2C device address (err %i)\n", ret);
        return HAL_ERROR;
    }

    /* Read current power mode setting */
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

/* Read sensor for measurements */
int16_t getSensorData(dfr_data *data)
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

    /* Shift high and low data in the buffer */
    data->oxData = (((uint16_t)recv_data[0] << 8) + (uint16_t)recv_data[1]);
    data->redData = (((uint16_t)recv_data[2] << 8) + (uint16_t)recv_data[3]);
    data->powerData = (((uint16_t)recv_data[4] << 8) + (uint16_t)recv_data[5]);

    /* Calculate oxsiding and reducing values */
    data->r0_ox = data->powerData - data->oxData;
    data->r0_red = data->powerData - data->redData;
    // printf("__r0_ox = %d, __r0_red = %d, powerData = %d, oxData = %d, redData = %d\n", data->r0_ox, data->r0_red, data->powerData, data->oxData, data->redData);

    data->RS_R0_RED_data = (float)(data->powerData - data->redData) / (float)data->r0_red;
    data->RS_R0_OX_data = (float)(data->powerData - data->oxData) / (float)data->r0_ox;

    return 0;
}

/* Device requires three minutes to warm up before taking samples*/
HAL_StatusTypeDef dfrWarmUpTime(dfr_data *data)
{
    static timing_t start_time, current_time;
    uint64_t delayTime = 60000000000;
    uint64_t total_cycles;
    uint64_t total_ns;
    static uint8_t flag = 0;

    /* Initialie timer */
    if (flag == 0)
    {
        flag = 1;
        current_time = 0;
        timing_init();
        timing_start();
        start_time = timing_counter_get();  // Store the current timer (start)
    }

    current_time = timing_counter_get();  // Store the current timer (current)

    /* Get the elapsed time in nanoseconds */
    total_cycles = timing_cycles_get(&start_time, &current_time);
    total_ns = timing_cycles_to_ns(total_cycles);

    // printf("Elapsed Time = %llu\n", total_ns);
    
    /* If timer has not reached value then return */
    if (total_ns < delayTime)
    {
        return false;
    }

    /* If timer has expired, reset the flag and stop the timer */
    flag = 0;
    timing_stop();

    /* Used to see measurements after every minute */
    if (getSensorData(dev_i2c, data) == ERROR)
    {
        return false;
    }

    return true;
}
