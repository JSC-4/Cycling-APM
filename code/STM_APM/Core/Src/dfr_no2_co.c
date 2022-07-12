#include "dfr_no2_co.h"             // Include header file for dfr sensor

HAL_StatusTypeDef dfr_initialise(dfr_data *dev, I2C_HandleTypeDef *i2cHandle) {

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
HAL_StatusTypeDef getGasData(dfr_data *dev)
{
	HAL_StatusTypeDef status;

	status = getSensorData(dev);
    if (status == HAL_ERROR)
    {
        return HAL_ERROR;
    }
    dev->no2 = getNitrogenDioxide(dev);
    dev->co = getCarbonMonoxide(dev);
    dev->ch4 = getMethane(dev);
    dev->c2h5oh = getEthanol(dev);
    dev->h2 = getHydrogen(dev);
    dev->nh3 = getAmmonia(dev);

    return HAL_OK;
}

/* Convert  measurement for NO2*/
float getNitrogenDioxide(dfr_data *dev)
{
    if (dev->RS_R0_OX_data < 1.1)
        return 0;
    float nitrogendioxide = (dev->RS_R0_OX_data - 0.045) / 6.13;
    if (nitrogendioxide < 0.1)
        return 0.0;
    if (nitrogendioxide > 10.0)
        return 10.0;
    return nitrogendioxide;
}

/* Convert measurement for CO*/
float getCarbonMonoxide(dfr_data *dev)
{
    if (dev->RS_R0_RED_data > 0.425)
        return 0.0;
    float co = (0.425 - dev->RS_R0_RED_data) / 0.000405;
    if (co > 1000.0)
        return 1000.0;
    if (co < 1.0)
        return 0.0;
    return co;
}

float getMethane(dfr_data *dev)
{
  if(dev->RS_R0_RED_data > 0.786)
    return 0.0;
  float methane = (0.786 - dev->RS_R0_RED_data) / 0.000023;
  if(methane < 1000.0) methane = 0.0;
  if(methane > 25000.0) methane = 25000.0;
  return methane;
}

float getEthanol(dfr_data *dev)
{
  if(dev->RS_R0_RED_data > 0.306)
    return 0.0;
  float ethanol = (0.306 - dev->RS_R0_RED_data) / 0.00057;
  if(ethanol < 10.0)
    return 0.0;
  if(ethanol > 500.0)
    return 500.0;
  return ethanol;
}

float getHydrogen(dfr_data *dev)
{
  if(dev->RS_R0_RED_data > 0.279)
    return 0.0;
  float hydrogen = (0.279 - dev->RS_R0_RED_data) / 0.00026;
  if(hydrogen < 1.0)
    return 0.0;
  if(hydrogen > 1000.0)
    return 1000.0;
  return hydrogen;
}

float getAmmonia(dfr_data *dev)
{
  if(dev->RS_R0_RED_data > 0.8)
    return 0.0;
  float ammonia = (0.8 - dev->RS_R0_RED_data) / 0.0015;
  if(ammonia < 1.0)
    return 0.0;
  if(ammonia > 500.0)
    return 500.0;
  return ammonia;
}

/* Check if device is in sleep state */
HAL_StatusTypeDef dfrWakeUp(dfr_data *dev)
{
	HAL_StatusTypeDef status;
    uint8_t mode[1] = {0x00};
    uint8_t regData[2] = {POWER_MODE_REGISTER, WAKE_UP_MODE};

    /* Set the address pointer to power mode register */
    status = HAL_I2C_Master_Transmit(dev->i2cHandle, (0x78 << 1), regData, 1,
			50);
    if (status != 0)
    {
        return HAL_ERROR;
    }

    /* Read current power mode setting */
    status = HAL_I2C_Master_Receive(dev->i2cHandle, (0x78 << 1) | 0x01, mode, 1,
			50);
    if (status != 0)
    {
//        printf("Failed to write I2C device address for getting sensor data (err %i)\n", ret);
        return HAL_ERROR;
    }

    /* Device is sleeping*/
    if (mode[0] == 0)
    {

        status = HAL_I2C_Master_Transmit(dev->i2cHandle, (0x78 << 1), regData, 2,
    			50);
        if (status != 0)
        {
            return HAL_ERROR;
        }
//        printf("Wake sensor up\n");
//        k_msleep(100);
    }
    else
    {
//        printf("Device is woken up\n");
    }
    return HAL_OK;
}

HAL_StatusTypeDef dfrWarmUpData(dfr_data *dev)
{
	HAL_StatusTypeDef status;

	status = getSensorData(dev);
    if (status == HAL_ERROR)
    {
        return HAL_ERROR;
    }

    /* Calculate oxsiding and reducing values */
    dev->r0_ox = dev->powerData - dev->oxData;
    dev->r0_red = dev->powerData - dev->redData;

    return HAL_OK;
}

/* Read sensor for measurements */
int16_t getSensorData(dfr_data *dev)
{
	HAL_StatusTypeDef status;
//    uint8_t buf[1] = {OX_REGISTER_HIGH};
    uint8_t recv_data[20] = {0x00};

    status = HAL_I2C_Mem_Read(dev->i2cHandle, (0x78 << 1), 0x04, I2C_MEMADD_SIZE_8BIT, recv_data, 6, 100);
    /* Set the address pointer to OX_REGISTER_HIGH */
//    status = HAL_I2C_Master_Transmit(dev->i2cHandle, (0x78 << 1), buf, 1,
//			50);
    if (status != 0)
    {
//        printf("Failed to write I2C device address for getting sensor data (err %i)\n", ret);
        return HAL_ERROR;
    }
//    status = HAL_I2C_Master_Receive(dev->i2cHandle, (0x78 << 1) | 0x01, recv_data, 6,
//			50);
//    if (status != 0)
//    {
////        printf("Failed to write I2C device address for getting sensor data (err %i)\n", ret);
//        return HAL_ERROR;
//    }

    /* Shift high and low data in the buffer */
    dev->oxData = (((uint16_t)recv_data[0] << 8) + (uint16_t)recv_data[1]);
    dev->redData = (((uint16_t)recv_data[2] << 8) + (uint16_t)recv_data[3]);
    dev->powerData = (((uint16_t)recv_data[4] << 8) + (uint16_t)recv_data[5]);

    // printf("__r0_ox = %d, __r0_red = %d, powerData = %d, oxData = %d, redData = %d\n", data->r0_ox, data->r0_red, data->powerData, data->oxData, data->redData);

    dev->RS_R0_RED_data = (float)(dev->powerData - dev->redData) / (float)dev->r0_red;
    dev->RS_R0_OX_data = (float)(dev->powerData - dev->oxData) / (float)dev->r0_ox;

    return 0;
}

/* Device requires three minutes to warm up before taking samples*/
//HAL_StatusTypeDef dfrWarmUpTime(dfr_data *data)
//{
//    static timing_t start_time, current_time;
//    uint64_t delayTime = 60000000000;
//    uint64_t total_cycles;
//    uint64_t total_ns;
//    static uint8_t flag = 0;
//
//    /* Initialie timer */
//    if (flag == 0)
//    {
//        flag = 1;
//        current_time = 0;
//        timing_init();
//        timing_start();
//        start_time = timing_counter_get();  // Store the current timer (start)
//    }
//
//    current_time = timing_counter_get();  // Store the current timer (current)
//
//    /* Get the elapsed time in nanoseconds */
//    total_cycles = timing_cycles_get(&start_time, &current_time);
//    total_ns = timing_cycles_to_ns(total_cycles);
//
//    // printf("Elapsed Time = %llu\n", total_ns);
//
//    /* If timer has not reached value then return */
//    if (total_ns < delayTime)
//    {
//        return false;
//    }
//
//    /* If timer has expired, reset the flag and stop the timer */
//    flag = 0;
//    timing_stop();
//
//    /* Used to see measurements after every minute */
//    if (getSensorData(dev_i2c, data) == ERROR)
//    {
//        return false;
//    }
//
//    return true;
//}
