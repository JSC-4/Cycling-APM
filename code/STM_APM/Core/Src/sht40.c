#include "sht40.h"      // Include header file for sht40

HAL_StatusTypeDef sht40_initialise(sht40_data *dev, I2C_HandleTypeDef *i2cHandle) {

	/* Set up the I2C structure */
	dev->i2cHandle = i2cHandle;

	/* Initialise values to zero */
	dev->temperature = 0;
	dev->humidity = 0;

	return HAL_OK;
}

/* Function to read the sht40 sensor */
HAL_StatusTypeDef sht40_read(sht40_data *dev)
{
	HAL_StatusTypeDef status;

    uint8_t sht40_buffer[6] = {0};
	float t_ticks, rh_ticks = 0;
    uint8_t buf[1] = {SHT40_HP};

    /* Write to the high precision register for temperature and humidity values */
        status = HAL_I2C_Master_Transmit(dev->i2cHandle, (0x44 << 1), buf, 1,
    			50);
    if (status != 0)
    {
    	return HAL_TIMEOUT;
    }

    /* wait for 10 milliseconds */
    HAL_Delay(10);

    /* Read the six return values for the register */
    status = HAL_I2C_Master_Receive(dev->i2cHandle, (0x44 << 1) | 0x01, sht40_buffer, 6,
			50);
    if (status != 0)
    {
    	return HAL_ERROR;
    }

    /* Shift the data and convert to temperature and humidity reading */
    t_ticks = (uint16_t)sht40_buffer[0] * 256 + (uint16_t)sht40_buffer[1];
    rh_ticks = (uint16_t)sht40_buffer[3] * 256 + (uint16_t)sht40_buffer[4];

    dev->temperature = -45 + 175 * t_ticks / 65535;
    dev->humidity = -6 + 125 * rh_ticks / 65535;

    return HAL_OK;
}
