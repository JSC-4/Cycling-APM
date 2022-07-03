/***************************************************************************************
 * File name: sht40.h
 *
 * Author: Jerome Samuels-Clarke
 * Date: Edited June 2022
 *
 * Description: A header for reading the temperature and humidity measurements.
 ***************************************************************************************/

#ifndef _SHT40_H_
#define _SHT40_H_

#include "stm32l4xx_hal.h"

/* Registers for interfacing with the sensor */
#define SHT40_ADDR	    (0x44 << 1)    // SHT40 address
#define SHT40_HP	    0xFD    // High precision mode

/* Structure for holding sensor readings */
typedef struct sht40_reading
{
	I2C_HandleTypeDef *i2cHandle; // I2C Handle

    double temperature;     // Temperature
    double humidity;       //  Humidity
} sht40_data;


/**
 * @brief Initialise sht40 struct
 *
 * @param[in] dev - structure for I2C and data
 * @param[in] i2cHandle - I2C handle
 *
 * @return HAL_StatusTypeDef - HAL_OK on success, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef sht40_initialise(sht40_data *dev, I2C_HandleTypeDef *i2cHandle);


/**
 * @brief Take a reading from the sht40 sensor for temperature and humidity.
 *
 * @param[in] dec_i2c - Pointer to the I2C struct.
 * @param[in] data - Pointer to the struct for holding sensor readings.
 *
 * @return bool - 1 on success, otherwise zero if an error.
 */
HAL_StatusTypeDef sht40_read(sht40_data *dev);

#endif /* _SHT40_H_ */
