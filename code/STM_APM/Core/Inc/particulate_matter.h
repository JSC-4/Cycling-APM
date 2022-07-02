/***************************************************************************************
 * File name: particulate_matter.h
 *
 * Author: Jerome Samuels-Clarke
 * Date: Edited June 2022
 *
 * Description: A header for reading the DFR MEMS Gas Sensor's NO2 and CO measurements.
 ***************************************************************************************/

#ifndef _PARTICULATE_MATTER_H_
#define _PARTICULATE_MATTER_H_

#include "stm32l4xx_hal.h"

/* Registers for interfacing with the sensor */
#define PM_ADDR         (0x12 << 1)    // Particulate matter address
#define PM_SC1          0x00    // Start address

/* Registers for interfacing with the sensor */
typedef struct pm_reading
{
	I2C_HandleTypeDef *i2cHandle; // I2C Handle

    uint16_t pm10_standard,  ///< Standard PM1.0
        pm25_standard,       ///< Standard PM2.5
        pm100_standard;      ///< Standard PM10.0
    uint16_t pm10_env,       ///< Environmental PM1.0
        pm25_env,            ///< Environmental PM2.5
        pm100_env;           ///< Environmental PM10.0
    uint16_t particles_03um, ///< 0.3um Particle Count
        particles_05um,      ///< 0.5um Particle Count
        particles_10um,      ///< 1.0um Particle Count
        particles_25um,      ///< 2.5um Particle Count
        particles_50um,      ///< 5.0um Particle Count
        particles_100um;     ///< 10.0um Particle Count
} pm_data;

/**
 * @brief Initialise pm struct
 *
 * @param[in] dev - structure for I2C and data
 * @param[in] i2cHandle - I2C handle
 *
 * @return HAL_StatusTypeDef - HAL_OK on success, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef pm_initialise(pm_data *dev, I2C_HandleTypeDef *i2cHandle);

/**
 * @brief Read the pm sensor, from various measurements.
 *
 * @param[in] dev - Pointer to the struct for holding sensor readings and I2C handle.
 *
 * @return HAL_StatusTypeDef - HAL_OK on success, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef pm_read(pm_data *dev);

#endif /* _PARTICULATE_MATTER_H_ */
