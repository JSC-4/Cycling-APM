/***************************************************************************************
 * File name: dfr_no2_co.h
 *
 * Author: Jerome Samuels-Clarke
 * Date: Edited June 2022
 *
 * Description: A header for reading the DFR MEMS Gas Sensor's NO2 and CO measurements.
 ***************************************************************************************/

#ifndef _DFR_NO2_CO_H_
#define _DFR_NO2_CO_H_

#include "stm32l4xx_hal.h"

/* Registers for interfacing with the sensor */
#define MICS_ADDRESS_3 (0x78 << 1) // Device address

#define CO 0x01                  // Carbon Monoxide
#define NO2 0x0A                 // Nitrogen Dioxide
#define POWER_MODE_REGISTER 0x0a // Power mode
#define SLEEP_MODE 0x00          // Sleep mode
#define WAKE_UP_MODE 0x01        // Wake up mode
#define OX_REGISTER_HIGH 0x04    // Oxidising gas
#define ERROR -1

/* Structure for holding sensor readings */
typedef struct dfr_reading
{
	I2C_HandleTypeDef *i2cHandle; // I2C Handle

    float no2;
    float co;
    float ch4;
    float c2h5oh;
    float h2;
    float nh3;

    int16_t r0_ox;
    int16_t r0_red;
    uint16_t oxData;
    uint16_t redData;
    uint16_t powerData;
    float RS_R0_RED_data;
    float RS_R0_OX_data;
} dfr_data;


/**
 * @brief Initialise pm struct
 *
 * @param[in] dev - structure for I2C and data
 * @param[in] i2cHandle - I2C handle
 *
 * @return HAL_StatusTypeDef - HAL_OK on success, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef dfr_initialise(dfr_data *dev, I2C_HandleTypeDef *i2cHandle);

/**
 * @brief Initially the device is in sleep mode, this function will wake the device.
 *
 * @param[in] dec_i2c - Pointer to the I2C struct.
 * @return bool - 0 on success, otherwise true if an error.
 */
HAL_StatusTypeDef dfrWakeUp(dfr_data *dev);


/**
 * @brief Device requires three minutes to warm up in clean air before taking a reading.
 *
 * @param[in] dec_i2c - Pointer to the I2C struct.
 * @param[in] data - Pointer to the struct for holding sensor readings.
 *
 * @return bool - 1 on success, otherwise zero if an error.
 */
//HAL_StatusTypeDef dfrWarmUpTime(dfr_data *dev);


/**
 * @brief Read the reducing and oxidising gas values, used for calculating other gases.
 *
 * @param[in] dec_i2c - Pointer to the I2C struct.
 * @param[in] data - Pointer to the struct for holding sensor readings.
 *
 * @return bool - 1 on success, otherwise zero if an error.
 */
int16_t getSensorData(dfr_data *dev);


/**
 * @brief Call this function to read NO2 and CO values
 *
 * @param[in] dec_i2c - Pointer to the I2C struct.
 * @param[in] data - Pointer to the struct for holding sensor readings.
 *
 * @return bool - 1 on success, otherwise zero if an error.
 */
HAL_StatusTypeDef getGasData(dfr_data *dev);

/**
 * @brief Call this function to read get the r0_ox and r0_red values after warmup
 *
 * @param[in] dec_i2c - Pointer to the I2C struct.
 * @param[in] data - Pointer to the struct for holding sensor readings.
 *
 * @return bool - 1 on success, otherwise zero if an error.
 */
HAL_StatusTypeDef dfrWarmUpData(dfr_data *dev);

/**
 * @brief Calculate Nitrogen Dioxide value.
 *
 * @param[in] data - Pointer to the struct for holding sensor reading.
 *
 * @return float - to no2 variable in struct.
 */
float getNitrogenDioxide(dfr_data *dev);


/**
 * @brief Calculate Carbon Monoxide value.
 *
 * @param[in] data - Pointer to the struct for holding sensor reading.
 *
 * @return float - to co variable in struct.
 */
float getCarbonMonoxide(dfr_data *dev);

/**
 * @brief Calculate Methane value.
 *
 * @param[in] data - Pointer to the struct for holding sensor reading.
 *
 * @return float - to ch4 variable in struct.
 */
float getMethane(dfr_data *dev);

/**
 * @brief Calculate Ethanol value.
 *
 * @param[in] data - Pointer to the struct for holding sensor reading.
 *
 * @return float - to c2h5oh variable in struct.
 */
float getEthanol(dfr_data *dev);

/**
 * @brief Calculate hydrogen value.
 *
 * @param[in] data - Pointer to the struct for holding sensor reading.
 *
 * @return float - to h2 variable in struct.
 */
float getHydrogen(dfr_data *dev);

/**
 * @brief Calculate Ammonia value.
 *
 * @param[in] data - Pointer to the struct for holding sensor reading.
 *
 * @return float - to nh3 variable in struct.
 */
float getAmmonia(dfr_data *dev);

#endif /* _DFR_NO2_CO_H_ */
