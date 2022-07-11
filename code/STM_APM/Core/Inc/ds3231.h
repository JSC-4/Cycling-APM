/***************************************************************************************
* File name: ds3231.h
*
* Author: Jerome Samuels-Clarke
* SID   : 10473050
* Date: Edited March 2022
*
* Description: A class created for controlling the DS3231 rtc module. The date and time
               can be set and retrived using the public methods.
***************************************************************************************/
#ifndef _DS3231_H_
#define _DS3231_H_

#include "stm32l4xx_hal.h"

/* The DS3231 registers, taken from the datasheet */
#define DS3231_ADDR  0x68

#define DS3231_SECONDS  0x00
#define DS3231_MINUTES  0x01
#define DS3231_HOURS    0x02
#define DS3231_DAY      0x03
#define DS3231_DATE     0x04
#define DS3231_MONTH    0x05
#define DS3231_YEAR     0x06


    /* constant array used for converting the day of the week into a string */
//    const char *dayNames[7] = {"MON", "TUE", "WED", "THR", "FRI", "SAT", "SUN"};

    /* Structure for holding sensor readings */
    typedef struct ds3231_reading
    {
    	I2C_HandleTypeDef *i2cHandle; // I2C Handle

    	uint8_t second;
    	uint8_t minute;
    	uint8_t hour;
    	uint8_t date;
    	uint8_t day;
    	uint8_t month;
    	uint8_t year;

    } ds3231_data;

    HAL_StatusTypeDef ds3231_initialise(ds3231_data *dev, I2C_HandleTypeDef *i2cHandle);

    /* Methods used to set/get the time and date */
    HAL_StatusTypeDef setTime(ds3231_data *dev, int hh, int mm, int ss);
    HAL_StatusTypeDef setDate(ds3231_data *dev, int dow, int dd, int mm, int yy);
    HAL_StatusTypeDef getTime(ds3231_data *dev);
    HAL_StatusTypeDef getDate(ds3231_data *dev);


#endif /* INC_DS3231_H_ */
