/***************************************************************************************
 * File name: ds3231.cpp
 *
 * Author: Jerome Samuels-Clarke
 * SID   : 10473050
 * Date: Edited March 2022
 *
 * Description: CPP file for the DS3231
 ***************************************************************************************/
/* Include the header file for the DS3231 */
#include "ds3231.h"

/* Reading from the DS3231 is in binary coded decimal (bcd) format. This function
 will convert the bcd into decimal (int).  */
int bcd2dec(uint8_t b)
{
	return b = ((b / 16) * 10 + (b % 16));
}

uint8_t DS3231_DecodeBCD(uint8_t bin) {
	return (((bin & 0xf0) >> 4) * 10) + (bin & 0x0f);
}
/* Writing to the DS3231 needs to be in binary coded decimal (bcd) format. This function
 will convert the decimal (char) values into bcd form.  */
int dec2bcd(char d) {
	return (((d / 10) << 4) | (d % 10));
}

HAL_StatusTypeDef ds3231_initialise(ds3231_data *dev,
		I2C_HandleTypeDef *i2cHandle) {

	/* Set up the I2C structure */
	dev->i2cHandle = i2cHandle;

	return HAL_OK;
}

/* Method to set the time for the DS3231 */
HAL_StatusTypeDef setTime(ds3231_data *dev, int hh, int mm, int ss) {

	HAL_StatusTypeDef status;

	uint8_t buf[2];

	buf[0] = DS3231_SECONDS;
	buf[1] = dec2bcd(ss);
	status = HAL_I2C_Master_Transmit(dev->i2cHandle, (0x68 << 1), buf, 2, 50);
	if (status != 0) {
//    	printf("Failed to write I2C device address (err %i)\n", ret);
		return HAL_TIMEOUT;
	}

	buf[0] = DS3231_MINUTES;
	buf[1] = dec2bcd(mm);
	status = HAL_I2C_Master_Transmit(dev->i2cHandle, (0x68 << 1), buf, 2, 50);
	if (status != 0) {
//    	printf("Failed to write I2C device address (err %i)\n", ret);
		return HAL_TIMEOUT;
	}

	buf[0] = DS3231_HOURS;
	buf[1] = dec2bcd(hh);
	status = HAL_I2C_Master_Transmit(dev->i2cHandle, (0x68 << 1), buf, 2, 50);
	if (status != 0) {
//    	printf("Failed to write I2C device address (err %i)\n", ret);
		return HAL_TIMEOUT;
	}
	return HAL_OK;
}

/* Method to set the date for the DS3231 */
HAL_StatusTypeDef setDate(ds3231_data *dev, int dow, int dd, int mm, int yy) {

	HAL_StatusTypeDef status;

	uint8_t buf[2];

	buf[0] = DS3231_DAY;
	buf[1] = dec2bcd(dow);
	status = HAL_I2C_Master_Transmit(dev->i2cHandle, (0x68 << 1), buf, 2, 50);
	if (status != 0) {
//    	printf("Failed to write I2C device address (err %i)\n", ret);
		return HAL_TIMEOUT;
	}

	buf[0] = DS3231_DATE;
	buf[1] = dec2bcd(dd);
	status = HAL_I2C_Master_Transmit(dev->i2cHandle, (0x68 << 1), buf, 2, 50);
	if (status != 0) {
//    	printf("Failed to write I2C device address (err %i)\n", ret);
		return HAL_TIMEOUT;
	}

	buf[0] = DS3231_MONTH;
	buf[1] = dec2bcd(mm);
	status = HAL_I2C_Master_Transmit(dev->i2cHandle, (0x68 << 1), buf, 2, 50);
	if (status != 0) {
//    	printf("Failed to write I2C device address (err %i)\n", ret);
		return HAL_TIMEOUT;
	}

	buf[0] = DS3231_YEAR;
	buf[1] = dec2bcd(yy);
	status = HAL_I2C_Master_Transmit(dev->i2cHandle, (0x68 << 1), buf, 2, 50);
	if (status != 0) {
//    	printf("Failed to write I2C device address (err %i)\n", ret);
		return HAL_TIMEOUT;
	}

	return HAL_OK;
}

/* Method to get the time from the DS3231. The three registers are one after the
 other, so an repeat I2C read can be done. */
HAL_StatusTypeDef getTime(ds3231_data *dev) {

	HAL_StatusTypeDef status;

	uint8_t cmd[1];
	uint8_t buf[3];

	cmd[0] = DS3231_SECONDS;

	/* Write to the DS3231 memory register to read seconds, minutes and hours */
    status = HAL_I2C_Mem_Read(dev->i2cHandle, (DS3231_ADDR << 1), cmd, I2C_MEMADD_SIZE_8BIT, buf, 3, 50);
    if (status != 0)
    {
    	return HAL_ERROR;
    }

    /* Convert the BCD values to integers, and store in the struct */
	dev->second = DS3231_DecodeBCD(buf[0]);
	dev->minute = DS3231_DecodeBCD(buf[1]);
	dev->hour = DS3231_DecodeBCD(buf[2]);

	return HAL_OK;
}

/* Method to get the date from the DS3231. The four registers are one after the
 other, so an repeat I2C read can be done. */
HAL_StatusTypeDef getDate(ds3231_data *dev) {

	HAL_StatusTypeDef status;

	uint8_t cmd[1];
	uint8_t buf[4];

	cmd[0] = DS3231_DAY;

	status = HAL_I2C_Master_Transmit(dev->i2cHandle, (0x68 << 1), cmd, 1, 50);
	if (status != 0) {
//    	printf("Failed to write I2C device address (err %i)\n", ret);
		return HAL_TIMEOUT;
	}

    status = HAL_I2C_Master_Receive(dev->i2cHandle, (0x68 << 1) | 0x01, buf, 4,
			50);
    if (status != 0)
    {
//    	printf("Failed to read I2C device address (err %i)\n", ret);
    	return HAL_ERROR;
    }

	for (int i = 0; i < 4; i++) {
		bcd2dec(buf[i]);
	}

	dev->date = buf[0];
	dev->day = buf[1];
	dev->month = buf[2];
	dev->year = buf[3];

	return HAL_OK;
}
