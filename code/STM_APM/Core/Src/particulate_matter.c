#include "particulate_matter.h"     // Include header file for pm sensor

HAL_StatusTypeDef pm_initialise(pm_data *dev, I2C_HandleTypeDef *i2cHandle) {

	/* Set up the I2C structure */
	dev->i2cHandle = i2cHandle;

	/* Initialise values to zero */
	dev->pm10_standard = 0;
	dev->pm25_standard = 0;
	dev->pm100_standard = 0;

	dev->pm10_env = 0;
	dev->pm25_env = 0;
	dev->pm100_env = 0;

	dev->particles_03um = 0;
	dev->particles_05um = 0;
	dev->particles_10um = 0;
	dev->particles_25um = 0;
	dev->particles_100um = 0;

	return HAL_OK;
}

HAL_StatusTypeDef pm_read(pm_data *dev) {
	HAL_StatusTypeDef status;
	uint8_t pm_buffer[32] = { 0 };

	/* Write address pointer to start of the registers */
	status = HAL_I2C_Master_Transmit(dev->i2cHandle, PM_ADDR, PM_SC1, 1,
			HAL_MAX_DELAY);
	if (status != HAL_OK) {
		return status;
	} else {
		/* Read particulate matter registers */
		status = HAL_I2C_Master_Receive(dev->i2cHandle, PM_ADDR, pm_buffer, 32,
				HAL_MAX_DELAY);

		/* Check if first element is 0x42 */
		if (status != HAL_OK) {
			return status;
		} else {
			if (pm_buffer[0] != 0x42) {
				return status;
			} else {

				/* Shift data in buffer */
				uint16_t buffer_u16[12];
				for (uint8_t i = 0; i < 12; i++) {
					buffer_u16[i] = pm_buffer[2 + i * 2 + 1];
					buffer_u16[i] += (pm_buffer[2 + i * 2] << 8);
				}

				/* Store readings in structure */
				dev->pm10_standard = buffer_u16[0];
				dev->pm25_standard = buffer_u16[1];
				dev->pm100_standard = buffer_u16[2];
				dev->pm10_env = buffer_u16[3];
				dev->pm25_env = buffer_u16[4];
				dev->pm100_env = buffer_u16[5];
				dev->particles_03um = buffer_u16[6];
				dev->particles_05um = buffer_u16[7];
				dev->particles_10um = buffer_u16[8];
				dev->particles_25um = buffer_u16[9];
				dev->particles_50um = buffer_u16[10];
				dev->particles_100um = buffer_u16[11];
			}
		}
	}
	return HAL_OK;
}
