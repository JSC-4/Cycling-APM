#ifndef _DFR_NO2_CO_H_
#define _DFR_NO2_CO_H_

#define MICS_ADDRESS_3 0x78

#define CO 0x01	 ///< Carbon Monoxide
#define NO2 0x0A ///< Nitrogen Dioxide
#define POWER_MODE_REGISTER 0x0a
#define SLEEP_MODE 0x00
#define WAKE_UP_MODE 0x01
#define CALIBRATION_TIME 3 // Default calibration time is three minutes
#define ERROR -1
#define OX_REGISTER_HIGH 0x04

#include <device.h>		/* Gives access to device_get_bindings() */


typedef struct dfr_reading
{
    float no2;  ///< Temperature reading
    float co;       ///< Humidity reading
    int16_t r0_ox;
    int16_t r0_red;
    uint16_t oxData;
    uint16_t redData;
    uint16_t powerData;
    float RS_R0_RED_data;
    float RS_R0_OX_data;
} dfr_data;

bool dfrWakeUp(const struct device *dev_i2c);
bool dfrWarmUpTime(const struct device *dev_i2c, dfr_data *dfr_data);
int16_t getSensorData(const struct device *dev_i2c, dfr_data *dfr_data);
float getNitrogenDioxide(const struct device *dev_i2c, dfr_data *dfr_data);
float getCarbonMonoxide(const struct device *dev_i2c, dfr_data *dfr_data);
#endif /* _DFR_NO2_CO_H_ */