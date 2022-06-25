#ifndef _PARTICULATE_MATTER_H_
#define _PARTICULATE_MATTER_H_

#define PM_ADDR 0x12

#define PM_SC1 0x00
#define DATA5_HIGH 0x0C
#define DATA5_LOW 0x0D

typedef struct pm_reading
{
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
 * @brief Take a reading from the particulate matter sensor.
 *
 * The readings will report back the pm1.0, pm2.5 and pm10 factory values and environmental
 * value. Along with the
 *
 * @param[out] temp - Pointer to the double to be filled with the taken temperature sample.
 * @return int - 0 on success, otherwise, negative error code.
 */
bool pm_read(pm_data *data);

#endif /* _PARTICULATE_MATTER_H_ */