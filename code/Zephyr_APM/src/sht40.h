#ifndef _SHT40_H_
#define _SHT40_H_

#define SHT40_ADDR	0x44
#define SHT40_HP	0xFD

typedef struct sht40_reading
{
    uint16_t temperature;  ///< Temperature reading
    uint16_t humidity;       ///< Humidity reading
} sht40_data;

/**
 * @brief Take a reading from the particulate matter sensor.
 *
 * The readings will report back the pm1.0, pm2.5 and pm10 factory values and environmental
 * value. Along with the
 *
 * @param[out] temp - Pointer to the double to be filled with the taken temperature sample.
 * @return int - 0 on success, otherwise, negative error code.
 */
bool sht40_read(sht40_data *data);

#endif /* _SHT40_H_ */