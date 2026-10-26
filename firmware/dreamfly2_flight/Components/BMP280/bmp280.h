/**
 * @file bmp280.h
 * @brief BMP280 pressure and temperature sensor interface.
 * @author 梅雨
 * @date 2026-10-11
 */

#ifndef DREAMFLY2_COMPONENTS_BMP280_H
#define DREAMFLY2_COMPONENTS_BMP280_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float temp_c;
    float press_pa;
} bmp280_t;

/**
 * @brief Initialize normal mode, factory compensation and the internal IIR filter.
 * @return true on success; false on ID, startup, coefficient or I2C failure.
 * @note Requires I2C1 initialization; uses blocking I2C and startup delays.
 */
bool bmp280_init(void);

/** @brief Read the chip ID; returns 0 on I2C failure (expected ID: 0x58). */
uint8_t bmp280_read_id(void);

/**
 * @brief Read and compensate one temperature and pressure sample.
 * @param[out] baro_data Temperature in degrees C and pressure in Pa.
 * @return true on success; false for NULL, I2C or compensation failure.
 * @note Call after bmp280_init(); output is unchanged on failure. No height
 *       conversion or additional software filtering is performed.
 */
bool bmp280_read(bmp280_t *baro_data);

#endif /* DREAMFLY2_COMPONENTS_BMP280_H */
