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

/** @brief Initialize the BMP280 and read its factory calibration data. */
bool bmp280_init(void);

/** @brief Read the BMP280 chip ID register. */
uint8_t bmp280_read_id(void);

/** @brief Read one compensated temperature and pressure sample. */
bool bmp280_read(bmp280_t *baro_data);

#endif /* DREAMFLY2_COMPONENTS_BMP280_H */
