/**
 * @file icm20948.h
 * @brief ICM-20948 accelerometer and gyroscope interface.
 * @author 梅雨
 * @date 2026-10-10
 */

#ifndef DREAMFLY2_COMPONENTS_ICM20948_H
#define DREAMFLY2_COMPONENTS_ICM20948_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} icm20948_t;

/** @brief Initialize the ICM-20948 and verify its device ID. */
bool icm20948_init(void);

/** @brief Read the ICM-20948 WHO_AM_I register. */
uint8_t icm20948_read_id(void);

/** @brief Read one set of raw accelerometer and gyroscope data. */
bool icm20948_read(icm20948_t *data);

#endif /* DREAMFLY2_COMPONENTS_ICM20948_H */
