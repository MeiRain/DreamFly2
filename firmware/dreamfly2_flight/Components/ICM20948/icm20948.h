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
    float acc_x_g;
    float acc_y_g;
    float acc_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} icm20948_t;

/** @brief Initialize the ICM-20948 and verify its device ID. */
bool icm20948_init(void);

/** @brief Read the ICM-20948 WHO_AM_I register. */
uint8_t icm20948_read_id(void);

/** @brief Read acceleration in g and angular rate in degrees/s. */
bool icm20948_read(icm20948_t *imu_data);

/** @brief Write one register on a device connected to the auxiliary I2C bus. */
bool icm20948_aux_write(uint8_t device_address,
                        uint8_t register_address,
                        uint8_t value);

/** @brief Read registers from a device connected to the auxiliary I2C bus. */
bool icm20948_aux_read(uint8_t device_address,
                       uint8_t register_address,
                       uint8_t *data,
                       uint8_t size);

#endif /* DREAMFLY2_COMPONENTS_ICM20948_H */
