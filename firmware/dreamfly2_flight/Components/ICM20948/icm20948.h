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

/**
 * @brief Reset and configure the ICM-20948 for +/-4 g and +/-2000 degrees/s.
 * @return true on success, false on an ID mismatch or I2C failure.
 * @note I2C1 must be initialized first. Uses blocking reset/startup delays.
 *       This driver and its auxiliary bus access must be serialized by the caller.
 */
bool icm20948_init(void);

/** @brief Read WHO_AM_I; returns 0 on communication failure (expected ID: 0xEA). */
uint8_t icm20948_read_id(void);

/**
 * @brief Read acceleration in g and angular rate in degrees/s.
 * @param[out] imu_data Sample in sensor axes, without software bias or filtering.
 * @return true on success; false for NULL or I2C failure, leaving output unchanged.
 * @note Call after icm20948_init(); this is a blocking I2C read.
 */
bool icm20948_read(icm20948_t *imu_data);

/**
 * @brief Write one register through the ICM auxiliary I2C master.
 * @param device_address Unshifted 7-bit auxiliary device address.
 * @param register_address Target device register.
 * @param value Byte to write.
 * @return true if host-side I2C operations succeed; auxiliary ACK is not checked.
 * @note Requires icm20948_init(); includes a blocking 1 ms transfer delay.
 */
bool icm20948_aux_write(uint8_t device_address,
                        uint8_t register_address,
                        uint8_t value);

/**
 * @brief Read consecutive registers through the ICM auxiliary I2C master.
 * @param device_address Unshifted 7-bit auxiliary device address.
 * @param register_address First target device register.
 * @param[out] data Buffer with at least size writable bytes; discard on failure.
 * @param size Number of bytes, 1..15.
 * @return true if arguments and host-side I2C operations succeed.
 * @note Requires icm20948_init(); includes a blocking 1 ms transfer delay.
 *       Auxiliary ACK is not checked, so device-specific validation is still needed.
 */
bool icm20948_aux_read(uint8_t device_address,
                       uint8_t register_address,
                       uint8_t *data,
                       uint8_t size);

#endif /* DREAMFLY2_COMPONENTS_ICM20948_H */
