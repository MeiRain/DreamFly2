/**
 * @file ak09916.h
 * @brief AK09916 three-axis magnetometer interface.
 * @author 梅雨
 * @date 2026-10-11
 */

#ifndef DREAMFLY2_COMPONENTS_AK09916_H
#define DREAMFLY2_COMPONENTS_AK09916_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float mag_x_ut;
    float mag_y_ut;
    float mag_z_ut;
} ak09916_t;

/**
 * @brief Initialize the AK09916 in continuous 100 Hz measurement mode.
 * @return true on success, false on device ID or auxiliary transfer failure.
 * @note The ICM20948 must be initialized first; uses blocking startup delays.
 */
bool ak09916_init(void);

/** @brief Read WIA2; returns 0 on transfer failure (expected ID: 0x09). */
uint8_t ak09916_read_id(void);

/**
 * @brief Read one magnetic-field sample in microtesla, in sensor axes.
 * @param[out] mag_data Uncalibrated magnetic field; unchanged on failure.
 * @return true on success; false for NULL, transfer failure, no new data or overflow.
 * @note Call after ak09916_init(); auxiliary transfer includes a blocking delay.
 */
bool ak09916_read(ak09916_t *mag_data);

#endif /* DREAMFLY2_COMPONENTS_AK09916_H */
