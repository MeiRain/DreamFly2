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
 * @note The ICM20948 must be initialized first.
 */
bool ak09916_init(void);

/** @brief Read the AK09916 WIA2 device ID. */
uint8_t ak09916_read_id(void);

/** @brief Read one magnetic-field sample in microtesla. */
bool ak09916_read(ak09916_t *mag_data);

#endif /* DREAMFLY2_COMPONENTS_AK09916_H */
