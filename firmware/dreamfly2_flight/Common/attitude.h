/**
 * @file attitude.h
 * @brief Six-axis and nine-axis Mahony attitude solutions.
 * @author 梅雨
 * @date 2026-10-11
 */

#ifndef DREAMFLY2_COMMON_ATTITUDE_H
#define DREAMFLY2_COMMON_ATTITUDE_H

#include <stdbool.h>

typedef struct
{
    float x;
    float y;
    float z;
} attitude_vector_t;

typedef struct
{
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
} attitude_t;

/**
 * @brief Initialize both attitude quaternions from acceleration.
 * @param acc_g Acceleration in g. A zero vector initializes a level attitude.
 */
void attitude_init(const attitude_vector_t *acc_g);

/**
 * @brief Calculate attitude from acceleration and angular rate.
 * @param acc_g Acceleration in g.
 * @param gyro_dps Angular rate in degrees per second.
 * @param sample_time_s Solution period in seconds.
 * @param[out] attitude_deg Calculated Euler angles in degrees.
 * @return true when the input and quaternion are valid.
 */
bool attitude_solve_6axis(const attitude_vector_t *acc_g,
                          const attitude_vector_t *gyro_dps,
                          float sample_time_s,
                          attitude_t *attitude_deg);

/**
 * @brief Calculate attitude from acceleration, angular rate and magnetic field.
 * @param acc_g Acceleration in g.
 * @param gyro_dps Angular rate in degrees per second.
 * @param mag_ut Magnetic field in microtesla, aligned with the IMU body axes.
 * @param sample_time_s Solution period in seconds.
 * @param[out] attitude_deg Calculated Euler angles in degrees.
 * @return true when the input and quaternion are valid.
 */
bool attitude_solve_9axis(const attitude_vector_t *acc_g,
                          const attitude_vector_t *gyro_dps,
                          const attitude_vector_t *mag_ut,
                          float sample_time_s,
                          attitude_t *attitude_deg);

#endif /* DREAMFLY2_COMMON_ATTITUDE_H */
