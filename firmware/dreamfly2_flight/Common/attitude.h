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
    /* Components in one common body frame; units are set by each function parameter. */
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
 * @param acc_g Acceleration in g; NULL or a near-zero vector initializes level.
 * @note Resets both solutions and their correction integrals; initial yaw is zero.
 */
void attitude_init(const attitude_vector_t *acc_g);

/**
 * @brief Calculate attitude from acceleration and angular rate.
 * @param acc_g Acceleration in g.
 * @param gyro_dps Angular rate in degrees per second.
 * @param sample_time_s Solution period in seconds.
 * @param[out] attitude_deg Calculated Euler angles in degrees.
 * @return true when the input and quaternion are valid.
 * @note Maintains private six-axis state; call serially at the supplied period.
 *       Gravity correction is gated by acceleration magnitude. Absolute yaw
 *       cannot be corrected without an external heading reference.
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
 * @return false for NULL pointers, a nonpositive period or near-zero vector/quaternion norms.
 * @note Maintains independent nine-axis state. Use calibrated, axis-aligned magnetic
 *       data; this demonstration solution does not reject magnetic interference.
 */
bool attitude_solve_9axis(const attitude_vector_t *acc_g,
                          const attitude_vector_t *gyro_dps,
                          const attitude_vector_t *mag_ut,
                          float sample_time_s,
                          attitude_t *attitude_deg);

#endif /* DREAMFLY2_COMMON_ATTITUDE_H */
