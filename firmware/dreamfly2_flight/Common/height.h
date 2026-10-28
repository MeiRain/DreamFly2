/**
 * @file height.h
 * @brief Third-order complementary height and vertical-velocity estimator.
 * @author 梅雨
 * @date 2026-10-27
 */

#ifndef DREAMFLY2_COMMON_HEIGHT_H
#define DREAMFLY2_COMMON_HEIGHT_H

#include <stdbool.h>

typedef struct
{
    float height_m;
    float velocity_mps; /* Height and velocity are positive upward. */
    float acc_correction_mps2; /* Slowly learned additive acceleration correction. */
} height_t;

/**
 * @brief Seed height from a ground-referenced measurement; reset velocity and correction.
 * @note NULL or a nonfinite height leaves the state unchanged.
 */
void height_init(height_t *state, float height_m);

/**
 * @brief Predict height and velocity from gravity-removed upward acceleration.
 * @param state Caller-owned estimator; initialize before use.
 * @param acc_up_mps2 Upward acceleration in m/s^2, excluding gravity.
 * @param sample_time_s Actual interval since the previous prediction, 0 < dt <= 0.02 s.
 * @return false for invalid input/state; state is unchanged on failure.
 * @note No sensor access or RTOS dependency. Serialize calls for each instance.
 */
bool height_predict(height_t *state, float acc_up_mps2, float sample_time_s);

/**
 * @brief Correct height, velocity and acceleration using one new barometric height.
 * @param state Caller-owned estimator, shared with height_predict().
 * @param height_m Measured relative height in meters, positive upward.
 * @param sample_time_s Actual interval between corrections, 0 < dt <= 0.2 s.
 * @return false for invalid input/state; state is unchanged on failure.
 * @note Call once per new measurement, not repeatedly for a cached sample.
 *       Gains are initial bench values. Slow barometric drift remains observable
 *       as height drift; this filter cannot replace an absolute ground reference.
 */
bool height_correct(height_t *state, float height_m, float sample_time_s);

#endif /* DREAMFLY2_COMMON_HEIGHT_H */
