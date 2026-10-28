/**
 * @file height.c
 * @brief Inertial prediction with third-order barometric error feedback.
 * @author 梅雨
 * @date 2026-10-27
 */

#include "height.h"

#include <math.h>
#include <stddef.h>

/* Continuous-time gains 3/tau, 3/tau^2 and 1/tau^3, with tau = 2 s.
 * Every correction is scaled by its actual interval, independent of IMU rate.
 * Formula locals: dt = elapsed seconds, e = height error, a = corrected acceleration.
 */
#define HEIGHT_POSITION_GAIN 1.5f
#define HEIGHT_VELOCITY_GAIN 0.75f
#define HEIGHT_ACCEL_GAIN 0.125f
#define HEIGHT_ERROR_LIMIT_M 1.0f
#define HEIGHT_ACCEL_CORRECTION_LIMIT_MPS2 2.0f

/** @brief Check estimator state before arithmetic or publication. */
static bool height_valid(const height_t *state)
{
    return state != NULL && isfinite(state->height_m)
        && isfinite(state->velocity_mps) && isfinite(state->acc_correction_mps2);
}

void height_init(height_t *state, float height_m)
{
    if (state != NULL && isfinite(height_m))
    {
        *state = (height_t){.height_m = height_m};
    }
}

bool height_predict(height_t *state, float acc_up_mps2, float sample_time_s)
{
    if (!height_valid(state) || !isfinite(acc_up_mps2)
        || !isfinite(sample_time_s) || sample_time_s <= 0.0f || sample_time_s > 0.02f)
    {
        return false;
    }
    float dt = sample_time_s;
    float a = acc_up_mps2 + state->acc_correction_mps2;
    height_t next = *state;
    next.height_m += state->velocity_mps * dt + 0.5f * a * dt * dt;
    next.velocity_mps += a * dt;
    if (!height_valid(&next))
    {
        return false;
    }
    *state = next;
    return true;
}

bool height_correct(height_t *state, float height_m, float sample_time_s)
{
    if (!height_valid(state) || !isfinite(height_m)
        || !isfinite(sample_time_s) || sample_time_s <= 0.0f || sample_time_s > 0.2f)
    {
        return false;
    }
    float dt = sample_time_s;
    float e = height_m - state->height_m;
    if (!isfinite(e))
    {
        return false;
    }
    if (e > HEIGHT_ERROR_LIMIT_M) { e = HEIGHT_ERROR_LIMIT_M; }
    if (e < -HEIGHT_ERROR_LIMIT_M) { e = -HEIGHT_ERROR_LIMIT_M; }

    height_t next = *state;
    next.height_m += HEIGHT_POSITION_GAIN * e * dt;
    next.velocity_mps += HEIGHT_VELOCITY_GAIN * e * dt;
    next.acc_correction_mps2 += HEIGHT_ACCEL_GAIN * e * dt;
    if (next.acc_correction_mps2 > HEIGHT_ACCEL_CORRECTION_LIMIT_MPS2)
    {
        next.acc_correction_mps2 = HEIGHT_ACCEL_CORRECTION_LIMIT_MPS2;
    }
    if (next.acc_correction_mps2 < -HEIGHT_ACCEL_CORRECTION_LIMIT_MPS2)
    {
        next.acc_correction_mps2 = -HEIGHT_ACCEL_CORRECTION_LIMIT_MPS2;
    }
    if (!height_valid(&next))
    {
        return false;
    }
    *state = next;
    return true;
}
