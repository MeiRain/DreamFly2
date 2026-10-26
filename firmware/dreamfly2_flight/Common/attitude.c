/**
 * @file attitude.c
 * @brief Six-axis and nine-axis Mahony attitude solutions.
 * @author 梅雨
 * @date 2026-10-11
 */

/*
 * Formula notation:
 * q0-q3: quaternion; a: normalized acceleration; g: angular rate (rad/s);
 * m: normalized magnetic field; v: estimated gravity direction;
 * w: estimated magnetic direction; e: direction error; dt: sample time (s).
 * h: magnetic field rotated into the reference frame; bx/bz: its horizontal
 * and vertical components; ex_int/ey_int/ez_int: integrated direction errors.
 */

#include "attitude.h"

#include <math.h>
#include <stddef.h>

#define ATTITUDE_KP 0.2f
#define ATTITUDE_KI 0.1f
#define ATTITUDE_DEG_TO_RAD 0.01745329252f
#define ATTITUDE_RAD_TO_DEG 57.29577951f
#define ATTITUDE_MIN_NORM 0.001f
/* Initial six-axis tuning: reject obvious linear acceleration; learn bias slowly. */
#define ATTITUDE_ACCEL_MIN_G 0.85f
#define ATTITUDE_ACCEL_MAX_G 1.15f
#define ATTITUDE_BIAS_MAX_RATE_DPS 20.0f

typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
    float ex_int;
    float ey_int;
    float ez_int;
} attitude_state_t;

/* Independent states allow the flight and demonstration solutions to run together. */
static attitude_state_t s_6axis_state = {.q0 = 1.0f};
static attitude_state_t s_9axis_state = {.q0 = 1.0f};

/**
 * @brief Integrate corrected angular rates in rad/s over dt seconds, normalize
 *        the quaternion and extract Euler angles in degrees.
 * @return false if the resulting quaternion norm is too small.
 */
static bool attitude_update(attitude_state_t *state,
                            float gx,
                            float gy,
                            float gz,
                            float dt,
                            attitude_t *angle)
{
    float q0 = state->q0;
    float q1 = state->q1;
    float q2 = state->q2;
    float q3 = state->q3;
    float half_dt = 0.5f * dt;
    float norm;
    float sin_pitch;

    state->q0 += (-q1 * gx - q2 * gy - q3 * gz) * half_dt;
    state->q1 += (q0 * gx - q3 * gy + q2 * gz) * half_dt;
    state->q2 += (q3 * gx + q0 * gy - q1 * gz) * half_dt;
    state->q3 += (-q2 * gx + q1 * gy + q0 * gz) * half_dt;

    norm = sqrtf(state->q0 * state->q0
                 + state->q1 * state->q1
                 + state->q2 * state->q2
                 + state->q3 * state->q3);

    if (norm < ATTITUDE_MIN_NORM)
    {
        return false;
    }

    state->q0 /= norm;
    state->q1 /= norm;
    state->q2 /= norm;
    state->q3 /= norm;

    sin_pitch = 2.0f * (state->q0 * state->q2
                        - state->q1 * state->q3);

    if (sin_pitch > 1.0f)
    {
        sin_pitch = 1.0f;
    }
    else if (sin_pitch < -1.0f)
    {
        sin_pitch = -1.0f;
    }

    angle->roll_deg = atan2f(2.0f * (state->q2 * state->q3
                                    + state->q0 * state->q1),
                             1.0f - 2.0f * (state->q1 * state->q1
                                            + state->q2 * state->q2))
                      * ATTITUDE_RAD_TO_DEG;
    angle->pitch_deg = asinf(sin_pitch) * ATTITUDE_RAD_TO_DEG;
    angle->yaw_deg = atan2f(2.0f * (state->q1 * state->q2
                                   + state->q0 * state->q3),
                            1.0f - 2.0f * (state->q2 * state->q2
                                           + state->q3 * state->q3))
                     * ATTITUDE_RAD_TO_DEG;

    return true;
}

void attitude_init(const attitude_vector_t *acc_g)
{
    attitude_state_t init = {.q0 = 1.0f};
    float norm_sq;
    float roll;
    float pitch;
    float roll_half;
    float pitch_half;
    float norm;

    if (acc_g != NULL)
    {
        norm_sq = acc_g->x * acc_g->x
                  + acc_g->y * acc_g->y
                  + acc_g->z * acc_g->z;

        if (norm_sq >= ATTITUDE_MIN_NORM * ATTITUDE_MIN_NORM)
        {
            roll = atan2f(acc_g->y, acc_g->z);
            pitch = atan2f(-acc_g->x,
                           sqrtf(acc_g->y * acc_g->y
                                 + acc_g->z * acc_g->z));
            roll_half = 0.5f * roll;
            pitch_half = 0.5f * pitch;

            init.q0 = cosf(pitch_half) * cosf(roll_half);
            init.q1 = cosf(pitch_half) * sinf(roll_half);
            init.q2 = sinf(pitch_half) * cosf(roll_half);
            init.q3 = -sinf(pitch_half) * sinf(roll_half);

            norm = sqrtf(init.q0 * init.q0
                         + init.q1 * init.q1
                         + init.q2 * init.q2
                         + init.q3 * init.q3);
            init.q0 /= norm;
            init.q1 /= norm;
            init.q2 /= norm;
            init.q3 /= norm;
        }
    }

    s_6axis_state = init;
    s_9axis_state = init;
}

bool attitude_solve_6axis(const attitude_vector_t *acc_g,
                          const attitude_vector_t *gyro_dps,
                          float sample_time_s,
                          attitude_t *attitude_deg)
{
    const float dt = sample_time_s;
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float vx;
    float vy;
    float vz;
    float ex;
    float ey;
    float ez;
    float norm;

    if ((acc_g == NULL)
        || (gyro_dps == NULL)
        || (attitude_deg == NULL)
        || (dt <= 0.0f))
    {
        return false;
    }

    ax = acc_g->x;
    ay = acc_g->y;
    az = acc_g->z;
    gx = gyro_dps->x * ATTITUDE_DEG_TO_RAD;
    gy = gyro_dps->y * ATTITUDE_DEG_TO_RAD;
    gz = gyro_dps->z * ATTITUDE_DEG_TO_RAD;

    norm = sqrtf(ax * ax + ay * ay + az * az);
    /* Motion acceleration is not a reliable gravity reference. */
    if ((norm >= ATTITUDE_ACCEL_MIN_G) && (norm <= ATTITUDE_ACCEL_MAX_G))
    {
        ax /= norm;
        ay /= norm;
        az /= norm;

        vx = 2.0f * (s_6axis_state.q1 * s_6axis_state.q3
                     - s_6axis_state.q0 * s_6axis_state.q2);
        vy = 2.0f * (s_6axis_state.q0 * s_6axis_state.q1
                     + s_6axis_state.q2 * s_6axis_state.q3);
        vz = 1.0f - 2.0f * (s_6axis_state.q1 * s_6axis_state.q1
                            + s_6axis_state.q2 * s_6axis_state.q2);

        ex = ay * vz - az * vy;
        ey = az * vx - ax * vz;
        ez = ax * vy - ay * vx;

        /* Avoid learning filter lag / vigorous-motion error as gyro bias. */
        if ((fabsf(gyro_dps->x) <= ATTITUDE_BIAS_MAX_RATE_DPS)
            && (fabsf(gyro_dps->y) <= ATTITUDE_BIAS_MAX_RATE_DPS)
            && (fabsf(gyro_dps->z) <= ATTITUDE_BIAS_MAX_RATE_DPS))
        {
            s_6axis_state.ex_int += ex * dt;
            s_6axis_state.ey_int += ey * dt;
            s_6axis_state.ez_int += ez * dt;
        }

        gx += ATTITUDE_KP * ex;
        gy += ATTITUDE_KP * ey;
        gz += ATTITUDE_KP * ez;
    }

    /* Retain the previously learned correction while gravity is unreliable. */
    gx += ATTITUDE_KI * s_6axis_state.ex_int;
    gy += ATTITUDE_KI * s_6axis_state.ey_int;
    gz += ATTITUDE_KI * s_6axis_state.ez_int;

    return attitude_update(&s_6axis_state, gx, gy, gz, dt, attitude_deg);
}

bool attitude_solve_9axis(const attitude_vector_t *acc_g,
                          const attitude_vector_t *gyro_dps,
                          const attitude_vector_t *mag_ut,
                          float sample_time_s,
                          attitude_t *attitude_deg)
{
    const float dt = sample_time_s;
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float mx;
    float my;
    float mz;
    float q0q1;
    float q0q2;
    float q0q3;
    float q1q1;
    float q1q2;
    float q1q3;
    float q2q2;
    float q2q3;
    float q3q3;
    float hx;
    float hy;
    float bx;
    float bz;
    float vx;
    float vy;
    float vz;
    float wx;
    float wy;
    float wz;
    float ex;
    float ey;
    float ez;
    float norm;

    if ((acc_g == NULL)
        || (gyro_dps == NULL)
        || (mag_ut == NULL)
        || (attitude_deg == NULL)
        || (dt <= 0.0f))
    {
        return false;
    }

    ax = acc_g->x;
    ay = acc_g->y;
    az = acc_g->z;
    gx = gyro_dps->x * ATTITUDE_DEG_TO_RAD;
    gy = gyro_dps->y * ATTITUDE_DEG_TO_RAD;
    gz = gyro_dps->z * ATTITUDE_DEG_TO_RAD;
    mx = mag_ut->x;
    my = mag_ut->y;
    mz = mag_ut->z;

    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < ATTITUDE_MIN_NORM)
    {
        return false;
    }
    ax /= norm;
    ay /= norm;
    az /= norm;

    norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm < ATTITUDE_MIN_NORM)
    {
        return false;
    }
    mx /= norm;
    my /= norm;
    mz /= norm;

    q0q1 = s_9axis_state.q0 * s_9axis_state.q1;
    q0q2 = s_9axis_state.q0 * s_9axis_state.q2;
    q0q3 = s_9axis_state.q0 * s_9axis_state.q3;
    q1q1 = s_9axis_state.q1 * s_9axis_state.q1;
    q1q2 = s_9axis_state.q1 * s_9axis_state.q2;
    q1q3 = s_9axis_state.q1 * s_9axis_state.q3;
    q2q2 = s_9axis_state.q2 * s_9axis_state.q2;
    q2q3 = s_9axis_state.q2 * s_9axis_state.q3;
    q3q3 = s_9axis_state.q3 * s_9axis_state.q3;

    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3)
                  + my * (q1q2 - q0q3)
                  + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3)
                  + my * (0.5f - q1q1 - q3q3)
                  + mz * (q2q3 - q0q1));
    bx = sqrtf(hx * hx + hy * hy);
    bz = 2.0f * (mx * (q1q3 - q0q2)
                  + my * (q2q3 + q0q1)
                  + mz * (0.5f - q1q1 - q2q2));

    vx = 2.0f * (q1q3 - q0q2);
    vy = 2.0f * (q0q1 + q2q3);
    vz = 1.0f - 2.0f * (q1q1 + q2q2);

    wx = 2.0f * (bx * (0.5f - q2q2 - q3q3)
                 + bz * (q1q3 - q0q2));
    wy = 2.0f * (bx * (q1q2 - q0q3)
                 + bz * (q0q1 + q2q3));
    wz = 2.0f * (bx * (q0q2 + q1q3)
                 + bz * (0.5f - q1q1 - q2q2));

    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

    s_9axis_state.ex_int += ex * dt;
    s_9axis_state.ey_int += ey * dt;
    s_9axis_state.ez_int += ez * dt;

    gx += ATTITUDE_KP * ex + ATTITUDE_KI * s_9axis_state.ex_int;
    gy += ATTITUDE_KP * ey + ATTITUDE_KI * s_9axis_state.ey_int;
    gz += ATTITUDE_KP * ez + ATTITUDE_KI * s_9axis_state.ez_int;

    return attitude_update(&s_9axis_state, gx, gy, gz, dt, attitude_deg);
}
