/**
 * @file pid.c
 * @brief Generic PID controller implementation.
 * @author 梅雨
 * @date 2026-10-11
 */

#include "pid.h"

#include <stddef.h>
#include <math.h>

/** @brief Clamp a value symmetrically to +/-limit; limit must be nonnegative. */
static float pid_limit(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }

    if (value < -limit)
    {
        return -limit;
    }

    return value;
}

void pid_init(pid_t *pid,
              float kp,
              float ki,
              float kd,
              float integral_threshold,
              float integral_limit,
              float output_limit)
{
    if (pid == NULL)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->target = 0.0f;
    pid->measurement = 0.0f;
    pid->integral_threshold = integral_threshold;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
    pid_reset(pid);
}

void pid_reset(pid_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->error = 0.0f;
    pid->output = 0.0f;
}

void pid_calculate(pid_t *pid,
                    float sample_time_s,
                    bool integral_enabled)
{
    float derivative;

    if (pid == NULL)
    {
        return;
    }

    if ((sample_time_s <= 0.0f)
        || (pid->integral_threshold < 0.0f)
        || (pid->integral_limit < 0.0f)
        || (pid->output_limit <= 0.0f))
    {
        pid->output = 0.0f;
        return;
    }

    pid->error = pid->target - pid->measurement;
    derivative = (pid->error - pid->previous_error) / sample_time_s;

    if (integral_enabled)
    {
        if ((pid->integral_threshold == 0.0f)
            || (fabsf(pid->error) < pid->integral_threshold))
        {
            pid->integral += pid->error * sample_time_s;
            pid->integral = pid_limit(pid->integral, pid->integral_limit);
        }
    }
    else
    {
        pid->integral = 0.0f;
    }

    pid->output = pid->kp * pid->error
             + pid->ki * pid->integral
             + pid->kd * derivative;
    pid->previous_error = pid->error;

    pid->output = pid_limit(pid->output, pid->output_limit);
}
