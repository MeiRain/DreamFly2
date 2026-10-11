/**
 * @file pid.c
 * @brief Generic PID controller implementation.
 * @author 梅雨
 * @date 2026-10-11
 */

#include "pid.h"

#include <stddef.h>

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
}

float pid_calculate(pid_t *pid,
                    float target,
                    float measurement,
                    float sample_time_s,
                    bool integral_enabled)
{
    float error;
    float derivative;
    float output;

    if ((pid == NULL)
        || (sample_time_s <= 0.0f)
        || (pid->integral_limit < 0.0f)
        || (pid->output_limit <= 0.0f))
    {
        return 0.0f;
    }

    error = target - measurement;
    derivative = (error - pid->previous_error) / sample_time_s;

    if (integral_enabled)
    {
        pid->integral += error * sample_time_s;
        pid->integral = pid_limit(pid->integral, pid->integral_limit);
    }
    else
    {
        pid->integral = 0.0f;
    }

    output = pid->kp * error
             + pid->ki * pid->integral
             + pid->kd * derivative;
    pid->previous_error = error;

    return pid_limit(output, pid->output_limit);
}
