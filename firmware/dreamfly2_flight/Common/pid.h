/**
 * @file pid.h
 * @brief Generic PID controller interface.
 * @author 梅雨
 * @date 2026-10-11
 */

#ifndef DREAMFLY2_COMMON_PID_H
#define DREAMFLY2_COMMON_PID_H

#include <stdbool.h>

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float previous_error;
    float integral_limit;
    float output_limit;
} pid_t;

/** @brief Initialize PID gains and limits, then clear controller state. */
void pid_init(pid_t *pid,
              float kp,
              float ki,
              float kd,
              float integral_limit,
              float output_limit);

/** @brief Clear the accumulated integral and previous error. */
void pid_reset(pid_t *pid);

/**
 * @brief Calculate one PID output.
 * @param pid PID controller state.
 * @param target Desired value.
 * @param measurement Measured value in the same unit as target.
 * @param sample_time_s Controller period in seconds.
 * @param integral_enabled Enable integral accumulation when true.
 * @return Limited controller output, or zero when the arguments are invalid.
 */
float pid_calculate(pid_t *pid,
                    float target,
                    float measurement,
                    float sample_time_s,
                    bool integral_enabled);

#endif /* DREAMFLY2_COMMON_PID_H */
