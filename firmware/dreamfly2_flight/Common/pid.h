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
    float target;
    float measurement;
    float error;
    float output;
    float integral;
    float previous_error;
    float integral_threshold; /**< Accumulate only below this absolute error; zero disables separation. */
    float integral_limit;
    float output_limit;
} pid_t;

/** @brief Initialize PID gains and limits, then clear controller state. */
void pid_init(pid_t *pid,
              float kp,
              float ki,
              float kd,
              float integral_threshold,
              float integral_limit,
              float output_limit);

/** @brief Clear error, output, accumulated integral and previous error. */
void pid_reset(pid_t *pid);

/**
 * @brief Calculate one PID output.
 * @param pid PID controller state.
 * @details Set target and measurement before calling; the result is stored in output.
 *          Outside integral_threshold, the existing integral is retained without accumulation.
 * @param sample_time_s Controller period in seconds.
 * @param integral_enabled Enable integral accumulation when true.
 */
void pid_calculate(pid_t *pid,
                    float sample_time_s,
                    bool integral_enabled);

#endif /* DREAMFLY2_COMMON_PID_H */
