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
    float integral; /**< Accumulated error * seconds, before multiplication by ki. */
    float previous_error;
    float integral_threshold; /**< Accumulate only below this absolute error; zero disables separation. */
    float integral_limit; /**< Symmetric limit on integral, not on ki * integral. */
    float output_limit; /**< Symmetric limit on the final controller output. */
} pid_t;

/**
 * @brief Initialize PID gains and limits, then clear controller state.
 * @param pid Controller instance; NULL is ignored.
 * @param kp Proportional gain.
 * @param ki Gain applied to accumulated error * seconds.
 * @param kd Gain applied to error change per second.
 * @param integral_threshold Absolute error threshold; 0 disables separation.
 * @param integral_limit Nonnegative limit on accumulated error * seconds.
 * @param output_limit Positive limit on controller output magnitude.
 */
void pid_init(pid_t *pid,
              float kp,
              float ki,
              float kd,
              float integral_threshold,
              float integral_limit,
              float output_limit);

/** @brief Clear dynamic state, retaining gains, limits, target and measurement; NULL is ignored. */
void pid_reset(pid_t *pid);

/**
 * @brief Calculate one PID output.
 * @param[in,out] pid PID controller state; NULL is ignored.
 * @details Set target and measurement before calling; the result is stored in output.
 *          Outside integral_threshold, the existing integral is retained without accumulation.
 * @param sample_time_s Controller period in seconds.
 * @param integral_enabled Enable integral accumulation when true.
 * @note Disabling integration clears the integral. Invalid period or limits set
 *       output to zero. Target and measurement must use the same unit.
 */
void pid_calculate(pid_t *pid,
                    float sample_time_s,
                    bool integral_enabled);

#endif /* DREAMFLY2_COMMON_PID_H */
