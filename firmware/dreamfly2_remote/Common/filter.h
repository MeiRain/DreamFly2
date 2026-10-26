/**
 * @file filter.h
 * @brief Reusable moving-average filter with caller-owned state.
 * @author 梅雨
 * @date 2026-10-26
 */

#ifndef DREAMFLY2_REMOTE_COMMON_FILTER_H
#define DREAMFLY2_REMOTE_COMMON_FILTER_H

#include <stdint.h>

#define FILTER_MOVING_AVERAGE_WINDOW_SIZE 10U

typedef struct
{
    float samples[FILTER_MOVING_AVERAGE_WINDOW_SIZE];
    float sum;
    uint8_t index;
    uint8_t count; /**< Available samples; startup zeros are not averaged. */
} filter_moving_average_t;

/** @brief Clear one stream's filter history; NULL is ignored. */
void filter_moving_average_init(filter_moving_average_t *filter);

/**
 * @brief Add a sample and return the mean of the latest available samples.
 * @param filter Independent stream state, initialized or zero-initialized.
 * @param input New sample, in the caller's units.
 * @return Filtered value in the same units; input is returned for a NULL filter.
 * @note Fixed 10-sample window, no HAL/RTOS dependencies. Call from one context
 *       per instance; do not share one instance between different channels.
 */
float filter_moving_average(filter_moving_average_t *filter, float input);

#endif /* DREAMFLY2_REMOTE_COMMON_FILTER_H */
