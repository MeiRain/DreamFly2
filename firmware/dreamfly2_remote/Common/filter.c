/**
 * @file filter.c
 * @brief Moving-average filter implementation.
 * @author 梅雨
 * @date 2026-10-26
 */

#include "filter.h"

#include <stddef.h>
#include <string.h>

void filter_moving_average_init(filter_moving_average_t *filter)
{
    if (filter != NULL)
    {
        memset(filter, 0, sizeof(*filter));
    }
}

float filter_moving_average(filter_moving_average_t *filter, float input)
{
    if (filter == NULL)
    {
        return input;
    }

    filter->sum -= filter->samples[filter->index];
    filter->samples[filter->index] = input;
    filter->sum += input;

    filter->index = (uint8_t)((filter->index + 1U) % FILTER_MOVING_AVERAGE_WINDOW_SIZE);
    if (filter->count < FILTER_MOVING_AVERAGE_WINDOW_SIZE)
    {
        ++filter->count;
    }

    return filter->sum / filter->count;
}
