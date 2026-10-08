/**
 * @file filter.c
 * @brief Common numerical filter implementations.
 * @author 梅雨
 * @date 2026-10-08
 */

#include "filter.h"

#include <stddef.h>

#define FILTER_MOVING_AVERAGE_MAX_WINDOW_SIZE 20U

void filter_moving_average(float data, float *filtered_data, uint8_t window_size)
{
    static float s_buffer[FILTER_MOVING_AVERAGE_MAX_WINDOW_SIZE];
    static float s_sum;
    static uint8_t s_index;
    static uint8_t s_sample_count;
    static uint8_t s_window_size;

    if ((filtered_data == NULL)
        || (window_size == 0U)
        || (window_size > FILTER_MOVING_AVERAGE_MAX_WINDOW_SIZE))
    {
        return;
    }

    if (s_window_size != window_size)
    {
        for (uint8_t index = 0U;
             index < FILTER_MOVING_AVERAGE_MAX_WINDOW_SIZE;
             ++index)
        {
            s_buffer[index] = 0.0f;
        }

        s_sum = 0.0f;
        s_index = 0U;
        s_sample_count = 0U;
        s_window_size = window_size;
    }

    s_sum -= s_buffer[s_index];
    s_buffer[s_index] = data;
    s_sum += data;

    ++s_index;
    if (s_index >= window_size)
    {
        s_index = 0U;
    }

    if (s_sample_count < window_size)
    {
        ++s_sample_count;
    }

    *filtered_data = s_sum / (float)s_sample_count;
}

float filter_kalman_1d_update(filter_kalman_1d_t *filter, float input)
{
    float predicted_covariance;
    float kalman_gain;

    if (filter == NULL)
    {
        return input;
    }

    predicted_covariance = filter->covariance + filter->process_noise;
    kalman_gain = predicted_covariance
                  / (predicted_covariance + filter->measurement_noise);

    filter->output += kalman_gain * (input - filter->output);
    filter->covariance = (1.0f - kalman_gain) * predicted_covariance;

    return filter->output;
}

float filter_low_pass_update(filter_low_pass_t *filter, float input)
{
    if (filter == NULL)
    {
        return input;
    }

    filter->output = filter->factor * input
                     + (1.0f - filter->factor) * filter->output;

    return filter->output;
}
