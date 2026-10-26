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
             index++)
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

void filter_kalman_1d_init(filter_kalman_1d_t *filter,
                           float process_noise,
                           float measurement_noise,
                           float initial_covariance,
                           float initial_output)
{
    if (filter == NULL)
    {
        return;
    }

    filter->process_noise = process_noise;
    filter->measurement_noise = measurement_noise;
    filter->covariance = initial_covariance;
    filter->output = initial_output;
}

float filter_kalman_1d(filter_kalman_1d_t *filter, float input)
{
    float p_pred;
    float k;

    if (filter == NULL)
    {
        return input;
    }

    p_pred = filter->covariance + filter->process_noise;
    k = p_pred / (p_pred + filter->measurement_noise);

    filter->output += k * (input - filter->output);
    filter->covariance = (1.0f - k) * p_pred;

    return filter->output;
}

void filter_low_pass_init(filter_low_pass_t *filter,
                          float factor,
                          float initial_output)
{
    if (filter == NULL)
    {
        return;
    }

    filter->factor = factor;
    filter->output = initial_output;
}

float filter_low_pass(filter_low_pass_t *filter, float input)
{
    if (filter == NULL)
    {
        return input;
    }

    filter->output = filter->factor * input
                     + (1.0f - filter->factor) * filter->output;

    return filter->output;
}
