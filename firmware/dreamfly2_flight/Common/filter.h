/**
 * @file filter.h
 * @brief Common numerical filters used by the flight-control application.
 * @author 梅雨
 * @date 2026-10-08
 */

#ifndef DREAMFLY2_COMMON_FILTER_H
#define DREAMFLY2_COMMON_FILTER_H

#include <stdint.h>

typedef struct
{
    float process_noise;
    float measurement_noise;
    float covariance;
    float output;
} filter_kalman_1d_t;

typedef struct
{
    float factor;
    float output;
} filter_low_pass_t;

/**
 * @brief Apply a moving-average filter to one data stream.
 *
 * The function maintains one internal window. During start-up, it averages
 * the samples currently available instead of waiting for the window to fill.
 *
 * @param data New sample.
 * @param[out] filtered_data Filtered result.
 * @param window_size Number of samples, from 1 to 20.
 */
void filter_moving_average(float data, float *filtered_data, uint8_t window_size);

/**
 * @brief Update a one-dimensional Kalman filter.
 *
 * @param filter Filter state and tuning parameters.
 * @param input New measurement.
 * @return Updated filtered value. If filter is NULL, input is returned.
 */
float filter_kalman_1d_update(filter_kalman_1d_t *filter, float input);

/**
 * @brief Update a first-order low-pass filter.
 *
 * The factor is normally in the range [0, 1]. A larger factor gives more
 * weight to the new input.
 *
 * @param filter Filter state and factor.
 * @param input New sample.
 * @return Updated filtered value. If filter is NULL, input is returned.
 */
float filter_low_pass_update(filter_low_pass_t *filter, float input);

#endif /* DREAMFLY2_COMMON_FILTER_H */
