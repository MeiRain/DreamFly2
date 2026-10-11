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
 * @brief Initialize a one-dimensional Kalman filter.
 * @param filter Filter instance.
 * @param process_noise Process-noise covariance.
 * @param measurement_noise Measurement-noise covariance.
 * @param initial_covariance Initial estimate covariance.
 * @param initial_output Initial filtered value.
 */
void filter_kalman_1d_init(filter_kalman_1d_t *filter,
                           float process_noise,
                           float measurement_noise,
                           float initial_covariance,
                           float initial_output);

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
float filter_kalman_1d(filter_kalman_1d_t *filter, float input);

/**
 * @brief Initialize a first-order low-pass filter.
 * @param filter Filter instance.
 * @param factor Weight of the new input, normally in the range [0, 1].
 * @param initial_output Initial filtered value.
 */
void filter_low_pass_init(filter_low_pass_t *filter,
                          float factor,
                          float initial_output);

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
float filter_low_pass(filter_low_pass_t *filter, float input);

#endif /* DREAMFLY2_COMMON_FILTER_H */
