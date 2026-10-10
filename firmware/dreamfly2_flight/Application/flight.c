/**
 * @file flight.c
 * @brief Flight-state management implementation.
 * @author 梅雨
 * @date 2026-10-08
 */

#include "flight.h"

#include <stddef.h>

#include "led.h"
#include "battery.h"
#include "filter.h"
#include "icm20948.h"
#include "ws2812b.h"

#define FLIGHT_RGB_BRIGHTNESS 64U
#define FLIGHT_ERROR_FLASH_COUNT 5U

#define FLIGHT_IMU_CALIBRATION_SAMPLES 256U
#define FLIGHT_IMU_STATIONARY_GYRO_DELTA 5
#define FLIGHT_IMU_STATIONARY_GYRO_LIMIT 100
#define FLIGHT_ACCEL_LSB_PER_G 8192.0f
#define FLIGHT_GYRO_LSB_PER_DPS 16.4f
#define FLIGHT_ACCEL_FILTER_PROCESS_NOISE 0.001f
#define FLIGHT_ACCEL_FILTER_MEASUREMENT_NOISE 0.543f
#define FLIGHT_ACCEL_FILTER_INITIAL_COVARIANCE 0.02f
#define FLIGHT_GYRO_FILTER_FACTOR 0.85f

/* Adjust these four indices after confirming the LEDs' physical positions. */
#define FLIGHT_LEFT_LED_1 0U
#define FLIGHT_LEFT_LED_2 1U
#define FLIGHT_RIGHT_LED_1 2U
#define FLIGHT_RIGHT_LED_2 3U

static float s_battery_voltage_v;
static flight_status_t s_flight_status = FLIGHT_STATUS_WAIT_PAIRING;
static flight_sensor_t s_sensor_error = FLIGHT_SENSOR_NONE;
static uint8_t s_error_flash_count;
static uint8_t s_error_visible = 1U;

static int32_t s_gyro_bias_sum[3];
static float s_gyro_bias[3];
static int16_t s_last_gyro[3];
static uint16_t s_imu_calibration_count;
static bool s_last_gyro_valid;
static bool s_imu_calibrated;
static bool s_imu_filter_initialized;

static filter_kalman_1d_t s_accel_filter[3];
static filter_low_pass_t s_gyro_filter[3];

static void flight_imu_reset_processing(void)
{
    for (uint8_t axis = 0U; axis < 3U; ++axis)
    {
        s_gyro_bias_sum[axis] = 0;
        s_gyro_bias[axis] = 0.0f;
        s_last_gyro[axis] = 0;

        s_accel_filter[axis].process_noise = FLIGHT_ACCEL_FILTER_PROCESS_NOISE;
        s_accel_filter[axis].measurement_noise = FLIGHT_ACCEL_FILTER_MEASUREMENT_NOISE;
        s_accel_filter[axis].covariance = FLIGHT_ACCEL_FILTER_INITIAL_COVARIANCE;
        s_accel_filter[axis].output = 0.0f;

        s_gyro_filter[axis].factor = FLIGHT_GYRO_FILTER_FACTOR;
        s_gyro_filter[axis].output = 0.0f;
    }

    s_imu_calibration_count = 0U;
    s_last_gyro_valid = false;
    s_imu_calibrated = false;
    s_imu_filter_initialized = false;
}

static bool flight_imu_is_stationary(const icm20948_t *raw)
{
    int32_t gyro[3] = {raw->gyro_x, raw->gyro_y, raw->gyro_z};
    bool stationary = true;

    if (!s_last_gyro_valid)
    {
        stationary = false;
        s_last_gyro_valid = true;
    }
    else
    {
        for (uint8_t axis = 0U; axis < 3U; ++axis)
        {
            int32_t delta = gyro[axis] - s_last_gyro[axis];

            if ((delta < -FLIGHT_IMU_STATIONARY_GYRO_DELTA)
                || (delta > FLIGHT_IMU_STATIONARY_GYRO_DELTA)
                || (gyro[axis] < -FLIGHT_IMU_STATIONARY_GYRO_LIMIT)
                || (gyro[axis] > FLIGHT_IMU_STATIONARY_GYRO_LIMIT))
            {
                stationary = false;
            }
        }
    }

    s_last_gyro[0] = raw->gyro_x;
    s_last_gyro[1] = raw->gyro_y;
    s_last_gyro[2] = raw->gyro_z;

    return stationary;
}

static void flight_rgb_all(uint8_t red, uint8_t green, uint8_t blue)
{
    ws2812b_set_all(red, green, blue);
    ws2812b_show();
}

static void flight_show_flying_rgb(void)
{
    ws2812b_set_pixel(FLIGHT_LEFT_LED_1, FLIGHT_RGB_BRIGHTNESS, 0U, 0U);
    ws2812b_set_pixel(FLIGHT_LEFT_LED_2, FLIGHT_RGB_BRIGHTNESS, 0U, 0U);
    ws2812b_set_pixel(FLIGHT_RIGHT_LED_1, 0U, FLIGHT_RGB_BRIGHTNESS, 0U);
    ws2812b_set_pixel(FLIGHT_RIGHT_LED_2, 0U, FLIGHT_RGB_BRIGHTNESS, 0U);
    ws2812b_show();
}

static void flight_show_sensor_error(void)
{
    if (s_error_visible == 0U)
    {
        flight_rgb_all(0U, 0U, 0U);
    }
    else
    {
        switch (s_sensor_error)
        {
            case FLIGHT_SENSOR_ICM20948:
                flight_rgb_all(FLIGHT_RGB_BRIGHTNESS, 0U, 0U);
                break;

            case FLIGHT_SENSOR_AK09916:
                flight_rgb_all(FLIGHT_RGB_BRIGHTNESS, 24U, 0U);
                break;

            case FLIGHT_SENSOR_BMP280:
                flight_rgb_all(0U, 0U, FLIGHT_RGB_BRIGHTNESS);
                break;

            case FLIGHT_SENSOR_PMW3901:
                flight_rgb_all(0U, FLIGHT_RGB_BRIGHTNESS, FLIGHT_RGB_BRIGHTNESS);
                break;

            case FLIGHT_SENSOR_VL53L1:
                flight_rgb_all(FLIGHT_RGB_BRIGHTNESS, 0U, FLIGHT_RGB_BRIGHTNESS);
                break;

            default:
                flight_rgb_all(FLIGHT_RGB_BRIGHTNESS, FLIGHT_RGB_BRIGHTNESS, FLIGHT_RGB_BRIGHTNESS);
                break;
        }
    }

    ++s_error_flash_count;
    if (s_error_flash_count >= FLIGHT_ERROR_FLASH_COUNT)
    {
        s_error_flash_count = 0U;
        s_error_visible = (uint8_t)!s_error_visible;
    }
}

bool flight_init(void)
{
    led_init();
    ws2812b_init();
    flight_imu_reset_processing();

    return icm20948_init();
}

void flight_led_toggle(void)
{
    led_toggle();
}

bool flight_imu_update(flight_imu_t *data)
{
    icm20948_t raw;
    float accel[3];
    float gyro[3];

    if ((data == NULL) || !icm20948_read(&raw))
    {
        return false;
    }

    if (!s_imu_calibrated)
    {
        if (!flight_imu_is_stationary(&raw))
        {
            for (uint8_t axis = 0U; axis < 3U; ++axis)
            {
                s_gyro_bias_sum[axis] = 0;
            }

            s_imu_calibration_count = 0U;
            return false;
        }

        s_gyro_bias_sum[0] += raw.gyro_x;
        s_gyro_bias_sum[1] += raw.gyro_y;
        s_gyro_bias_sum[2] += raw.gyro_z;
        ++s_imu_calibration_count;

        if (s_imu_calibration_count >= FLIGHT_IMU_CALIBRATION_SAMPLES)
        {
            for (uint8_t axis = 0U; axis < 3U; ++axis)
            {
                s_gyro_bias[axis] = (float)s_gyro_bias_sum[axis]
                                    / (float)FLIGHT_IMU_CALIBRATION_SAMPLES;
            }

            s_imu_calibrated = true;
        }

        return false;
    }

    /* Adjust this mapping here if the sensor orientation changes. */
    accel[0] = (float)raw.accel_x / FLIGHT_ACCEL_LSB_PER_G;
    accel[1] = (float)raw.accel_y / FLIGHT_ACCEL_LSB_PER_G;
    accel[2] = (float)raw.accel_z / FLIGHT_ACCEL_LSB_PER_G;
    gyro[0] = ((float)raw.gyro_x - s_gyro_bias[0]) / FLIGHT_GYRO_LSB_PER_DPS;
    gyro[1] = ((float)raw.gyro_y - s_gyro_bias[1]) / FLIGHT_GYRO_LSB_PER_DPS;
    gyro[2] = ((float)raw.gyro_z - s_gyro_bias[2]) / FLIGHT_GYRO_LSB_PER_DPS;

    if (!s_imu_filter_initialized)
    {
        for (uint8_t axis = 0U; axis < 3U; ++axis)
        {
            s_accel_filter[axis].output = accel[axis];
            s_gyro_filter[axis].output = gyro[axis];
        }

        s_imu_filter_initialized = true;
    }
    else
    {
        for (uint8_t axis = 0U; axis < 3U; ++axis)
        {
            accel[axis] = filter_kalman_1d_update(&s_accel_filter[axis],
                                                  accel[axis]);
            gyro[axis] = filter_low_pass_update(&s_gyro_filter[axis],
                                                 gyro[axis]);
        }
    }

    data->accel_x_g = accel[0];
    data->accel_y_g = accel[1];
    data->accel_z_g = accel[2];
    data->gyro_x_dps = gyro[0];
    data->gyro_y_dps = gyro[1];
    data->gyro_z_dps = gyro[2];

    return true;
}

void flight_set_status(flight_status_t status)
{
    s_flight_status = status;
}

void flight_set_sensor_error(flight_sensor_t sensor)
{
    s_sensor_error = sensor;
    s_error_flash_count = 0U;
    s_error_visible = 1U;
}

void flight_update_rgb(void)
{
    if (s_sensor_error != FLIGHT_SENSOR_NONE)
    {
        flight_show_sensor_error();
        return;
    }

    switch (s_flight_status)
    {
        case FLIGHT_STATUS_PAIRED:
            ws2812b_gradient();
            break;

        case FLIGHT_STATUS_FLYING:
            flight_show_flying_rgb();
            break;

        case FLIGHT_STATUS_WAIT_PAIRING:
        default:
            flight_rgb_all(FLIGHT_RGB_BRIGHTNESS, FLIGHT_RGB_BRIGHTNESS, 0U);
            break;
    }
}

float flight_update_battery(void)
{
    s_battery_voltage_v = battery_read_voltage();

    return s_battery_voltage_v;
}
