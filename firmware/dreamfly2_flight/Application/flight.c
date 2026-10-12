/**
 * @file flight.c
 * @brief Flight-state management implementation.
 * @author 梅雨
 * @date 2026-10-08
 */

#include "flight.h"

#include <stdint.h>

#include "led.h"
#include "battery.h"
#include "ws2812b.h"
#include "pid.h"

#define FLIGHT_RGB_BRIGHTNESS 64U
#define FLIGHT_ERROR_FLASH_COUNT 5U

/* Adjust these four indices after confirming the LEDs' physical positions. */
#define FLIGHT_LEFT_LED_1 0U
#define FLIGHT_LEFT_LED_2 1U
#define FLIGHT_RIGHT_LED_1 2U
#define FLIGHT_RIGHT_LED_2 3U

static float s_battery_voltage_v;
static pid_t s_roll_pid;
static pid_t s_pitch_pid;
static pid_t s_yaw_pid;
static flight_status_t s_flight_status = FLIGHT_STATUS_WAIT_PAIRING;
static flight_sensor_t s_sensor_error = FLIGHT_SENSOR_NONE;
static uint8_t s_error_flash_count;
static uint8_t s_error_visible = 1U;

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
            case FLIGHT_SENSOR_IMU:
                flight_rgb_all(FLIGHT_RGB_BRIGHTNESS, 0U, 0U);
                break;

            case FLIGHT_SENSOR_MAGNETOMETER:
                flight_rgb_all(FLIGHT_RGB_BRIGHTNESS, 24U, 0U);
                break;

            case FLIGHT_SENSOR_BAROMETER:
                flight_rgb_all(0U, 0U, FLIGHT_RGB_BRIGHTNESS);
                break;

            case FLIGHT_SENSOR_OPTICAL_FLOW:
                flight_rgb_all(0U, FLIGHT_RGB_BRIGHTNESS, FLIGHT_RGB_BRIGHTNESS);
                break;

            case FLIGHT_SENSOR_RANGEFINDER:
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

void flight_init(void)
{
    led_init();
    ws2812b_init();

    /* P-only bench demonstration; these gains are not tuned for flight. */
    pid_init(&s_roll_pid, 1.0f, 0.0f, 0.0f, 10.0f, 50.0f, 100.0f);
    pid_init(&s_pitch_pid, 1.0f, 0.0f, 0.0f, 10.0f, 50.0f, 100.0f);
    pid_init(&s_yaw_pid, 1.0f, 0.0f, 0.0f, 10.0f, 50.0f, 100.0f);
}

void flight_rate_control(const flight_axis_t *target_dps,
                         const flight_axis_t *measurement_dps,
                         float sample_time_s,
                         bool integral_enabled,
                         flight_axis_t *output)
{
    s_roll_pid.target = target_dps->roll;
    s_roll_pid.measurement = measurement_dps->roll;
    pid_calculate(&s_roll_pid, sample_time_s, integral_enabled);

    s_pitch_pid.target = target_dps->pitch;
    s_pitch_pid.measurement = measurement_dps->pitch;
    pid_calculate(&s_pitch_pid, sample_time_s, integral_enabled);

    s_yaw_pid.target = target_dps->yaw;
    s_yaw_pid.measurement = measurement_dps->yaw;
    pid_calculate(&s_yaw_pid, sample_time_s, integral_enabled);

    output->roll = s_roll_pid.output;
    output->pitch = s_pitch_pid.output;
    output->yaw = s_yaw_pid.output;
}

void flight_led_toggle(void)
{
    led_toggle();
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
