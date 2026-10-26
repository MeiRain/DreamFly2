/**
 * @file flight.h
 * @brief Public interface for flight-state management.
 * @author 梅雨
 * @date 2026-10-08
 */

#ifndef DREAMFLY2_APPLICATION_FLIGHT_H
#define DREAMFLY2_APPLICATION_FLIGHT_H

#include <stdbool.h>

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} flight_axis_t;

/**
 * @brief Calculate three rate PID outputs without applying them to the motors.
 * @param target_dps Target body-axis angular rates in degrees/s.
 * @param measurement_dps Measured body-axis angular rates in degrees/s.
 * @param sample_time_s Controller period in seconds.
 * @param integral_enabled Whether the PID integrals may accumulate.
 * @param[out] output Controller outputs, not motor compare values.
 * @note Call after flight_init(); all pointers must be valid. PID state is private
 *       to this module, so calls must be serialized. Current gains are bench defaults.
 */
void flight_rate_control(const flight_axis_t *target_dps,
                         const flight_axis_t *measurement_dps,
                         float sample_time_s,
                         bool integral_enabled,
                         flight_axis_t *output);

typedef enum
{
    FLIGHT_STATUS_WAIT_PAIRING = 0,
    FLIGHT_STATUS_PAIRED,
    FLIGHT_STATUS_FLYING
} flight_status_t;

typedef enum
{
    FLIGHT_SENSOR_NONE = 0,
    FLIGHT_SENSOR_IMU,
    FLIGHT_SENSOR_MAGNETOMETER,
    FLIGHT_SENSOR_BAROMETER,
    FLIGHT_SENSOR_OPTICAL_FLOW,
    FLIGHT_SENSOR_RANGEFINDER
} flight_sensor_t;

/**
 * @brief Initialize flight-state indicators.
 * @note Also initializes bench PID gains. GPIO, TIM2 and DMA must be ready first.
 */
void flight_init(void);

/** @brief Toggle the on-board flight status LED. */
void flight_led_toggle(void);

/** @brief Select an RGB indication state; does not change actual arming or flight mode. */
void flight_set_status(flight_status_t status);

/**
 * @brief Set the sensor fault shown by the RGB LEDs.
 * @note Pass FLIGHT_SENSOR_NONE to clear the fault indication.
 */
void flight_set_sensor_error(flight_sensor_t sensor);

/**
 * @brief Request the current fault or flight-state RGB indication.
 * @note Fault indication takes priority. Call periodically from one task;
 *       animation and fault blink speeds depend on the call interval.
 */
void flight_update_rgb(void);

/**
 * @brief Read and update the current battery voltage.
 * @return Latest battery voltage in volts.
 */
float flight_update_battery(void);

#endif /* DREAMFLY2_APPLICATION_FLIGHT_H */
