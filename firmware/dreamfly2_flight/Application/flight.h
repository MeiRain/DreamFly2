/**
 * @file flight.h
 * @brief Public interface for flight-state management.
 * @author 梅雨
 * @date 2026-10-08
 */

#ifndef DREAMFLY2_APPLICATION_FLIGHT_H
#define DREAMFLY2_APPLICATION_FLIGHT_H

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
 */
void flight_init(void);

/** @brief Toggle the on-board flight status LED. */
void flight_led_toggle(void);

/** @brief Set the current flight status shown by the RGB LEDs. */
void flight_set_status(flight_status_t status);

/**
 * @brief Set the sensor fault shown by the RGB LEDs.
 * @note Pass FLIGHT_SENSOR_NONE to clear the fault indication.
 */
void flight_set_sensor_error(flight_sensor_t sensor);

/** @brief Update the RGB indication for the current flight state. */
void flight_update_rgb(void);

/**
 * @brief Read and update the current battery voltage.
 * @return Latest battery voltage in volts.
 */
float flight_update_battery(void);

#endif /* DREAMFLY2_APPLICATION_FLIGHT_H */
