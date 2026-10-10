/**
 * @file flight.h
 * @brief Public interface for flight-state management.
 * @author 梅雨
 * @date 2026-10-08
 */

#ifndef DREAMFLY2_APPLICATION_FLIGHT_H
#define DREAMFLY2_APPLICATION_FLIGHT_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} flight_imu_t;

typedef enum
{
    FLIGHT_STATUS_WAIT_PAIRING = 0,
    FLIGHT_STATUS_PAIRED,
    FLIGHT_STATUS_FLYING
} flight_status_t;

typedef enum
{
    FLIGHT_SENSOR_NONE = 0,
    FLIGHT_SENSOR_ICM20948,
    FLIGHT_SENSOR_AK09916,
    FLIGHT_SENSOR_BMP280,
    FLIGHT_SENSOR_PMW3901,
    FLIGHT_SENSOR_VL53L1
} flight_sensor_t;

/**
 * @brief Initialize the flight application, IMU and indicators.
 * @return true if the ICM-20948 initialized successfully.
 */
bool flight_init(void);

/** @brief Toggle the on-board flight status LED. */
void flight_led_toggle(void);

/**
 * @brief Read, calibrate and filter one IMU sample.
 * @param[out] data Processed acceleration in g and angular rate in degrees/s.
 * @return true when processed data is available; false while calibrating or
 *         when the sensor read fails.
 * @note Keep the aircraft stationary during the initial calibration samples.
 */
bool flight_imu_update(flight_imu_t *data);

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
