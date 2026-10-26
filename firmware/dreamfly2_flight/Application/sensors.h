/**
 * @file sensors.h
 * @brief Unified flight-sensor interface.
 * @author 梅雨
 * @date 2026-10-11
 */

#ifndef DREAMFLY2_APPLICATION_SENSORS_H
#define DREAMFLY2_APPLICATION_SENSORS_H

#include <stdbool.h>

typedef struct
{
    float acc_x_g;
    float acc_y_g;
    float acc_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} sensors_imu_t;

typedef struct
{
    float mag_x_ut;
    float mag_y_ut;
    float mag_z_ut;
} sensors_mag_t;

typedef struct
{
    float temp_c;
    float press_pa;
} sensors_baro_t;

typedef struct
{
    bool imu;
    bool magnetometer;
    bool barometer;
    bool optical_flow;
    bool rangefinder;
} sensors_status_t;

/**
 * @brief Initialize required and optional flight sensors.
 * @return true when the required IMU is ready.
 * @note Call from a running RTOS task after I2C initialization. Gyro calibration
 *       waits for consecutive stationary samples with no overall timeout.
 *       Optional sensor failures do not fail initialization. Status records
 *       initialization results, not continuous sensor health.
 */
bool sensors_init(void);

/**
 * @brief Calibrate accelerometer zero bias while the aircraft is level and still.
 * @return false if the IMU is unavailable, a read fails or the attempt limit is reached.
 * @note Blocking, user-triggered calibration; keep Z facing upward at +1 g.
 *       Results stay in RAM. Serialize calibration and reads in the sensor task.
 */
bool sensors_calibrate_accel_level(void);

/** @brief Return the detected sensor capabilities. */
sensors_status_t sensors_get_status(void);

/**
 * @brief Read one calibrated and filtered IMU sample.
 * @param[out] imu_data Acceleration in g and angular rate in degrees/s.
 * @return true on success; false for NULL, unavailable IMU or read failure.
 * @note Call after sensors_init(); output is unchanged on failure. Filtering
 *       uses one private state per axis and must be called serially.
 */
bool sensors_read_imu(sensors_imu_t *imu_data);

/**
 * @brief Read one magnetic-field sample.
 * @param[out] mag_data Magnetic field in microtesla.
 * @return true when the magnetometer is available and a sample is ready.
 * @note NULL or read failure returns false without changing output. No software
 *       magnetic calibration, axis remapping or filtering is applied here.
 */
bool sensors_read_mag(sensors_mag_t *mag_data);

/**
 * @brief Read one compensated barometer sample.
 * @param[out] baro_data Temperature in degrees Celsius and pressure in pascals.
 * @return true when the barometer is available and a sample was read.
 * @note NULL or read failure returns false without changing output. Uses factory
 *       compensation and the configured hardware filter; no height conversion.
 */
bool sensors_read_baro(sensors_baro_t *baro_data);

#endif /* DREAMFLY2_APPLICATION_SENSORS_H */
