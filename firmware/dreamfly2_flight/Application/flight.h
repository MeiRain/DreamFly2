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

#include "rcprotocol.h"
#include "attitude.h"
#include "height.h"
#include "motor.h"

typedef enum
{
    FLIGHT_MODE_STABILIZE = 0,
    FLIGHT_MODE_ALT_HOLD
} flight_mode_t;

typedef struct
{
    float roll_deg;
    float pitch_deg;
    float yaw_rate_dps;
    uint16_t throttle_pwm; /* STABILIZE: base compare request 0..1000, not applied motor output. */
    float climb_rate_mps; /* ALT_HOLD: upward positive; centered throttle requests zero. */
} flight_target_t;

/**
 * @brief Map validated RC channels to candidate targets for one control mode.
 * @param control RC channels in 1000..2000; centered attitude axes are 1500.
 * @param mode STABILIZE maps throttle to base PWM; ALT_HOLD maps it to climb rate.
 * @param[out] target Candidate targets; unchanged on failure. The unused throttle field is zero.
 * @return false for NULL pointers, invalid channels or an unsupported mode.
 * @note Roll/pitch limits are +/-20 deg, yaw rate +/-90 deg/s. Altitude-mode
 *       throttle is zero in 1450..1550 and maps continuously to +/-0.5 m/s outside.
 *       Attitude channels use the remote's existing deadzone. This pure mapping
 *       does not arm, authorize takeoff, lock height, run PID or drive motors.
 *       The caller must reject stale input and enforce flight-state conditions.
 */
bool flight_map_control(const rcprotocol_control_t *control, flight_mode_t mode,
                         flight_target_t *target);

/**
 * @brief Process arming gestures and advance the nonblocking flight state.
 * @param control Latest validated RC channels; must be valid when connected.
 * @param connected Whether the latest RC packet is fresh.
 * @param imu_ready Whether calibrated IMU attitude is fresh and usable.
 * @param now_ms Current monotonic time in milliseconds.
 * @return true when armed, including after the short motor indication ends.
 * @note Call periodically from the control task after flight_init(). Hold both
 *       sticks inward and down for 1500 ms to toggle. Release before repeating.
 *       Startup/recovery requires release first. Link/IMU loss clears arming.
 *       Arming runs M3, M4, M1, M2 (front-left, rear-left, rear-right,
 *       front-right), each at compare 100 for 200 ms.
 *       Motors are separated by 300 ms with all outputs zero. The sequence
 *       finishes after 1700 ms; only one motor is commanded at a time.
 *       After the indication, center all axes (1450..1550) for 200 ms, then
 *       raise throttle to at least 1600 to latch a takeoff request.
 *       After indication, auto-lock after 15 s without channel movement over
 *       20 steps or a key-state change. The activity baseline updates on movement;
 *       small cumulative movements can therefore count, but sequence changes do not.
 *       Auto-lock only applies while waiting for center/throttle, never after
 *       a takeoff request. Actual flight/landing logic is not implemented yet.
 *       This function only updates state; flight_process_control() writes the
 *       motor outputs afterward. With FLIGHT_RATE_TEST_ENABLED=1 the request
 *       enables limited propellers-off rate feedback; throttle <=1550 stops it
 *       and requires center confirmation before another >=1600 trigger.
 *       Call every 2 ms; stopping depends
 *       on this task continuing to run. Verify with propellers removed.
 *       This bench policy must be revisited before enabling actual flight.
 */
bool flight_process_remote(const rcprotocol_control_t *control,
                           bool connected, bool imu_ready, uint32_t now_ms);

/**
 * @brief Return the latched throttle takeoff request; does not start takeoff.
 * @note Read from the control task after flight_process_remote(). The request
 *       stays set until disarming or link/IMU loss. In bench mode throttle
 *       <=1550 also clears it. No height/takeoff controller is connected.
 */
bool flight_takeoff_requested(void);

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
    FLIGHT_STATUS_ARMED
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

typedef struct
{
    float acc_x_g;
    float acc_y_g;
    float acc_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} flight_imu_t;

typedef struct
{
    float mag_x_ut;
    float mag_y_ut;
    float mag_z_ut;
} flight_mag_t;

typedef struct
{
    float temp_c;
    float press_pa;
    float height_m; /* Barometric height relative to the startup ground reference; up positive. */
    bool height_valid; /* False until the ground reference has been collected. */
} flight_baro_t;

typedef struct
{
    bool imu;
    bool magnetometer;
    bool barometer;
    bool optical_flow;
    bool rangefinder;
} flight_sensor_status_t;

/* Latest measurements and estimates. A valid flag means received at least once;
 * timestamps must also be checked before using data for flight decisions.
 */
typedef struct
{
    flight_imu_t imu;
    flight_mag_t mag;
    flight_baro_t baro;
    attitude_t attitude_6axis;
    attitude_t attitude_9axis;
    height_t height;
    uint32_t attitude_last_ms;
    uint32_t baro_last_ms;
    uint32_t height_last_ms;
    bool imu_valid;
    bool mag_valid;
    bool baro_valid;
    bool attitude_6axis_valid;
    bool attitude_9axis_valid;
    bool height_valid;
    bool imu_pending_control; /* Consumed once by the control task. */
} flight_sensor_data_t;

/* Shared freshness rules and the current mapping preview mode. */
#define FLIGHT_BARO_TIMEOUT_MS 200U
#define FLIGHT_LINK_TIMEOUT_MS 300U
#define FLIGHT_ATTITUDE_TIMEOUT_MS 100U
#define FLIGHT_BATTERY_TIMEOUT_MS 500U
#define FLIGHT_HEIGHT_TIMEOUT_MS 100U
#define FLIGHT_MAPPING_MODE FLIGHT_MODE_STABILIZE
/* Propellers-off bench only: 1 permits limited zero-rate motor feedback after
 * the arming/center/throttle sequence; 0 restores calculation-only behavior.
 */
#define FLIGHT_RATE_TEST_ENABLED 1U

typedef struct
{
    float attitude_6axis_kp;
    float attitude_6axis_ki;
    float attitude_9axis_kp;
    float attitude_9axis_ki;
} flight_params_t;

/**
 * @brief Load validated Flash parameters, or select RAM defaults on an invalid record.
 * @return true for a valid saved record; false when defaults were selected.
 * @note Called once by flight_init() before periodic tasks are released. Later
 *       calls are rejected without changing parameters. Loading never erases or
 *       writes Flash. Selected values are also supplied to the attitude solver.
 */
bool flight_load_params(void);

/**
 * @brief Explicitly persist the current RAM parameters while disarmed.
 * @return false for invalid parameters, an unsafe state, program/storage overlap
 *         or a driver error. RAM parameters are retained on failure.
 * @note Call from a running task after flight_init(), never from an ISR or every
 *       loop. A scheduler lock prevents arming while saving; motors stay stopped.
 *       Flash operations pause normal scheduling/communication. Sector 7 must be
 *       reserved by limiting program Flash to 384 KB in the linker script.
 *       An identical stored record returns success without another erase.
 *       Single-sector saving is not power-loss atomic; invalid data falls back
 *       to defaults on next boot. No automatic write occurs at startup.
 */
bool flight_save_params(void);

typedef struct
{
    flight_params_t params;
    flight_sensor_status_t sensor_status;
    bool acc_calibrated;
    flight_sensor_data_t sensors;
    rcprotocol_control_t remote;
    uint32_t remote_received_count;
    uint32_t remote_last_ms;
    uint16_t battery_cv; /* 0.01 V for transport; 0 means unavailable. */
    uint32_t battery_last_ms;
    bool radio_ready;
    bool armed;
    bool takeoff_requested;
    flight_axis_t rate_output; /* PID correction in PWM compare counts, not angular rate. */
    bool rate_output_valid;
    uint32_t rate_last_ms; /* IMU/attitude sample time used for the last PID result. */
    motor_t motor_preview; /* Mixed candidate; only the gated bench path may apply it. */
    bool motor_preview_valid; /* Numeric candidate validity, not authorization to run. */
    motor_t motor_output; /* Last M1..M4 compare commands sent to the driver, not measured RPM. */
    flight_target_t target;
    bool target_valid;
} flight_data_t;

/** @brief Return whether startup attempts have completed; not overall module success. */
bool flight_is_initialized(void);

/**
 * @brief Run the 10-tick system step, including RGB while startup calibration runs.
 * @note Battery sampling (100 ms) and status LED toggle (500 ms) begin only after
 *       startup completes. RGB frames are limited to one every 20 ms.
 */
void flight_process_system(void);

/**
 * @brief Run one 2-tick RC/bench-control step, then write all four motor outputs.
 * @note With FLIGHT_RATE_TEST_ENABLED=1, the centered-then-raised throttle request
 *       permits propellers-off zero-rate feedback at base PWM=100, correction
 *       +/-20 per axis and per-motor maximum 160. Throttle <=1550 stops the test.
 *       Missing/invalid IMU data or rate results >=20 ms old disarm the bench;
 *       RC timeout also disarms. Between fresh samples, retain the last bounded
 *       result without rerunning PID. This does not implement self-leveling flight.
 *       With the switch at 0, only the arming indication may spin motors.
 *       This is the sole periodic motor writer; locked Flash saves also ensure zero.
 */
void flight_process_control(void);

/**
 * @brief Read sensors, solve attitude and estimate height once every 2 ticks.
 * @note At a 1 kHz kernel tick, IMU/attitude runs at 500 Hz, magnetic reads at
 *       100 Hz and pressure reads at 20 Hz. Startup calibration must finish first.
 *       Only the sensor task calls this function; published data uses scheduler locks.
 */
void flight_process_sensors(void);

/**
 * @brief Service UART2 parameters and report rate/motor previews every 20 ticks.
 * @note ANO responses precede periodic telemetry. Writes/reset require disarming,
 *       update gains at the sensor-task boundary, and ACK only after application.
 *       The debug task supplies the 20-tick period; USART1 uses a single format.
 */
void flight_process_debug(void);

/** @brief Service RX/ACK once, from the sole link task after IRQ or a 5-tick timeout. */
void flight_process_link(void);

/* Call each periodic function from its own task only. Except for the system
 * task, callers wait for completed flight_init(). Storage is private to flight.c.
 */

/**
 * @brief Load parameters and initialize flight state, motors, sensors, attitude and radio.
 * @note Call once from a running task after CubeMX peripheral initialization.
 *       Results are retained in the application snapshot; completion does not
 *       mean every module succeeded. Other tasks must wait for return. Motors start
 *       at zero. Gyro calibration blocks until still; keep the aircraft level.
 *       Attitude starts from measured acceleration, or level if that read fails.
 *       Height seeding still waits for the measured barometric ground reference.
 *       Blocking calibration may yield to RTOS; do not hold the scheduler lock.
 */
void flight_init(void);

/** @brief Toggle the on-board flight status LED. */
void flight_led_toggle(void);

/** @brief Select an RGB indication state; does not change actual arming or flight mode. */
void flight_set_status(flight_status_t status);

/**
 * @brief Set the sensor fault shown by the RGB LEDs.
 * @note Pass FLIGHT_SENSOR_NONE to clear the fault indication. Repeating the
 *       same value preserves the blink phase. System-task RGB processing selects
 *       the startup fault automatically.
 */
void flight_set_sensor_error(flight_sensor_t sensor);

/**
 * @brief Show startup, connection or sensor-fault RGB indication.
 * @note Called only by the system task. May run before flight_init() returns;
 *       it waits for the RGB driver to be ready. Frames are limited to 20 ms.
 *       Initialization blinks yellow (500 ms on/off); waiting for RC is steady
 *       yellow; fresh RC while locked uses the shared-color gradient. Armed
 *       indication is front RGB1/2 solid red and rear RGB3/4 green blinking at
 *       1000 ms on/off, including the motor indication and takeoff waiting phases.
 *       Startup sensor faults
 *       take priority: IMU/level calibration red, magnetometer orange, barometer
 *       blue (500 ms on/off). FLOW/RANGE are not required yet. This reports
 *       startup results, not continuous sensor health. Faults and stale RC take
 *       priority over the armed indication.
 */
void flight_update_rgb(void);

/**
 * @brief Read and update the current battery voltage.
 * @return Latest battery voltage in volts.
 */
float flight_update_battery(void);

#endif /* DREAMFLY2_APPLICATION_FLIGHT_H */
