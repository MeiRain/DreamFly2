/**
 * @file flight.c
 * @brief Flight-state management implementation.
 * @author 梅雨
 * @date 2026-10-08
 */

#include "flight.h"

#include <stdint.h>
#include <stddef.h>

#include "led.h"
#include "battery.h"
#include "ws2812b.h"
#include "pid.h"
#include "motor.h"
#include "attitude.h"
#include "filter.h"
#include "ak09916.h"
#include "bmp280.h"
#include "icm20948.h"
#include "si24r1.h"
#include "debug.h"
#include "flash.h"
#include "anoprotocol.h"

#include <string.h>
#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"

#include <math.h>

#define FLIGHT_RGB_BRIGHTNESS 64U
#define FLIGHT_ERROR_FLASH_COUNT 25U /* 25 frames x 20 ms = 500 ms per on/off phase. */
#define FLIGHT_RGB_PERIOD_MS 20U
#define FLIGHT_INIT_BLINK_MS 500U
#define FLIGHT_ARMED_BLINK_MS 1000U
#define FLIGHT_IDLE_LOCK_MS 15000U
#define FLIGHT_RC_ACTIVITY_DELTA 20U /* Channel steps, not ADC counts. */
#define FLIGHT_STICK_LOW 1100U
#define FLIGHT_STICK_HIGH 1900U
#define FLIGHT_ARM_HOLD_MS 1500U
/* Bench starting values: ARR=999 gives 10% duty; verify with propellers removed. */
#define FLIGHT_ARM_SPIN_PWM 100
#define FLIGHT_ARM_SPIN_MS 200U
#define FLIGHT_ARM_SPIN_GAP_MS 300U
#define FLIGHT_STICK_CENTER_MIN 1450U
#define FLIGHT_STICK_CENTER_MAX 1550U
#define FLIGHT_CENTER_HOLD_MS 200U
#define FLIGHT_TAKEOFF_THROTTLE 1600U
#define FLIGHT_MAX_TILT_DEG 20.0f
#define FLIGHT_MAX_YAW_RATE_DPS 90.0f
#define FLIGHT_MAX_CLIMB_RATE_MPS 0.5f

/* Restricted propellers-off motor feedback; unrelated to the mapped flight throttle. */
#define FLIGHT_RATE_TEST_BASE_PWM 100U
#define FLIGHT_RATE_TEST_OUTPUT_LIMIT 20.0f
#define FLIGHT_RATE_TEST_MAX_PWM 160
#define FLIGHT_RATE_TEST_TIMEOUT_MS 20U

_Static_assert(FLIGHT_RATE_TEST_MAX_PWM <= MOTOR_PWM_MAX,
               "Bench limit must not exceed the motor driver limit");

#define FLIGHT_LED_PERIOD_CYCLES 50U
#define FLIGHT_BATTERY_PERIOD_CYCLES 10U
#define FLIGHT_ATTITUDE_PERIOD_S 0.002f

#define IMU_CALIBRATION_SAMPLES 256U
#define IMU_CALIBRATION_INTERVAL_TICKS 2U
#define IMU_STILL_GYRO_DELTA_DPS 0.31f
#define IMU_STILL_GYRO_LIMIT_DPS 6.10f
#define IMU_ACCEL_CALIBRATION_TIMEOUT_SAMPLES 2500U
#define IMU_LEVEL_ACCEL_XY_LIMIT_G 0.15f
#define IMU_LEVEL_ACCEL_Z_MIN_G 0.85f
#define IMU_LEVEL_ACCEL_Z_MAX_G 1.15f

#define IMU_ACCEL_FILTER_PROCESS_NOISE 0.001f
#define IMU_ACCEL_FILTER_MEASUREMENT_NOISE 0.543f
#define IMU_ACCEL_FILTER_INITIAL_COVARIANCE 0.02f
#define IMU_GYRO_FILTER_FACTOR 0.85f

#define BARO_REFERENCE_SAMPLES 32U
#define BARO_SEA_LEVEL_PRESSURE_PA 101325.0f
#define BARO_ALTITUDE_SCALE_M 44330.0f
#define BARO_ALTITUDE_EXPONENT (1.0f / 5.255f)

#define FLIGHT_MAG_PERIOD_CYCLES 5U
#define FLIGHT_BARO_PERIOD_CYCLES 25U
#define FLIGHT_DEG_TO_RAD 0.01745329252f
#define FLIGHT_GRAVITY_MPS2 9.80665f

#define FLIGHT_PARAMS_MAGIC 0x44464C59U
#define FLIGHT_PARAMS_VERSION 1U
/* Mirrors the sector-7 address in Core/Src/flash.c. */
#define FLIGHT_PARAMS_ADDRESS 0x08060000U
#define FLIGHT_PARAMS_MAX_KP 20.0f
#define FLIGHT_PARAMS_MAX_KI 5.0f

/* Parameter format v1: 12-byte header, four float gains, then CRC32. */
typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t size;
    flight_params_t params;
    uint32_t crc;
} flight_params_record_t;

_Static_assert(sizeof(flight_params_record_t) == 32U,
               "Update the Flash parameter format/version after a layout change");

/* Defaults used only when a saved record is absent or invalid. */
static const flight_params_t s_default_params = {
    .attitude_6axis_kp = 0.8f,
    .attitude_6axis_ki = 0.0003f,
    .attitude_9axis_kp = 0.2f,
    .attitude_9axis_ki = 0.1f
};

/* Provided by STM32F411xx_FLASH.ld, after code and initialized-data images. */
extern const uint8_t __data_source_end;

/* ANO parameter IDs follow this table order; float type is protocol value 8. */
typedef struct
{
    uint16_t offset;
    int32_t max;
    const char *name;
    const char *info;
} flight_param_info_t;

static const flight_param_info_t s_param_info[] = {
    {offsetof(flight_params_t, attitude_6axis_kp), 20, "ATT6_KP", "Six-axis proportional correction"},
    {offsetof(flight_params_t, attitude_6axis_ki), 5, "ATT6_KI", "Six-axis integral correction"},
    {offsetof(flight_params_t, attitude_9axis_kp), 20, "ATT9_KP", "Nine-axis proportional correction"},
    {offsetof(flight_params_t, attitude_9axis_ki), 5, "ATT9_KI", "Nine-axis integral correction"}
};
#define FLIGHT_PARAM_COUNT (sizeof(s_param_info) / sizeof(s_param_info[0]))
#define FLIGHT_PARAM_RX_TIMEOUT_MS 200U
#define FLIGHT_PARAM_APPLY_TIMEOUT_MS 200U

typedef enum
{
    PARAM_CHANGE_NONE = 0,
    PARAM_CHANGE_WAITING,
    PARAM_CHANGE_APPLIED,
    PARAM_CHANGE_REJECTED
} flight_param_change_state_t;

/* Debug task posts one change; sensor task applies it between solver calls. */
static struct
{
    flight_param_change_state_t state;
    flight_params_t params;
} s_param_change;

/* Single debug-task transaction: one reply gets priority over periodic telemetry. */
static struct
{
    uint8_t rx[ANOPROTOCOL_MAX_FRAME_SIZE];
    uint8_t tx[ANOPROTOCOL_MAX_FRAME_SIZE];
    uint16_t rx_size;
    uint16_t tx_size;
    uint32_t last_rx_ms;
    uint32_t change_start_ms;
} s_param_io;



/* Published measurements/state are copied under scheduler locks. Only the
 * sensor task writes s_data.sensors, except the control task consumes its pending flag.
 */
static flight_data_t s_data;
/* RGB may run during calibration, but only after its driver is initialized. */
typedef enum
{
    FLIGHT_INIT_PENDING = 0,
    FLIGHT_INIT_RUNNING,
    FLIGHT_INIT_COMPLETE
} flight_init_stage_t;

static flight_init_stage_t s_init_stage;

typedef enum
{
    FLIGHT_LOCKED = 0,
    FLIGHT_ARM_INDICATING,
    FLIGHT_WAIT_CENTER,
    FLIGHT_CONFIRM_CENTER,
    FLIGHT_WAIT_THROTTLE,
    FLIGHT_TAKEOFF_REQUESTED
} flight_state_t;

typedef enum
{
    GESTURE_WAIT_RELEASE = 0, /* Release first after startup, a trigger or a fault. */
    GESTURE_IDLE,             /* Ready to recognize a new inward-stick hold. */
    GESTURE_HOLDING           /* Timing the current inward-stick hold. */
} flight_gesture_t;

/* Physical front RGB1/2 use chain indices 2/3; rear RGB3/4 use indices 0/1. */
#define FLIGHT_FRONT_LED_1 2U
#define FLIGHT_FRONT_LED_2 3U
#define FLIGHT_REAR_LED_1 0U
#define FLIGHT_REAR_LED_2 1U

/* Related state stays private to this file; task code uses the public functions. */
typedef struct
{
    flight_state_t state; /* One state describes arming and takeoff preparation. */
    flight_gesture_t gesture;
    uint32_t gesture_start_ms; /* Inward-stick hold start tick. */
    uint32_t state_start_ms; /* Shared start tick for indication or center confirmation. */
    uint32_t idle_start_ms; /* Most recent pre-takeoff operator activity. */
    rcprotocol_control_t idle_control; /* Activity baseline; sequence is ignored. */
} flight_remote_state_t;

typedef struct
{
    flight_status_t status; /* RGB indication only, not actual flight authorization. */
    flight_sensor_t sensor_error;
    uint8_t flash_count; /* Full blink cycle: first half on, second half off. */
} flight_rgb_state_t;

typedef struct
{
    pid_t roll;
    pid_t pitch;
    pid_t yaw;
} flight_pid_t;

static flight_remote_state_t s_remote = {
    .state = FLIGHT_LOCKED,
    .gesture = GESTURE_WAIT_RELEASE
};
static flight_rgb_state_t s_rgb = {
    .status = FLIGHT_STATUS_WAIT_PAIRING,
    .sensor_error = FLIGHT_SENSOR_NONE
};
static flight_pid_t s_rate_pid;

/* The 25-call cycle also schedules the magnetometer every five calls. */
static uint8_t s_sensor_count;
static uint32_t s_height_correction_ms; /* Last new barometer correction tick. */
static float s_accel_bias[3]; /* XYZ offsets in g; horizontal calibration preserves +1 g on Z. */
static float s_gyro_bias[3]; /* XYZ zero-rate offsets in degrees/s, stored in RAM only. */
static filter_kalman_1d_t s_accel_filter[3];
static filter_low_pass_t s_gyro_filter[3];
static float s_baro_alt_sum_m; /* Startup altitude sum used only while collecting the reference. */
static float s_baro_alt_offset_m; /* Mean ground altitude; may be zero or negative. */
static uint8_t s_baro_reference_count;

bool flight_map_control(const rcprotocol_control_t *control, flight_mode_t mode,
                         flight_target_t *target)
{
    if (control == NULL || target == NULL
        || (mode != FLIGHT_MODE_STABILIZE && mode != FLIGHT_MODE_ALT_HOLD)
        || control->roll < 1000U || control->roll > 2000U
        || control->pitch < 1000U || control->pitch > 2000U
        || control->yaw < 1000U || control->yaw > 2000U
        || control->throttle < 1000U || control->throttle > 2000U)
    {
        return false;
    }

    flight_target_t mapped = {0};
    mapped.roll_deg = ((float)control->roll - 1500.0f) / 500.0f * FLIGHT_MAX_TILT_DEG;
    mapped.pitch_deg = ((float)control->pitch - 1500.0f) / 500.0f * FLIGHT_MAX_TILT_DEG;
    mapped.yaw_rate_dps = ((float)control->yaw - 1500.0f) / 500.0f * FLIGHT_MAX_YAW_RATE_DPS;

    if (mode == FLIGHT_MODE_STABILIZE)
    {
        mapped.throttle_pwm = control->throttle - 1000U;
    }
    else if (control->throttle > FLIGHT_STICK_CENTER_MAX)
    {
        mapped.climb_rate_mps = ((float)control->throttle - FLIGHT_STICK_CENTER_MAX)
            / (2000.0f - FLIGHT_STICK_CENTER_MAX) * FLIGHT_MAX_CLIMB_RATE_MPS;
    }
    else if (control->throttle < FLIGHT_STICK_CENTER_MIN)
    {
        mapped.climb_rate_mps = ((float)control->throttle - FLIGHT_STICK_CENTER_MIN)
            / (FLIGHT_STICK_CENTER_MIN - 1000.0f) * FLIGHT_MAX_CLIMB_RATE_MPS;
    }

    *target = mapped;
    return true;
}

/** @brief Test one centered RC channel without assuming an exact ADC midpoint. */
static bool flight_stick_centered(uint16_t channel)
{
    return channel >= FLIGHT_STICK_CENTER_MIN && channel <= FLIGHT_STICK_CENTER_MAX;
}

/** @brief Ignore small RC jitter while detecting movement relative to the activity baseline. */
static bool flight_channel_moved(uint16_t value, uint16_t reference)
{
    return value > reference + FLIGHT_RC_ACTIVITY_DELTA
        || reference > value + FLIGHT_RC_ACTIVITY_DELTA;
}

/**
 * @brief Mix a candidate into bounded M1..M4 compare counts without driving motors.
 * @note Draft body frame: X forward, Y left, Z up. R is right-wing-down,
 *       P is nose-down, Y is counterclockwise from above. M1 rear-right and
 *       M3 front-left turn CCW; M2 front-right and M4 rear-left turn CW.
 *       Handheld axis signs were checked; real motor reaction remains unverified.
 *       All corrections use PWM counts; base throttle is 0..1000. Bench output
 *       clamps to 0..160; calculation-only mode retains the driver limit 0..900.
 * @return false for invalid input or a nonfinite sum; output stays unchanged.
 *         Zero base throttle produces four zeros. Independent clipping may
 *         reduce torque authority; this is not a flight-ready allocator.
 */
static bool flight_mix_motors(uint16_t throttle_pwm, const flight_axis_t *correction,
                              motor_t *motor)
{
    if (correction == NULL || motor == NULL || throttle_pwm > 1000U
        || !isfinite(correction->roll) || !isfinite(correction->pitch)
        || !isfinite(correction->yaw))
    {
        return false;
    }

    float maximum = FLIGHT_RATE_TEST_ENABLED ? FLIGHT_RATE_TEST_MAX_PWM : MOTOR_PWM_MAX;
    float t = (float)throttle_pwm;
    float r = correction->roll;
    float p = correction->pitch;
    float y = correction->yaw;
    float pwm[4] = {t - r + p - y, t - r - p + y,
                    t + r - p - y, t + r + p + y};
    for (uint8_t i = 0U; i < 4U; i++)
    {
        if (!isfinite(pwm[i]))
        {
            return false;
        }
        if (throttle_pwm == 0U || pwm[i] < 0.0f)
        {
            pwm[i] = 0.0f;
        }
        else if (pwm[i] > maximum)
        {
            pwm[i] = maximum;
        }
    }

    *motor = (motor_t){
        .pwm1 = (int16_t)(pwm[0] + 0.5f),
        .pwm2 = (int16_t)(pwm[1] + 0.5f),
        .pwm3 = (int16_t)(pwm[2] + 0.5f),
        .pwm4 = (int16_t)(pwm[3] + 0.5f)
    };
    return true;
}

/**
 * @brief Select all four motor outputs from the flight state and write them once.
 * @note Called by the control task, or while locked before a Flash save.
 *       The arming indication and gated propellers-off rate test are the only
 *       nonzero sources. candidate must already be mixed and bounded, or NULL.
 */
static void flight_write_motors(uint32_t now_ms, const motor_t *candidate)
{
    /* Top view, nose forward: front-left, rear-left, rear-right, front-right.
     * Motor IDs remain tied to the PCB outputs; only the indication is reordered.
     */
    static const uint8_t motor_order[4] = {3U, 4U, 1U, 2U};
    uint8_t motor_number = 0U;
    if (s_remote.state == FLIGHT_ARM_INDICATING)
    {
        uint32_t elapsed_ms = (uint32_t)(now_ms - s_remote.state_start_ms);
        uint32_t slot_ms = FLIGHT_ARM_SPIN_MS + FLIGHT_ARM_SPIN_GAP_MS;
        uint32_t slot = elapsed_ms / slot_ms;
        if (slot < 4U && elapsed_ms % slot_ms < FLIGHT_ARM_SPIN_MS)
        {
            motor_number = motor_order[slot];
        }
    }

    motor_t output = {
        .pwm1 = motor_number == 1U ? FLIGHT_ARM_SPIN_PWM : 0,
        .pwm2 = motor_number == 2U ? FLIGHT_ARM_SPIN_PWM : 0,
        .pwm3 = motor_number == 3U ? FLIGHT_ARM_SPIN_PWM : 0,
        .pwm4 = motor_number == 4U ? FLIGHT_ARM_SPIN_PWM : 0
    };
    if (FLIGHT_RATE_TEST_ENABLED && s_remote.state == FLIGHT_TAKEOFF_REQUESTED
        && candidate != NULL)
    {
        output = *candidate;
    }
    motor_control(&output);

    /* Preserve a caller's scheduler lock during an explicit locked Flash save. */
    int32_t lock = osKernelLock();
    s_data.motor_output = output;
    (void)osKernelRestoreLock(lock);
}

/** @brief Buffer one RGB color for all LEDs and request a DMA transfer. */
static void flight_rgb_all(uint8_t red, uint8_t green, uint8_t blue)
{
    ws2812b_set_all(red, green, blue);
    ws2812b_show();
}

/** @brief Show front red continuously and rear green with 1000 ms on/off phases. */
static void flight_show_armed_rgb(uint32_t now_ms)
{
    uint8_t green = (now_ms / FLIGHT_ARMED_BLINK_MS) % 2U == 0U
        ? FLIGHT_RGB_BRIGHTNESS : 0U;
    ws2812b_set_pixel(FLIGHT_FRONT_LED_1, FLIGHT_RGB_BRIGHTNESS, 0U, 0U);
    ws2812b_set_pixel(FLIGHT_FRONT_LED_2, FLIGHT_RGB_BRIGHTNESS, 0U, 0U);
    ws2812b_set_pixel(FLIGHT_REAR_LED_1, 0U, green, 0U);
    ws2812b_set_pixel(FLIGHT_REAR_LED_2, 0U, green, 0U);
    ws2812b_show();
}

/** @brief Show the active fault color; toggle visibility after a fixed number of calls. */
static void flight_show_sensor_error(void)
{
    if (s_rgb.flash_count >= FLIGHT_ERROR_FLASH_COUNT)
    {
        flight_rgb_all(0U, 0U, 0U);
    }
    else
    {
        switch (s_rgb.sensor_error)
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

    s_rgb.flash_count++;
    if (s_rgb.flash_count >= 2U * FLIGHT_ERROR_FLASH_COUNT)
    {
        s_rgb.flash_count = 0U;
    }
}

/** @brief Test per-axis gyro magnitude and inter-sample change; inputs must be valid. */
static bool flight_imu_is_still(const icm20948_t *sample,
                         const float last_gyro[3])
{
    float gyro[3] = {
        sample->gyro_x_dps,
        sample->gyro_y_dps,
        sample->gyro_z_dps
    };

    for (uint8_t axis = 0U; axis < 3U; axis++)
    {
        float delta = gyro[axis] - last_gyro[axis];

        if ((delta < -IMU_STILL_GYRO_DELTA_DPS)
            || (delta > IMU_STILL_GYRO_DELTA_DPS)
            || (gyro[axis] < -IMU_STILL_GYRO_LIMIT_DPS)
            || (gyro[axis] > IMU_STILL_GYRO_LIMIT_DPS))
        {
            return false;
        }
    }

    return true;
}

/** @brief Seed each axis filter with a bias-corrected sample to avoid a zero startup transient. */
static void flight_imu_init_filters(const icm20948_t *sample)
{
    float accel[3] = {
        sample->acc_x_g - s_accel_bias[0],
        sample->acc_y_g - s_accel_bias[1],
        sample->acc_z_g - s_accel_bias[2]
    };
    float gyro[3] = {
        sample->gyro_x_dps - s_gyro_bias[0],
        sample->gyro_y_dps - s_gyro_bias[1],
        sample->gyro_z_dps - s_gyro_bias[2]
    };

    for (uint8_t axis = 0U; axis < 3U; axis++)
    {
        filter_kalman_1d_init(&s_accel_filter[axis],
                              IMU_ACCEL_FILTER_PROCESS_NOISE,
                              IMU_ACCEL_FILTER_MEASUREMENT_NOISE,
                              IMU_ACCEL_FILTER_INITIAL_COVARIANCE,
                              accel[axis]);
        filter_low_pass_init(&s_gyro_filter[axis],
                             IMU_GYRO_FILTER_FACTOR,
                             gyro[axis]);
    }
}

/**
 * @brief Average consecutive stationary gyro samples and initialize the filters.
 * @return false on an IMU read failure.
 * @note Uses RTOS delays; movement restarts accumulation with no overall timeout.
 */
static bool flight_calibrate_gyro(void)
{
    icm20948_t sample;
    float gyro_sum[3] = {0.0f};
    float last_gyro[3] = {0.0f};
    uint16_t sample_count = 0U;
    bool last_gyro_valid = false;

    while (sample_count < IMU_CALIBRATION_SAMPLES)
    {
        if (!icm20948_read(&sample))
        {
            return false;
        }

        if (last_gyro_valid && flight_imu_is_still(&sample, last_gyro))
        {
            gyro_sum[0] += sample.gyro_x_dps;
            gyro_sum[1] += sample.gyro_y_dps;
            gyro_sum[2] += sample.gyro_z_dps;
            ++sample_count;
        }
        else
        {
            gyro_sum[0] = 0.0f;
            gyro_sum[1] = 0.0f;
            gyro_sum[2] = 0.0f;
            sample_count = 0U;
        }

        last_gyro[0] = sample.gyro_x_dps;
        last_gyro[1] = sample.gyro_y_dps;
        last_gyro[2] = sample.gyro_z_dps;
        last_gyro_valid = true;

        osDelay(IMU_CALIBRATION_INTERVAL_TICKS);
    }

    for (uint8_t axis = 0U; axis < 3U; axis++)
    {
        s_gyro_bias[axis] = gyro_sum[axis]
                            / (float)IMU_CALIBRATION_SAMPLES;
    }

    flight_imu_init_filters(&sample);
    return true;
}

/** @brief Initialize sensors and gyro bias; optional sensor failures remain in status. */
static bool flight_init_sensors(void)
{
    s_sensor_count = 0U;
    s_height_correction_ms = 0U;
    s_data.sensor_status = (flight_sensor_status_t){0};
    s_baro_alt_sum_m = 0.0f;
    s_baro_alt_offset_m = 0.0f;
    s_baro_reference_count = 0U;

    if (!icm20948_init() || !flight_calibrate_gyro())
    {
        return false;
    }

    s_data.sensor_status.imu = true;
    s_data.sensor_status.magnetometer = ak09916_init();
    s_data.sensor_status.barometer = bmp280_init();

    return true;
}

/** @brief Average consecutive level/still samples; keep +1 g on Z and fail at the attempt limit. */
static bool flight_calibrate_accel(void)
{
    icm20948_t sample;
    float accel_sum[3] = {0.0f};
    float last_gyro[3] = {0.0f};
    uint16_t sample_count = 0U;
    uint16_t attempt_count = 0U;
    bool last_gyro_valid = false;

    if (!s_data.sensor_status.imu)
    {
        return false;
    }

    while ((sample_count < IMU_CALIBRATION_SAMPLES)
           && (attempt_count < IMU_ACCEL_CALIBRATION_TIMEOUT_SAMPLES))
    {
        if (!icm20948_read(&sample))
        {
            return false;
        }
        ++attempt_count;

        if (last_gyro_valid
            && flight_imu_is_still(&sample, last_gyro)
            && (sample.acc_x_g >= -IMU_LEVEL_ACCEL_XY_LIMIT_G)
            && (sample.acc_x_g <= IMU_LEVEL_ACCEL_XY_LIMIT_G)
            && (sample.acc_y_g >= -IMU_LEVEL_ACCEL_XY_LIMIT_G)
            && (sample.acc_y_g <= IMU_LEVEL_ACCEL_XY_LIMIT_G)
            && (sample.acc_z_g >= IMU_LEVEL_ACCEL_Z_MIN_G)
            && (sample.acc_z_g <= IMU_LEVEL_ACCEL_Z_MAX_G))
        {
            accel_sum[0] += sample.acc_x_g;
            accel_sum[1] += sample.acc_y_g;
            accel_sum[2] += sample.acc_z_g;
            ++sample_count;
        }
        else
        {
            accel_sum[0] = 0.0f;
            accel_sum[1] = 0.0f;
            accel_sum[2] = 0.0f;
            sample_count = 0U;
        }

        last_gyro[0] = sample.gyro_x_dps;
        last_gyro[1] = sample.gyro_y_dps;
        last_gyro[2] = sample.gyro_z_dps;
        last_gyro_valid = true;

        osDelay(IMU_CALIBRATION_INTERVAL_TICKS);
    }

    if (sample_count < IMU_CALIBRATION_SAMPLES)
    {
        return false;
    }

    s_accel_bias[0] = accel_sum[0] / (float)IMU_CALIBRATION_SAMPLES;
    s_accel_bias[1] = accel_sum[1] / (float)IMU_CALIBRATION_SAMPLES;
    s_accel_bias[2] = accel_sum[2] / (float)IMU_CALIBRATION_SAMPLES - 1.0f;

    flight_imu_init_filters(&sample);
    return true;
}

/** @brief Read physical IMU values, subtract biases and filter each axis. */
static bool flight_read_imu(flight_imu_t *imu_data)
{
    icm20948_t sample;
    float accel[3];
    float gyro[3];

    if ((imu_data == NULL) || !s_data.sensor_status.imu || !icm20948_read(&sample))
    {
        return false;
    }

    accel[0] = sample.acc_x_g - s_accel_bias[0];
    accel[1] = sample.acc_y_g - s_accel_bias[1];
    accel[2] = sample.acc_z_g - s_accel_bias[2];
    gyro[0] = sample.gyro_x_dps - s_gyro_bias[0];
    gyro[1] = sample.gyro_y_dps - s_gyro_bias[1];
    gyro[2] = sample.gyro_z_dps - s_gyro_bias[2];

    for (uint8_t axis = 0U; axis < 3U; axis++)
    {
        accel[axis] = filter_kalman_1d(&s_accel_filter[axis], accel[axis]);
        gyro[axis] = filter_low_pass(&s_gyro_filter[axis], gyro[axis]);
    }

    imu_data->acc_x_g = accel[0];
    imu_data->acc_y_g = accel[1];
    imu_data->acc_z_g = accel[2];
    imu_data->gyro_x_dps = gyro[0];
    imu_data->gyro_y_dps = gyro[1];
    imu_data->gyro_z_dps = gyro[2];

    return true;
}

/** @brief Read magnetic field in microtesla without software magnetic calibration. */
static bool flight_read_mag(flight_mag_t *mag_data)
{
    ak09916_t sample;

    if ((mag_data == NULL)
        || !s_data.sensor_status.magnetometer
        || !ak09916_read(&sample))
    {
        return false;
    }

    mag_data->mag_x_ut = sample.mag_x_ut;
    mag_data->mag_y_ut = sample.mag_y_ut;
    mag_data->mag_z_ut = sample.mag_z_ut;

    return true;
}

/** @brief Convert compensated pressure to height relative to the first 32 valid samples. */
static bool flight_read_baro(flight_baro_t *baro_data)
{
    bmp280_t sample;

    if ((baro_data == NULL)
        || !s_data.sensor_status.barometer
        || !bmp280_read(&sample))
    {
        return false;
    }

    if (!isfinite(sample.press_pa) || sample.press_pa <= 0.0f
        || !isfinite(sample.temp_c))
    {
        return false;
    }

    float altitude_m = BARO_ALTITUDE_SCALE_M
        * (1.0f - powf(sample.press_pa / BARO_SEA_LEVEL_PRESSURE_PA,
                       BARO_ALTITUDE_EXPONENT));
    if (!isfinite(altitude_m))
    {
        return false;
    }

    if (s_baro_reference_count < BARO_REFERENCE_SAMPLES)
    {
        s_baro_alt_sum_m += altitude_m;
        s_baro_reference_count++;
        if (s_baro_reference_count >= BARO_REFERENCE_SAMPLES)
        {
            s_baro_alt_offset_m = s_baro_alt_sum_m / (float)BARO_REFERENCE_SAMPLES;
        }
    }

    baro_data->temp_c = sample.temp_c;
    baro_data->press_pa = sample.press_pa;
    baro_data->height_m = s_baro_reference_count >= BARO_REFERENCE_SAMPLES
        ? altitude_m - s_baro_alt_offset_m : 0.0f;
    baro_data->height_valid = s_baro_reference_count >= BARO_REFERENCE_SAMPLES;

    return true;
}

/** @brief CRC-32/ISO-HDLC over the record bytes preceding the CRC field. */
static uint32_t flight_params_crc(const flight_params_record_t *record)
{
    const uint8_t *bytes = (const uint8_t *)record;
    uint32_t crc = 0xFFFFFFFFU;
    for (uint32_t i = 0U; i < offsetof(flight_params_record_t, crc); i++)
    {
        crc ^= bytes[i];
        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            crc = (crc >> 1U) ^ ((crc & 1U) != 0U ? 0xEDB88320U : 0U);
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

/** @brief Reject nonfinite/out-of-range payloads before applying or saving gains. */
static bool flight_params_valid(const flight_params_t *params)
{
    return isfinite(params->attitude_6axis_kp) && isfinite(params->attitude_6axis_ki)
        && isfinite(params->attitude_9axis_kp) && isfinite(params->attitude_9axis_ki)
        && params->attitude_6axis_kp >= 0.0f && params->attitude_6axis_kp <= FLIGHT_PARAMS_MAX_KP
        && params->attitude_9axis_kp >= 0.0f && params->attitude_9axis_kp <= FLIGHT_PARAMS_MAX_KP
        && params->attitude_6axis_ki >= 0.0f && params->attitude_6axis_ki <= FLIGHT_PARAMS_MAX_KI
        && params->attitude_9axis_ki >= 0.0f && params->attitude_9axis_ki <= FLIGHT_PARAMS_MAX_KI;
}

bool flight_load_params(void)
{
    int32_t lock = osKernelLock();
    if (lock < 0)
    {
        return false;
    }
    if (s_init_stage == FLIGHT_INIT_COMPLETE)
    {
        (void)osKernelRestoreLock(lock);
        return false;
    }

    flight_params_record_t record = {0};
    bool valid = flash_read(&record, sizeof(record))
        && record.magic == FLIGHT_PARAMS_MAGIC
        && record.version == FLIGHT_PARAMS_VERSION
        && record.size == sizeof(record.params)
        && record.crc == flight_params_crc(&record)
        && flight_params_valid(&record.params);
    s_data.params = valid ? record.params : s_default_params;
    (void)attitude_set_gains(s_data.params.attitude_6axis_kp, s_data.params.attitude_6axis_ki,
                             s_data.params.attitude_9axis_kp, s_data.params.attitude_9axis_ki);
    (void)osKernelRestoreLock(lock);
    return valid;
}

bool flight_save_params(void)
{
    int32_t lock = osKernelLock();
    if (lock < 0)
    {
        return false;
    }

    /* Check actual load-image end as a backup to the linker reservation.
     * Keep the scheduler locked through writing so no task can arm midway.
     */
    if (s_init_stage != FLIGHT_INIT_COMPLETE || s_remote.state != FLIGHT_LOCKED
        || s_param_change.state == PARAM_CHANGE_WAITING
        || (uintptr_t)&__data_source_end > FLIGHT_PARAMS_ADDRESS
        || !flight_params_valid(&s_data.params))
    {
        (void)osKernelRestoreLock(lock);
        return false;
    }

    flight_write_motors(HAL_GetTick(), NULL);
    flight_params_record_t record = {
        .magic = FLIGHT_PARAMS_MAGIC,
        .version = FLIGHT_PARAMS_VERSION,
        .size = sizeof(s_data.params),
        .params = s_data.params
    };
    record.crc = flight_params_crc(&record);
    flight_params_record_t saved;
    bool success = flash_read(&saved, sizeof(saved))
        && memcmp(&saved, &record, sizeof(record)) == 0;
    if (!success)
    {
        success = flash_write(&record, sizeof(record));
    }
    (void)osKernelRestoreLock(lock);
    return success;
}

/** @brief Queue a validated RAM change while locked; caller awaits the sensor-task result. */
static bool flight_request_params(const flight_params_t *params)
{
    bool accepted = false;
    (void)osKernelLock();
    if (s_init_stage == FLIGHT_INIT_COMPLETE && s_remote.state == FLIGHT_LOCKED
        && s_param_change.state == PARAM_CHANGE_NONE && flight_params_valid(params))
    {
        accepted = true;
        if (memcmp(params, &s_data.params, sizeof(*params)) != 0)
        {
            s_param_change.params = *params;
            s_param_change.state = PARAM_CHANGE_WAITING;
            s_param_io.change_start_ms = HAL_GetTick();
        }
    }
    (void)osKernelUnlock();
    return accepted;
}

/** @brief Apply queued gains only at the sensor-task boundary, preserving orientation. */
static void flight_apply_params(void)
{
    (void)osKernelLock();
    if (s_param_change.state == PARAM_CHANGE_WAITING)
    {
        const flight_params_t *params = &s_param_change.params;
        if (s_remote.state == FLIGHT_LOCKED
            && attitude_set_gains(params->attitude_6axis_kp, params->attitude_6axis_ki,
                                   params->attitude_9axis_kp, params->attitude_9axis_ki))
        {
            s_data.params = *params;
            s_param_change.state = PARAM_CHANGE_APPLIED;
        }
        else
        {
            s_param_change.state = PARAM_CHANGE_REJECTED;
        }
    }
    (void)osKernelUnlock();
}

/** @brief Report an operation failure as an ANO text log, without a success ACK. */
static uint16_t flight_param_error(uint8_t destination, const char *message)
{
    uint8_t data[80] = {0};
    uint16_t length = (uint16_t)strlen(message);
    if (length > sizeof(data) - 1U)
    {
        length = sizeof(data) - 1U;
    }
    data[0] = 1U;
    memcpy(&data[1], message, length);
    return anoprotocol_pack_frame(ANOPROTOCOL_SOURCE_ADDRESS, destination, 0xA0U,
                                  data, length + 1U, s_param_io.tx, sizeof(s_param_io.tx));
}

/** @brief Prepare one parameter response; successful writes wait for the sensor task before ACK. */
static uint16_t flight_handle_parameter(const anoprotocol_frame_t *frame)
{
    uint8_t data[ANOPROTOCOL_MAX_FRAME_SIZE - 8U] = {0};
    uint16_t length = 0U;
    uint8_t id = 0x00U;

    if (frame->id == ANOPROTOCOL_PARAM_VALUE)
    {
        if (frame->length != 6U)
        {
            return flight_param_error(frame->source, "PARAM: invalid value frame");
        }
        uint16_t index = (uint16_t)frame->data[0] | ((uint16_t)frame->data[1] << 8U);
        if (index >= FLIGHT_PARAM_COUNT)
        {
            return flight_param_error(frame->source, "PARAM: unknown ID");
        }
        flight_params_t params;
        (void)osKernelLock();
        params = s_data.params;
        (void)osKernelUnlock();
        memcpy((uint8_t *)&params + s_param_info[index].offset, &frame->data[2], sizeof(float));
        if (!flight_request_params(&params))
        {
            return flight_param_error(frame->source, "PARAM: rejected (armed, busy or range)");
        }
    }
    else if (frame->id == ANOPROTOCOL_PARAM_COMMAND)
    {
        if (frame->length == 0U)
        {
            return flight_param_error(frame->source, "PARAM: missing command");
        }
        uint8_t command = frame->data[0];
        if (command == ANOPROTOCOL_PARAM_GET_DEVICE)
        {
            const uint16_t versions[4] = {100U, 100U, 0U, 810U};
            const char name[] = "DreamFly2";
            id = ANOPROTOCOL_DEVICE_INFO;
            data[0] = ANOPROTOCOL_SOURCE_ADDRESS;
            memcpy(&data[1], versions, sizeof(versions));
            memcpy(&data[9], name, sizeof(name) - 1U);
            length = 9U + sizeof(name) - 1U;
        }
        else if (command == ANOPROTOCOL_PARAM_GET_COUNT)
        {
            id = ANOPROTOCOL_PARAM_COMMAND;
            data[0] = ANOPROTOCOL_PARAM_GET_COUNT;
            data[1] = (uint8_t)FLIGHT_PARAM_COUNT;
            data[2] = (uint8_t)(FLIGHT_PARAM_COUNT >> 8U);
            length = 3U;
        }
        else
        {
            if (frame->length != 3U)
            {
                return flight_param_error(frame->source, "PARAM: invalid command length");
            }
            uint16_t index = (uint16_t)frame->data[1] | ((uint16_t)frame->data[2] << 8U);
            if (command == ANOPROTOCOL_PARAM_SUBCOMMAND)
            {
                if (index == ANOPROTOCOL_PARAM_SAVE)
                {
                    if (!flight_save_params())
                    {
                        return flight_param_error(frame->source, "PARAM: Flash save failed or armed");
                    }
                }
                else if (index == ANOPROTOCOL_PARAM_RESET)
                {
                    if (!flight_request_params(&s_default_params))
                    {
                        return flight_param_error(frame->source, "PARAM: reset rejected (armed or busy)");
                    }
                }
                else
                {
                    return flight_param_error(frame->source, "PARAM: unsupported subcommand");
                }
            }
            else if (index >= FLIGHT_PARAM_COUNT)
            {
                return flight_param_error(frame->source, "PARAM: unknown ID");
            }
            else if (command == ANOPROTOCOL_PARAM_GET_VALUE)
            {
                id = ANOPROTOCOL_PARAM_VALUE;
                data[0] = (uint8_t)index;
                data[1] = (uint8_t)(index >> 8U);
                (void)osKernelLock();
                memcpy(&data[2], (const uint8_t *)&s_data.params + s_param_info[index].offset, sizeof(float));
                (void)osKernelUnlock();
                length = 6U;
            }
            else if (command == ANOPROTOCOL_PARAM_GET_INFO)
            {
                id = ANOPROTOCOL_PARAM_INFO;
                data[0] = (uint8_t)index;
                data[1] = (uint8_t)(index >> 8U);
                data[2] = 8U; /* PVT_Float: raw IEEE754 float32 value, no integer scaling. */
                int32_t minimum = 0;
                memcpy(&data[3], &minimum, sizeof(minimum));
                memcpy(&data[7], &s_param_info[index].max, sizeof(s_param_info[index].max));
                uint16_t name_size = (uint16_t)strlen(s_param_info[index].name);
                if (name_size > 20U) { name_size = 20U; }
                memcpy(&data[11], s_param_info[index].name, name_size);
                uint16_t info_size = (uint16_t)strlen(s_param_info[index].info);
                if (info_size > sizeof(data) - 31U) { info_size = sizeof(data) - 31U; }
                memcpy(&data[31], s_param_info[index].info, info_size);
                length = 31U + info_size;
            }
            else
            {
                return flight_param_error(frame->source, "PARAM: unsupported command");
            }
        }
    }
    else
    {
        return 0U;
    }

    if (id == 0x00U)
    {
        data[0] = frame->id;
        data[1] = frame->sc;
        data[2] = frame->ac;
        length = 3U;
    }
    return anoprotocol_pack_frame(ANOPROTOCOL_SOURCE_ADDRESS, frame->source, id,
                                  data, length, s_param_io.tx, sizeof(s_param_io.tx));
}

/** @brief Service bounded UART input and one pending response in the debug task. */
static void flight_process_parameters(void)
{
    uint32_t now = HAL_GetTick();
    if (s_param_io.tx_size != 0U)
    {
        bool rejected = false;
        bool waiting;
        (void)osKernelLock();
        if (s_param_change.state == PARAM_CHANGE_WAITING
            && (uint32_t)(now - s_param_io.change_start_ms) >= FLIGHT_PARAM_APPLY_TIMEOUT_MS)
        {
            s_param_change.state = PARAM_CHANGE_REJECTED;
        }
        waiting = s_param_change.state == PARAM_CHANGE_WAITING;
        rejected = s_param_change.state == PARAM_CHANGE_REJECTED;
        if (!waiting)
        {
            s_param_change.state = PARAM_CHANGE_NONE;
        }
        (void)osKernelUnlock();
        if (waiting)
        {
            return;
        }
        if (rejected)
        {
            uint8_t destination = s_param_io.tx[2];
            s_param_io.tx_size = flight_param_error(destination, "PARAM: apply rejected or timed out");
        }
        if (debug_send(s_param_io.tx, s_param_io.tx_size))
        {
            s_param_io.tx_size = 0U;
        }
        return;
    }

    int16_t received = debug_receive(&s_param_io.rx[s_param_io.rx_size],
                                     sizeof(s_param_io.rx) - s_param_io.rx_size);
    if (received < 0)
    {
        s_param_io.rx_size = 0U;
        return;
    }
    if (received > 0)
    {
        s_param_io.rx_size += (uint16_t)received;
        s_param_io.last_rx_ms = now;
    }
    while (s_param_io.rx_size != 0U)
    {
        anoprotocol_frame_t frame;
        int16_t consumed = anoprotocol_parse_frame(s_param_io.rx, s_param_io.rx_size, &frame);
        if (consumed == 0)
        {
            if ((uint32_t)(now - s_param_io.last_rx_ms) < FLIGHT_PARAM_RX_TIMEOUT_MS)
            {
                return;
            }
            /* Discard only the stale prefix, preserving any complete frame
             * already queued behind a truncated/corrupt one.
             */
            consumed = -1;
        }
        if (consumed > 0
            && (frame.destination == ANOPROTOCOL_SOURCE_ADDRESS
                || frame.destination == ANOPROTOCOL_BROADCAST_ADDRESS)
            && (frame.id == ANOPROTOCOL_PARAM_COMMAND || frame.id == ANOPROTOCOL_PARAM_VALUE))
        {
            s_param_io.tx_size = flight_handle_parameter(&frame);
        }
        uint16_t remove = consumed > 0 ? (uint16_t)consumed : 1U;
        s_param_io.rx_size -= remove;
        memmove(s_param_io.rx, &s_param_io.rx[remove], s_param_io.rx_size);
        if (s_param_io.tx_size != 0U)
        {
            /* The next service sends the reply after any queued gains are applied.
             * Periodic telemetry may be skipped while command responses are pending.
             */
            return;
        }
    }
}

void flight_init(void)
{
    s_data = (flight_data_t){0};
    attitude_vector_t acc_g = {0.0f, 0.0f, 1.0f};
    flight_imu_t imu;

    s_remote = (flight_remote_state_t){
        .state = FLIGHT_LOCKED,
        .gesture = GESTURE_WAIT_RELEASE
    };
    s_rgb = (flight_rgb_state_t){
        .status = FLIGHT_STATUS_WAIT_PAIRING,
        .sensor_error = FLIGHT_SENSOR_NONE
    };
    motor_init();
    led_init();
    ws2812b_init();
    (void)osKernelLock();
    s_init_stage = FLIGHT_INIT_RUNNING;
    (void)osKernelUnlock();

    (void)flight_load_params();

    /* P-only bench demonstration; these gains are not tuned for flight. */
    float rate_limit = FLIGHT_RATE_TEST_ENABLED ? FLIGHT_RATE_TEST_OUTPUT_LIMIT : 100.0f;
    pid_init(&s_rate_pid.roll, 1.0f, 0.0f, 0.0f, 10.0f, 50.0f, rate_limit);
    pid_init(&s_rate_pid.pitch, 1.0f, 0.0f, 0.0f, 10.0f, 50.0f, rate_limit);
    pid_init(&s_rate_pid.yaw, 1.0f, 0.0f, 0.0f, 10.0f, 50.0f, rate_limit);

    if (flight_init_sensors())
    {
        s_data.acc_calibrated = flight_calibrate_accel();
        if (flight_read_imu(&imu))
        {
            acc_g = (attitude_vector_t){imu.acc_x_g, imu.acc_y_g, imu.acc_z_g};
        }
    }
    attitude_init(&acc_g);
    s_data.radio_ready = si24r1_init(SI24R1_MODE_RX);
    (void)debug_init();

    (void)osKernelLock();
    s_init_stage = FLIGHT_INIT_COMPLETE;
    (void)osKernelUnlock();
}

bool flight_process_remote(const rcprotocol_control_t *control,
                           bool connected, bool imu_ready, uint32_t now_ms)
{
    if (!connected || !imu_ready || control == NULL)
    {
        s_remote = (flight_remote_state_t){
            .state = FLIGHT_LOCKED,
            .gesture = GESTURE_WAIT_RELEASE
        };
        return false;
    }

    /* LH/LV: yaw/throttle; RH/RV: roll/pitch. Left/down = 1000. */
    bool inward = control->yaw >= FLIGHT_STICK_HIGH
        && control->throttle <= FLIGHT_STICK_LOW
        && control->roll <= FLIGHT_STICK_LOW
        && control->pitch <= FLIGHT_STICK_LOW;
    if (!inward)
    {
        s_remote.gesture = GESTURE_IDLE;
    }
    else if (s_remote.gesture == GESTURE_IDLE)
    {
        s_remote.gesture_start_ms = now_ms;
        s_remote.gesture = GESTURE_HOLDING;
    }
    else if (s_remote.gesture == GESTURE_HOLDING
             && (uint32_t)(now_ms - s_remote.gesture_start_ms) >= FLIGHT_ARM_HOLD_MS)
    {
        s_remote.state = s_remote.state == FLIGHT_LOCKED
            ? FLIGHT_ARM_INDICATING : FLIGHT_LOCKED;
        s_remote.state_start_ms = now_ms;
        s_remote.gesture = GESTURE_WAIT_RELEASE;
    }

    /* Only idle on the ground may auto-lock. Packet sequence changes are not
     * operator activity. Never apply this rule after a takeoff request.
     */
    if (s_remote.state != FLIGHT_WAIT_CENTER && s_remote.state != FLIGHT_CONFIRM_CENTER
        && s_remote.state != FLIGHT_WAIT_THROTTLE)
    {
        s_remote.idle_start_ms = now_ms;
        s_remote.idle_control = *control;
    }
    else if (flight_channel_moved(control->roll, s_remote.idle_control.roll)
             || flight_channel_moved(control->pitch, s_remote.idle_control.pitch)
             || flight_channel_moved(control->yaw, s_remote.idle_control.yaw)
             || flight_channel_moved(control->throttle, s_remote.idle_control.throttle)
             || control->keys != s_remote.idle_control.keys)
    {
        s_remote.idle_start_ms = now_ms;
        s_remote.idle_control = *control;
    }
    else if ((uint32_t)(now_ms - s_remote.idle_start_ms) >= FLIGHT_IDLE_LOCK_MS)
    {
        s_remote = (flight_remote_state_t){
            .state = FLIGHT_LOCKED,
            .gesture = GESTURE_WAIT_RELEASE
        };
        return false;
    }

    if (s_remote.state == FLIGHT_ARM_INDICATING)
    {
        uint32_t elapsed_ms = (uint32_t)(now_ms - s_remote.state_start_ms);
        uint32_t slot_ms = FLIGHT_ARM_SPIN_MS + FLIGHT_ARM_SPIN_GAP_MS;
        if (elapsed_ms >= 3U * slot_ms + FLIGHT_ARM_SPIN_MS)
        {
            s_remote.state = FLIGHT_WAIT_CENTER;
        }
    }

    if (s_remote.state == FLIGHT_WAIT_CENTER || s_remote.state == FLIGHT_CONFIRM_CENTER)
    {
        bool centered = flight_stick_centered(control->roll)
            && flight_stick_centered(control->pitch)
            && flight_stick_centered(control->yaw)
            && flight_stick_centered(control->throttle);
        if (!centered)
        {
            s_remote.state = FLIGHT_WAIT_CENTER;
        }
        else if (s_remote.state == FLIGHT_WAIT_CENTER)
        {
            s_remote.state = FLIGHT_CONFIRM_CENTER;
            s_remote.state_start_ms = now_ms;
        }
        else if ((uint32_t)(now_ms - s_remote.state_start_ms) >= FLIGHT_CENTER_HOLD_MS)
        {
            s_remote.state = FLIGHT_WAIT_THROTTLE;
        }
    }
    else if (s_remote.state == FLIGHT_WAIT_THROTTLE
             && control->throttle >= FLIGHT_TAKEOFF_THROTTLE)
    {
        /* The request may authorize a limited bench test, never automatic takeoff. */
        s_remote.state = FLIGHT_TAKEOFF_REQUESTED;
    }
    else if (FLIGHT_RATE_TEST_ENABLED && s_remote.state == FLIGHT_TAKEOFF_REQUESTED
             && control->throttle <= FLIGHT_STICK_CENTER_MAX)
    {
        s_remote.state = FLIGHT_WAIT_CENTER;
    }

    return s_remote.state != FLIGHT_LOCKED;
}

bool flight_takeoff_requested(void)
{
    return s_remote.state == FLIGHT_TAKEOFF_REQUESTED;
}

void flight_rate_control(const flight_axis_t *target_dps,
                         const flight_axis_t *measurement_dps,
                         float sample_time_s,
                         bool integral_enabled,
                         flight_axis_t *output)
{
    s_rate_pid.roll.target = target_dps->roll;
    s_rate_pid.roll.measurement = measurement_dps->roll;
    pid_calculate(&s_rate_pid.roll, sample_time_s, integral_enabled);

    s_rate_pid.pitch.target = target_dps->pitch;
    s_rate_pid.pitch.measurement = measurement_dps->pitch;
    pid_calculate(&s_rate_pid.pitch, sample_time_s, integral_enabled);

    s_rate_pid.yaw.target = target_dps->yaw;
    s_rate_pid.yaw.measurement = measurement_dps->yaw;
    pid_calculate(&s_rate_pid.yaw, sample_time_s, integral_enabled);

    output->roll = s_rate_pid.roll.output;
    output->pitch = s_rate_pid.pitch.output;
    output->yaw = s_rate_pid.yaw.output;
}

void flight_led_toggle(void)
{
    led_toggle();
}

void flight_set_status(flight_status_t status)
{
    s_rgb.status = status;
}

void flight_set_sensor_error(flight_sensor_t sensor)
{
    if (s_rgb.sensor_error != sensor)
    {
        s_rgb.sensor_error = sensor;
        s_rgb.flash_count = 0U;
    }
}

void flight_update_rgb(void)
{
    static uint32_t last_frame_ms;
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_frame_ms) < FLIGHT_RGB_PERIOD_MS)
    {
        return;
    }
    last_frame_ms = now;

    flight_init_stage_t stage;
    flight_sensor_t error = FLIGHT_SENSOR_NONE;
    bool connected = false;
    bool armed = false;
    (void)osKernelLock();
    stage = s_init_stage;
    if (stage == FLIGHT_INIT_COMPLETE)
    {
        /* Report the first startup fault; unconnected FLOW/RANGE are not errors. */
        if (!s_data.sensor_status.imu || !s_data.acc_calibrated)
        {
            error = FLIGHT_SENSOR_IMU;
        }
        else if (!s_data.sensor_status.magnetometer)
        {
            error = FLIGHT_SENSOR_MAGNETOMETER;
        }
        else if (!s_data.sensor_status.barometer)
        {
            error = FLIGHT_SENSOR_BAROMETER;
        }
        armed = s_data.armed;
        connected = s_data.radio_ready && s_data.remote_received_count != 0U
            && (uint32_t)(HAL_GetTick() - s_data.remote_last_ms) < FLIGHT_LINK_TIMEOUT_MS;
    }
    (void)osKernelUnlock();

    if (stage == FLIGHT_INIT_PENDING)
    {
        return;
    }
    if (stage == FLIGHT_INIT_RUNNING)
    {
        uint8_t brightness = (now / FLIGHT_INIT_BLINK_MS) % 2U == 0U
            ? FLIGHT_RGB_BRIGHTNESS : 0U;
        flight_rgb_all(brightness, brightness, 0U);
        return;
    }

    flight_set_sensor_error(error);
    if (!connected)
    {
        flight_set_status(FLIGHT_STATUS_WAIT_PAIRING);
    }
    else
    {
        flight_set_status(armed ? FLIGHT_STATUS_ARMED : FLIGHT_STATUS_PAIRED);
    }

    if (s_rgb.sensor_error != FLIGHT_SENSOR_NONE)
    {
        flight_show_sensor_error();
        return;
    }

    switch (s_rgb.status)
    {
        case FLIGHT_STATUS_PAIRED:
            ws2812b_gradient();
            break;

        case FLIGHT_STATUS_ARMED:
            flight_show_armed_rgb(now);
            break;

        case FLIGHT_STATUS_WAIT_PAIRING:
        default:
            flight_rgb_all(FLIGHT_RGB_BRIGHTNESS, FLIGHT_RGB_BRIGHTNESS, 0U);
            break;
    }
}

float flight_update_battery(void)
{
    return battery_read_voltage();
}

/** @brief Encode degrees in 0.01-degree units, reserving INT16_MIN for unavailable data. */
static int16_t flight_angle_cdeg(float angle_deg)
{
    if (!isfinite(angle_deg) || angle_deg < -327.67f || angle_deg > 327.67f)
    {
        return RCPROTOCOL_INVALID_I16;
    }
    float value = angle_deg * 100.0f;
    return (int16_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

/** @brief Encode fresh relative barometric height in cm; reserve INT16_MIN for unavailable data. */
static int16_t flight_height_cm(const flight_baro_t *baro, bool sample_valid,
                                uint32_t last_ms, uint32_t now_ms)
{
    if (!sample_valid || !baro->height_valid
        || (uint32_t)(now_ms - last_ms) >= FLIGHT_BARO_TIMEOUT_MS)
    {
        return RCPROTOCOL_INVALID_I16;
    }
    float height_cm = baro->height_m * 100.0f;
    if (!isfinite(height_cm) || height_cm < -32767.0f || height_cm > 32767.0f)
    {
        return RCPROTOCOL_INVALID_I16;
    }
    return (int16_t)(height_cm >= 0.0f ? height_cm + 0.5f : height_cm - 0.5f);
}

bool flight_is_initialized(void)
{
    (void)osKernelLock();
    bool complete = s_init_stage == FLIGHT_INIT_COMPLETE;
    (void)osKernelUnlock();
    return complete;
}

void flight_process_system(void)
{
    static uint8_t system_count; /* One cycle covers battery and LED schedules. */

    flight_update_rgb();
    if (!flight_is_initialized())
    {
        return;
    }

    if (system_count % FLIGHT_BATTERY_PERIOD_CYCLES == 0U)
    {
        float voltage_v = flight_update_battery();
        uint16_t voltage_cv = isfinite(voltage_v)
            && voltage_v > 0.0f && voltage_v <= 655.35f
            ? (uint16_t)(voltage_v * 100.0f + 0.5f) : 0U;
        (void)osKernelLock();
        s_data.battery_cv = voltage_cv;
        s_data.battery_last_ms = HAL_GetTick();
        (void)osKernelUnlock();
    }
    system_count++;
    if (system_count >= FLIGHT_LED_PERIOD_CYCLES)
    {
        system_count = 0U;
        flight_led_toggle();
    }
}

void flight_process_control(void)
{
    flight_imu_t imu;
    flight_axis_t target_dps = {0.0f, 0.0f, 0.0f};
    flight_axis_t output;
    bool rate_valid;
    uint32_t rate_last_ms;
    bool sample_ready;

    (void)osKernelLock();
    imu = s_data.sensors.imu;
    sample_ready = s_data.sensors.imu_pending_control;
    s_data.sensors.imu_pending_control = false;
    output = s_data.rate_output;
    rate_valid = s_data.rate_output_valid;
    rate_last_ms = s_data.rate_last_ms;
    rcprotocol_control_t remote = s_data.remote;
    uint32_t remote_last_ms = s_data.remote_last_ms;
    bool have_remote = s_data.radio_ready && s_data.remote_received_count != 0U;
    uint32_t attitude_last_ms = s_data.sensors.attitude_last_ms;
    attitude_t attitude = s_data.sensors.attitude_6axis;
    bool imu_ready = s_data.sensor_status.imu && s_data.sensors.imu_valid
        && s_data.acc_calibrated && s_data.sensors.attitude_6axis_valid;
    (void)osKernelUnlock();

    uint32_t now = HAL_GetTick();
    bool connected = have_remote
        && (uint32_t)(now - remote_last_ms) < FLIGHT_LINK_TIMEOUT_MS;
    imu_ready = imu_ready
        && (uint32_t)(now - attitude_last_ms) < FLIGHT_ATTITUDE_TIMEOUT_MS
        && isfinite(attitude.roll_deg) && isfinite(attitude.pitch_deg)
        && isfinite(attitude.yaw_deg)
        && isfinite(imu.gyro_x_dps) && isfinite(imu.gyro_y_dps)
        && isfinite(imu.gyro_z_dps);

    if (sample_ready && imu_ready)
    {
        flight_axis_t measurement_dps = {imu.gyro_x_dps, imu.gyro_y_dps, imu.gyro_z_dps};
        /* Zero-rate target only; never feed tilt angles (deg) into this loop (deg/s). */
        flight_rate_control(&target_dps, &measurement_dps,
                            FLIGHT_ATTITUDE_PERIOD_S, false, &output);
        rate_valid = isfinite(output.roll) && isfinite(output.pitch) && isfinite(output.yaw);
        rate_last_ms = attitude_last_ms;
    }
    if (!imu_ready || !rate_valid
        || (uint32_t)(now - rate_last_ms) >= FLIGHT_RATE_TEST_TIMEOUT_MS)
    {
        output = (flight_axis_t){0};
        rate_valid = false;
        pid_reset(&s_rate_pid.roll);
        pid_reset(&s_rate_pid.pitch);
        pid_reset(&s_rate_pid.yaw);
    }

    /* A stale rate result must disarm the bench, not merely pause and auto-resume. */
    bool armed = flight_process_remote(&remote, connected,
        imu_ready && (!FLIGHT_RATE_TEST_ENABLED || rate_valid), now);
    bool takeoff_requested = flight_takeoff_requested();
    flight_target_t target = {0};
    bool target_valid = connected
        && flight_map_control(&remote, FLIGHT_MAPPING_MODE, &target);
    motor_t motor_preview = {0};
    bool motor_preview_valid = false;
    if (rate_valid && target_valid && FLIGHT_MAPPING_MODE == FLIGHT_MODE_STABILIZE)
    {
        uint16_t base_pwm = FLIGHT_RATE_TEST_ENABLED ? FLIGHT_RATE_TEST_BASE_PWM : target.throttle_pwm;
        motor_preview_valid = flight_mix_motors(base_pwm, &output, &motor_preview);
    }

    (void)osKernelLock();
    s_data.armed = armed;
    s_data.takeoff_requested = takeoff_requested;
    s_data.target = target;
    s_data.target_valid = target_valid;
    s_data.rate_output = output;
    s_data.rate_output_valid = rate_valid;
    s_data.rate_last_ms = rate_last_ms;
    s_data.motor_preview = motor_preview;
    s_data.motor_preview_valid = motor_preview_valid;
    (void)osKernelUnlock();

    /* Every step arbitrates output, including stop/fault steps without a new sample. */
    flight_write_motors(now, motor_preview_valid ? &motor_preview : NULL);
}

void flight_process_sensors(void)
{
    flight_apply_params();
    flight_sensor_data_t *data = &s_data.sensors;
    flight_imu_t imu;
    attitude_vector_t acc_g;
    attitude_vector_t gyro_dps;
    attitude_vector_t mag_ut;
    attitude_t attitude;
    bool imu_sample_valid = false;
    bool attitude_sample_valid = false;
    bool baro_sample_new = false;

    if (s_data.sensor_status.imu && flight_read_imu(&imu))
    {
        (void)osKernelLock();
        data->imu = imu;
        data->imu_valid = true;
        data->imu_pending_control = true;
        (void)osKernelUnlock();
        imu_sample_valid = true;

        acc_g = (attitude_vector_t){imu.acc_x_g, imu.acc_y_g, imu.acc_z_g};
        gyro_dps = (attitude_vector_t){imu.gyro_x_dps, imu.gyro_y_dps, imu.gyro_z_dps};
        if (attitude_solve_6axis(&acc_g, &gyro_dps, FLIGHT_ATTITUDE_PERIOD_S, &attitude))
        {
            attitude_sample_valid = true;
            (void)osKernelLock();
            data->attitude_6axis = attitude;
            data->attitude_6axis_valid = true;
            data->attitude_last_ms = HAL_GetTick();
            (void)osKernelUnlock();
        }
    }

    s_sensor_count++;
    if (s_sensor_count % FLIGHT_MAG_PERIOD_CYCLES == 0U)
    {
        flight_mag_t mag;
        if (flight_read_mag(&mag))
        {
            (void)osKernelLock();
            data->mag = mag;
            data->mag_valid = true;
            (void)osKernelUnlock();
        }
    }
    if (s_sensor_count >= FLIGHT_BARO_PERIOD_CYCLES)
    {
        s_sensor_count = 0U;
        flight_baro_t baro;
        if (flight_read_baro(&baro))
        {
            baro_sample_new = baro.height_valid;
            (void)osKernelLock();
            data->baro = baro;
            data->baro_valid = true;
            data->baro_last_ms = HAL_GetTick();
            (void)osKernelUnlock();
        }
    }

    /* Update the one shared estimator in place. No I2C or blocking calls are
     * made while locked; readers cannot copy a half-updated height state.
     */
    uint32_t now = HAL_GetTick();
    (void)osKernelLock();
    if (!attitude_sample_valid || !s_data.acc_calibrated || !data->baro.height_valid
        || (uint32_t)(now - data->baro_last_ms) >= FLIGHT_BARO_TIMEOUT_MS)
    {
        data->height_valid = false;
    }
    else if (!data->height_valid)
    {
        if (baro_sample_new)
        {
            height_init(&data->height, data->baro.height_m);
            s_height_correction_ms = now;
            data->height_valid = true;
        }
    }
    else
    {
        float r = attitude.roll_deg * FLIGHT_DEG_TO_RAD;
        float p = attitude.pitch_deg * FLIGHT_DEG_TO_RAD;
        float acc_up_mps2 = (-acc_g.x * sinf(p)
            + acc_g.y * sinf(r) * cosf(p)
            + acc_g.z * cosf(r) * cosf(p) - 1.0f) * FLIGHT_GRAVITY_MPS2;
        uint32_t elapsed_ms = (uint32_t)(now - data->height_last_ms);
        if (elapsed_ms != 0U)
        {
            data->height_valid = height_predict(&data->height, acc_up_mps2,
                                                (float)elapsed_ms * 0.001f);
        }
        if (data->height_valid && baro_sample_new)
        {
            data->height_valid = height_correct(&data->height, data->baro.height_m,
                (float)(uint32_t)(now - s_height_correction_ms) * 0.001f);
            s_height_correction_ms = now;
        }
    }
    if (data->height_valid)
    {
        data->height_last_ms = now;
    }
    (void)osKernelUnlock();

    if (imu_sample_valid && data->mag_valid)
    {
        mag_ut = (attitude_vector_t){data->mag.mag_x_ut, data->mag.mag_y_ut, data->mag.mag_z_ut};
        if (attitude_solve_9axis(&acc_g, &gyro_dps, &mag_ut, FLIGHT_ATTITUDE_PERIOD_S, &attitude))
        {
            (void)osKernelLock();
            data->attitude_9axis = attitude;
            data->attitude_9axis_valid = true;
            (void)osKernelUnlock();
        }
    }
}

void flight_process_debug(void)
{
    flight_process_parameters();

    flight_data_t data;
    (void)osKernelLock();
    data = s_data;
    (void)osKernelUnlock();
    debug_report(&data);
}

void flight_process_link(void)
{
    static uint8_t status_sequence; /* Advanced only when an ACK payload is queued. */
    static bool have_control; /* Retained even if the 32-bit receive count wraps. */
    bool radio_ready;

    (void)osKernelLock();
    radio_ready = s_data.radio_ready;
    (void)osKernelUnlock();

    if (radio_ready)
    {
        /* Drain the hardware's three-entry FIFO without waiting for a packet. */
        for (uint8_t i = 0U; i < 3U; i++)
        {
            uint8_t buffer[SI24R1_MAX_PAYLOAD_SIZE];
            uint8_t size;
            rcprotocol_control_t incoming;
            if (!si24r1_receive(buffer, sizeof(buffer), &size))
            {
                break;
            }
            if (!rcprotocol_parse_control(buffer, size, &incoming))
            {
                continue;
            }

            uint32_t now = HAL_GetTick();
            if (have_control && incoming.sequence == s_data.remote.sequence
                && (uint32_t)(now - s_data.remote_last_ms) < FLIGHT_LINK_TIMEOUT_MS)
            {
                continue;
            }
            (void)osKernelLock();
            s_data.remote = incoming;
            s_data.remote_last_ms = now;
            s_data.remote_received_count++;
            (void)osKernelUnlock();
            have_control = true;
        }

        if (have_control)
        {
            /* Copy published data only; no ADC/I2C acquisition in the link task. */
            attitude_t attitude;
            flight_sensor_status_t sensor_status;
            bool attitude_valid;
            bool armed;
            uint32_t attitude_last_ms;
            uint32_t battery_last_ms;
            uint16_t battery_cv;
            flight_baro_t baro;
            bool baro_valid;
            uint32_t baro_last_ms;
            (void)osKernelLock();
            attitude = s_data.sensors.attitude_6axis;
            attitude_valid = s_data.sensors.attitude_6axis_valid;
            attitude_last_ms = s_data.sensors.attitude_last_ms;
            sensor_status = s_data.sensor_status;
            battery_cv = s_data.battery_cv;
            battery_last_ms = s_data.battery_last_ms;
            armed = s_data.armed;
            baro = s_data.sensors.baro;
            baro_valid = s_data.sensors.baro_valid;
            baro_last_ms = s_data.sensors.baro_last_ms;
            (void)osKernelUnlock();

            uint32_t now = HAL_GetTick();
            attitude_valid = attitude_valid
                && (uint32_t)(now - attitude_last_ms) < FLIGHT_ATTITUDE_TIMEOUT_MS;
            rcprotocol_status_t status = {
                .sequence = status_sequence,
                .roll_cdeg = RCPROTOCOL_INVALID_I16,
                .pitch_cdeg = RCPROTOCOL_INVALID_I16,
                .yaw_cdeg = RCPROTOCOL_INVALID_I16,
                .throttle = s_data.remote.throttle,
                .height_cm = flight_height_cm(&baro, baro_valid, baro_last_ms, now),
                .battery_cv = (uint32_t)(now - battery_last_ms)
                    < FLIGHT_BATTERY_TIMEOUT_MS ? battery_cv : 0U
            };
            if (attitude_valid)
            {
                status.roll_cdeg = flight_angle_cdeg(attitude.roll_deg);
                status.pitch_cdeg = flight_angle_cdeg(attitude.pitch_deg);
                /* The solver already emits yaw in -180..180 degrees. */
                status.yaw_cdeg = flight_angle_cdeg(attitude.yaw_deg);
            }
            if (!attitude_valid || status.roll_cdeg == RCPROTOCOL_INVALID_I16
                || status.pitch_cdeg == RCPROTOCOL_INVALID_I16
                || status.yaw_cdeg == RCPROTOCOL_INVALID_I16
                || !sensor_status.imu)
            {
                status.sensor_errors |= RCPROTOCOL_SENSOR_ERROR_IMU;
            }
            /* MAG/BARO bits report configured onboard sensor initialization.
             * Unconnected FLOW/RANGE modules are not marked as failed.
             */
            if (!sensor_status.magnetometer)
            {
                status.sensor_errors |= RCPROTOCOL_SENSOR_ERROR_MAG;
            }
            if (!sensor_status.barometer)
            {
                status.sensor_errors |= RCPROTOCOL_SENSOR_ERROR_BARO;
            }
            if (armed
                && (uint32_t)(now - s_data.remote_last_ms) < FLIGHT_LINK_TIMEOUT_MS
                && (status.sensor_errors & RCPROTOCOL_SENSOR_ERROR_IMU) == 0U)
            {
                status.flags |= RCPROTOCOL_STATUS_ARMED;
            }
            /* Barometric height is telemetry only; no takeoff/height controller
             * is connected. ACK data is queued for a future packet.
             */
            uint8_t buffer[RCPROTOCOL_STATUS_FRAME_SIZE];
            uint8_t size = rcprotocol_pack_status(&status, buffer, sizeof(buffer));
            if (size != 0U && si24r1_write_ack_payload(buffer, size))
            {
                status_sequence++;
            }
        }

        /* In RX mode this reads only the driver's transport-fault latch. */
        radio_ready = si24r1_get_tx_result() != SI24R1_TX_ERROR;
        (void)osKernelLock();
        s_data.radio_ready = radio_ready;
        (void)osKernelUnlock();
    }
}
