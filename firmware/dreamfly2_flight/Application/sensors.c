/**
 * @file sensors.c
 * @brief Unified flight-sensor implementation.
 * @author 梅雨
 * @date 2026-10-11
 */

#include "sensors.h"

#include <stddef.h>
#include <stdint.h>

#include "cmsis_os2.h"

#include "filter.h"
#include "ak09916.h"
#include "bmp280.h"
#include "icm20948.h"

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

static sensors_status_t s_status;
static float s_accel_bias[3]; /* XYZ offsets in g; horizontal calibration preserves +1 g on Z. */
static float s_gyro_bias[3]; /* XYZ zero-rate offsets in degrees/s, stored in RAM only. */
static filter_kalman_1d_t s_accel_filter[3];
static filter_low_pass_t s_gyro_filter[3];

/** @brief Test per-axis gyro magnitude and inter-sample change; inputs must be valid. */
static bool imu_is_still(const icm20948_t *sample,
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
static void imu_init_filters(const icm20948_t *sample)
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
static bool imu_calibrate(void)
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

        if (last_gyro_valid && imu_is_still(&sample, last_gyro))
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

    imu_init_filters(&sample);
    return true;
}

bool sensors_init(void)
{
    s_status = (sensors_status_t){0};

    if (!icm20948_init() || !imu_calibrate())
    {
        return false;
    }

    s_status.imu = true;
    s_status.magnetometer = ak09916_init();
    s_status.barometer = bmp280_init();

    return true;
}

bool sensors_calibrate_accel_level(void)
{
    icm20948_t sample;
    float accel_sum[3] = {0.0f};
    float last_gyro[3] = {0.0f};
    uint16_t sample_count = 0U;
    uint16_t attempt_count = 0U;
    bool last_gyro_valid = false;

    if (!s_status.imu)
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
            && imu_is_still(&sample, last_gyro)
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

    imu_init_filters(&sample);
    return true;
}

sensors_status_t sensors_get_status(void)
{
    return s_status;
}

bool sensors_read_imu(sensors_imu_t *imu_data)
{
    icm20948_t sample;
    float accel[3];
    float gyro[3];

    if ((imu_data == NULL) || !s_status.imu || !icm20948_read(&sample))
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

bool sensors_read_mag(sensors_mag_t *mag_data)
{
    ak09916_t sample;

    if ((mag_data == NULL)
        || !s_status.magnetometer
        || !ak09916_read(&sample))
    {
        return false;
    }

    mag_data->mag_x_ut = sample.mag_x_ut;
    mag_data->mag_y_ut = sample.mag_y_ut;
    mag_data->mag_z_ut = sample.mag_z_ut;

    return true;
}

bool sensors_read_baro(sensors_baro_t *baro_data)
{
    bmp280_t sample;

    if ((baro_data == NULL)
        || !s_status.barometer
        || !bmp280_read(&sample))
    {
        return false;
    }

    baro_data->temp_c = sample.temp_c;
    baro_data->press_pa = sample.press_pa;

    return true;
}
