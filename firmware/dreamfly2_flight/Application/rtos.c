/**
 * @file rtos.c
 * @brief FreeRTOS task implementations for the flight controller.
 * @author 梅雨
 * @date 2026-10-08
 */

#include "rtos.h"

#include "cmsis_os2.h"

#include "attitude.h"
#include "flight.h"
#include "sensors.h"

#include "debug.h"

#define FLIGHT_CONTROL_TASK_PERIOD_TICKS 2U
#define FLIGHT_SYSTEM_TASK_PERIOD_TICKS 10U
#define FLIGHT_SENSOR_TASK_PERIOD_TICKS 2U
#define FLIGHT_DEBUG_TASK_PERIOD_TICKS 100U
#define FLIGHT_LINK_TASK_PERIOD_TICKS 5U
#define FLIGHT_LED_PERIOD_CYCLES 50U
#define FLIGHT_SLOW_SENSOR_PERIOD_CYCLES 5U
#define FLIGHT_ATTITUDE_PERIOD_S 0.002f

static sensors_imu_t s_latest_imu;
static sensors_mag_t s_latest_mag;
static sensors_baro_t s_latest_baro;
static sensors_status_t s_sensor_status;
static attitude_t s_latest_attitude_6axis;
static attitude_t s_latest_attitude_9axis;
static bool s_latest_imu_valid;
static bool s_latest_mag_valid;
static bool s_latest_baro_valid;
static bool s_sensor_status_valid;
static bool s_accel_calibrated;
static bool s_latest_attitude_6axis_valid;
static bool s_latest_attitude_9axis_valid;
static flight_axis_t s_latest_rate_output;
static bool s_latest_rate_output_valid;
static bool s_imu_pending_control;

void flight_system_task_entry(void *argument)
{
    uint32_t next_wake_tick;
    uint16_t led_count = 0U;

    (void)argument;

    next_wake_tick = osKernelGetTickCount();

    for (;;)
    {
        ++led_count;
        if (led_count >= FLIGHT_LED_PERIOD_CYCLES)
        {
            led_count = 0U;
            flight_led_toggle();
        }

        next_wake_tick += FLIGHT_SYSTEM_TASK_PERIOD_TICKS;
        osDelayUntil(next_wake_tick);
    }
}

void flight_control_task_entry(void *argument)
{
    uint32_t next_wake_tick;
    sensors_imu_t imu;
    flight_axis_t target_dps = {0.0f, 0.0f, 0.0f};
    flight_axis_t measurement_dps;
    flight_axis_t output;
    bool sample_ready;

    (void)argument;

    flight_init();
    next_wake_tick = osKernelGetTickCount();

    for (;;)
    {
        (void)osKernelLock();
        imu = s_latest_imu;
        sample_ready = s_imu_pending_control;
        s_imu_pending_control = false;
        (void)osKernelUnlock();

        if (sample_ready)
        {
            measurement_dps = (flight_axis_t){imu.gyro_x_dps,
                                               imu.gyro_y_dps,
                                               imu.gyro_z_dps};
            flight_rate_control(&target_dps, &measurement_dps,
                                FLIGHT_ATTITUDE_PERIOD_S, false, &output);
            (void)osKernelLock();
            s_latest_rate_output = output;
            s_latest_rate_output_valid = true;
            (void)osKernelUnlock();
        }

        next_wake_tick += FLIGHT_CONTROL_TASK_PERIOD_TICKS;
        osDelayUntil(next_wake_tick);
    }
}

void flight_sensor_task_entry(void *argument)
{
    sensors_imu_t imu;
    sensors_mag_t mag;
    sensors_baro_t baro;
    attitude_vector_t acc_g;
    attitude_vector_t gyro_dps;
    attitude_vector_t mag_ut;
    attitude_t attitude;
    uint32_t next_wake_tick;
    uint8_t slow_sensor_count = 0U;
    bool sensors_ready;
    bool mag_ready = false;
    bool imu_sample_valid;
    bool attitude_initialized = false;

    (void)argument;

    sensors_ready = sensors_init();
    if (sensors_ready)
    {
        s_accel_calibrated = sensors_calibrate_accel_level();
    }
    s_sensor_status = sensors_get_status();
    s_sensor_status_valid = true;
    next_wake_tick = osKernelGetTickCount();

    for (;;)
    {
        imu_sample_valid = false;

        if (sensors_ready && sensors_read_imu(&imu))
        {
            (void)osKernelLock();
            s_latest_imu = imu;
            s_latest_imu_valid = true;
            s_imu_pending_control = true;
            (void)osKernelUnlock();
            imu_sample_valid = true;

            acc_g = (attitude_vector_t){imu.acc_x_g,
                                        imu.acc_y_g,
                                        imu.acc_z_g};
            gyro_dps = (attitude_vector_t){imu.gyro_x_dps,
                                           imu.gyro_y_dps,
                                           imu.gyro_z_dps};

            if (!attitude_initialized)
            {
                attitude_init(&acc_g);
                attitude_initialized = true;
            }

            if (attitude_solve_6axis(&acc_g,
                                     &gyro_dps,
                                     FLIGHT_ATTITUDE_PERIOD_S,
                                     &attitude))
            {
                s_latest_attitude_6axis = attitude;
                s_latest_attitude_6axis_valid = true;
            }
        }

        ++slow_sensor_count;
        if (slow_sensor_count >= FLIGHT_SLOW_SENSOR_PERIOD_CYCLES)
        {
            slow_sensor_count = 0U;

            if (sensors_read_mag(&mag))
            {
                s_latest_mag = mag;
                s_latest_mag_valid = true;
                mag_ready = true;
            }

            if (sensors_read_baro(&baro))
            {
                s_latest_baro = baro;
                s_latest_baro_valid = true;
            }
        }

        if (imu_sample_valid && mag_ready)
        {
            mag_ut = (attitude_vector_t){mag.mag_x_ut,
                                         mag.mag_y_ut,
                                         mag.mag_z_ut};

            if (attitude_solve_9axis(&acc_g,
                                     &gyro_dps,
                                     &mag_ut,
                                     FLIGHT_ATTITUDE_PERIOD_S,
                                     &attitude))
            {
                s_latest_attitude_9axis = attitude;
                s_latest_attitude_9axis_valid = true;
            }
        }

        next_wake_tick += FLIGHT_SENSOR_TASK_PERIOD_TICKS;
        osDelayUntil(next_wake_tick);
    }
}

void flight_debug_task_entry(void *argument)
{
    sensors_imu_t imu;
    sensors_mag_t mag;
    sensors_baro_t baro;
    sensors_status_t status;
    attitude_t attitude_6axis;
    attitude_t attitude_9axis;
    flight_axis_t rate_output;
    bool rate_output_valid;
    uint32_t next_wake_tick;
    uint8_t debug_page = 0U;
    bool imu_valid;
    bool mag_valid;
    bool baro_valid;
    bool status_valid;
    bool accel_calibrated;
    bool attitude_6axis_valid;
    bool attitude_9axis_valid;
    bool status_reported = false;

    (void)argument;

    next_wake_tick = osKernelGetTickCount();

    for (;;)
    {
        (void)osKernelLock();
        imu = s_latest_imu;
        mag = s_latest_mag;
        baro = s_latest_baro;
        status = s_sensor_status;
        attitude_6axis = s_latest_attitude_6axis;
        attitude_9axis = s_latest_attitude_9axis;
        imu_valid = s_latest_imu_valid;
        mag_valid = s_latest_mag_valid;
        baro_valid = s_latest_baro_valid;
        status_valid = s_sensor_status_valid;
        accel_calibrated = s_accel_calibrated;
        attitude_6axis_valid = s_latest_attitude_6axis_valid;
        attitude_9axis_valid = s_latest_attitude_9axis_valid;
        rate_output = s_latest_rate_output;
        rate_output_valid = s_latest_rate_output_valid;
        (void)osKernelUnlock();

        if (status_valid && !status_reported)
        {
            debug_printf("sensors: imu=%u acc_cal=%u mag=%u baro=%u\r\n",
                         (unsigned int)status.imu,
                         (unsigned int)accel_calibrated,
                         (unsigned int)status.magnetometer,
                         (unsigned int)status.barometer);
            status_reported = true;
        }
        else if ((debug_page == 0U) && imu_valid)
        {
            debug_printf("acc=%.3f,%.3f,%.3f g gyro=%.2f,%.2f,%.2f dps\r\n",
                         imu.acc_x_g,
                         imu.acc_y_g,
                         imu.acc_z_g,
                         imu.gyro_x_dps,
                         imu.gyro_y_dps,
                         imu.gyro_z_dps);
        }
        else if ((debug_page == 1U) && (mag_valid || baro_valid))
        {
            debug_printf("mag=%.2f,%.2f,%.2f uT temp=%.2f C press=%.1f Pa\r\n",
                         mag.mag_x_ut,
                         mag.mag_y_ut,
                         mag.mag_z_ut,
                         baro.temp_c,
                         baro.press_pa);
        }
        else if ((debug_page == 2U)
                 && attitude_6axis_valid
                 && attitude_9axis_valid)
        {
            debug_printf("att6=%.1f,%.1f,%.1f att9=%.1f,%.1f,%.1f deg\r\n",
                         attitude_6axis.roll_deg,
                         attitude_6axis.pitch_deg,
                         attitude_6axis.yaw_deg,
                         attitude_9axis.roll_deg,
                         attitude_9axis.pitch_deg,
                         attitude_9axis.yaw_deg);
        }

        else if ((debug_page == 3U) && rate_output_valid)
        {
            debug_printf("rate_pid=%.2f,%.2f,%.2f target=0,0,0 dps\r\n",
                         rate_output.roll, rate_output.pitch, rate_output.yaw);
        }

        ++debug_page;
        if (debug_page >= 4U)
        {
            debug_page = 0U;
        }

        next_wake_tick += FLIGHT_DEBUG_TASK_PERIOD_TICKS;
        osDelayUntil(next_wake_tick);
    }
}

void flight_link_task_entry(void *argument)
{
    uint32_t next_wake_tick;

    (void)argument;

    next_wake_tick = osKernelGetTickCount();

    for (;;)
    {
        next_wake_tick += FLIGHT_LINK_TASK_PERIOD_TICKS;
        osDelayUntil(next_wake_tick);
    }
}
