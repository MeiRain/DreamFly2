/**
 * @file rtos.c
 * @brief FreeRTOS task implementations for the flight controller.
 * @author 梅雨
 * @date 2026-10-08
 */

#include "rtos.h"

#include <math.h>

#include "cmsis_os2.h"

#include "attitude.h"
#include "flight.h"
#include "sensors.h"
#include "si24r1.h"
#include "stm32f4xx_hal.h"

#include "debug.h"

/* Periods use kernel ticks; the fixed 0.002 s algorithm period assumes a 1 kHz tick. */
#define FLIGHT_CONTROL_TASK_PERIOD_TICKS 2U
#define FLIGHT_SYSTEM_TASK_PERIOD_TICKS 10U
#define FLIGHT_SENSOR_TASK_PERIOD_TICKS 2U
#define FLIGHT_DEBUG_TASK_PERIOD_TICKS 100U
#define FLIGHT_LINK_TASK_PERIOD_TICKS 5U
#define FLIGHT_LED_PERIOD_CYCLES 50U
#define FLIGHT_SLOW_SENSOR_PERIOD_CYCLES 5U
#define FLIGHT_ATTITUDE_PERIOD_S 0.002f
#define FLIGHT_LINK_TIMEOUT_MS 300U
#define FLIGHT_ATTITUDE_TIMEOUT_MS 100U
#define FLIGHT_BATTERY_PERIOD_CYCLES 10U
#define FLIGHT_BATTERY_TIMEOUT_MS 500U
/* Select USART1 RC verification; USART2 attitude telemetry remains enabled. */
#define FLIGHT_DEBUG_REMOTE_ENABLED 1U

/* Private snapshots exchanged by these tasks; validity flags mean "received at
 * least once", not "fresh now". Scheduler locks prevent task-level copy races,
 * but do not exclude interrupts. No ISR may write these snapshots.
 */
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
static bool s_imu_pending_control; /* One fresh IMU sample is waiting for the rate loop. */

/* The link task is the sole radio owner; the debug task copies these under lock. */
static rcprotocol_control_t s_latest_remote;
static uint32_t s_remote_received_count;
static uint32_t s_remote_last_ms;
static bool s_radio_ready;
static uint32_t s_attitude_last_ms; /* Timestamp of the latest successful six-axis solve. */
static uint16_t s_battery_cv;       /* System-task sample, rounded to 0.01 V; 0 unavailable. */
static uint32_t s_battery_last_ms;

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

void flight_system_task_entry(void *argument)
{
    uint32_t next_wake_tick;
    uint16_t led_count = 0U;
    uint8_t battery_count = 0U;

    (void)argument;

    next_wake_tick = osKernelGetTickCount();

    for (;;)
    {
        if (battery_count == 0U)
        {
            float voltage_v = flight_update_battery();
            uint16_t voltage_cv = isfinite(voltage_v)
                && voltage_v > 0.0f && voltage_v <= 655.35f
                ? (uint16_t)(voltage_v * 100.0f + 0.5f) : 0U;
            (void)osKernelLock();
            s_battery_cv = voltage_cv;
            s_battery_last_ms = HAL_GetTick();
            (void)osKernelUnlock();
        }
        battery_count++;
        if (battery_count >= FLIGHT_BATTERY_PERIOD_CYCLES)
        {
            battery_count = 0U;
        }

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
    sensors_status_t sensor_status = sensors_get_status();
    (void)osKernelLock();
    s_sensor_status = sensor_status;
    s_sensor_status_valid = true;
    (void)osKernelUnlock();
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
                (void)osKernelLock();
                s_latest_attitude_6axis = attitude;
                s_latest_attitude_6axis_valid = true;
                s_attitude_last_ms = HAL_GetTick();
                (void)osKernelUnlock();
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
    rcprotocol_control_t remote;
    uint32_t remote_received_count;
    uint32_t remote_last_ms;
    bool radio_ready;

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
        remote = s_latest_remote;
        remote_received_count = s_remote_received_count;
        remote_last_ms = s_remote_last_ms;
        radio_ready = s_radio_ready;
        (void)osKernelUnlock();

        /* USART2 telemetry is independent of the rotating USART1 text pages. */
        if (attitude_6axis_valid)
        {
            (void)debug_send_attitude(attitude_6axis.roll_deg,
                                      attitude_6axis.pitch_deg,
                                      attitude_6axis.yaw_deg);
        }

        if (FLIGHT_DEBUG_REMOTE_ENABLED)
        {
            bool connected = radio_ready && remote_received_count != 0U
                && (uint32_t)(HAL_GetTick() - remote_last_ms) < FLIGHT_LINK_TIMEOUT_MS;
            debug_print_remote(radio_ready, connected, remote_received_count, &remote);
        }
        else if (status_valid && !status_reported)
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
    rcprotocol_control_t control = {0};
    uint32_t received_count = 0U;
    uint32_t last_receive_ms = 0U;
    uint8_t status_sequence = 0U;
    bool have_control = false;
    bool radio_ready;

    (void)argument;

    radio_ready = si24r1_init(SI24R1_MODE_RX);
    (void)osKernelLock();
    s_radio_ready = radio_ready;
    (void)osKernelUnlock();
    next_wake_tick = osKernelGetTickCount();

    for (;;)
    {
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
                if (have_control && incoming.sequence == control.sequence
                    && (uint32_t)(now - last_receive_ms) < FLIGHT_LINK_TIMEOUT_MS)
                {
                    continue;
                }
                control = incoming;
                have_control = true;
                last_receive_ms = now;
                received_count++;
                (void)osKernelLock();
                s_latest_remote = control;
                s_remote_last_ms = last_receive_ms;
                s_remote_received_count = received_count;
                (void)osKernelUnlock();
            }

            if (have_control)
            {
                /* Copy published data only; no ADC/I2C acquisition in the link task. */
                attitude_t attitude;
                sensors_status_t sensor_status;
                bool attitude_valid;
                bool sensor_status_valid;
                uint32_t attitude_last_ms;
                uint32_t battery_last_ms;
                uint16_t battery_cv;
                (void)osKernelLock();
                attitude = s_latest_attitude_6axis;
                attitude_valid = s_latest_attitude_6axis_valid;
                attitude_last_ms = s_attitude_last_ms;
                sensor_status = s_sensor_status;
                sensor_status_valid = s_sensor_status_valid;
                battery_cv = s_battery_cv;
                battery_last_ms = s_battery_last_ms;
                (void)osKernelUnlock();

                uint32_t now = HAL_GetTick();
                attitude_valid = attitude_valid
                    && (uint32_t)(now - attitude_last_ms) < FLIGHT_ATTITUDE_TIMEOUT_MS;
                rcprotocol_status_t status = {
                    .sequence = status_sequence,
                    .roll_cdeg = RCPROTOCOL_INVALID_I16,
                    .pitch_cdeg = RCPROTOCOL_INVALID_I16,
                    .yaw_cdeg = RCPROTOCOL_INVALID_I16,
                    .throttle = control.throttle,
                    .height_cm = RCPROTOCOL_INVALID_I16,
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
                    || !sensor_status_valid || !sensor_status.imu)
                {
                    status.sensor_errors |= RCPROTOCOL_SENSOR_ERROR_IMU;
                }
                /* MAG/BARO bits report configured onboard sensor initialization.
                 * Unconnected FLOW/RANGE modules are not marked as failed.
                 */
                if (sensor_status_valid && !sensor_status.magnetometer)
                {
                    status.sensor_errors |= RCPROTOCOL_SENSOR_ERROR_MAG;
                }
                if (sensor_status_valid && !sensor_status.barometer)
                {
                    status.sensor_errors |= RCPROTOCOL_SENSOR_ERROR_BARO;
                }
                /* Arming/mode control and height estimation remain unconnected.
                 * No armed flag is asserted in the current motor-free bench loop.
                 * ACK data belongs to a future packet, not a command confirmation.
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
            s_radio_ready = radio_ready;
            (void)osKernelUnlock();
        }

        next_wake_tick += FLIGHT_LINK_TASK_PERIOD_TICKS;
        uint32_t now_tick = osKernelGetTickCount();
        if ((int32_t)(next_wake_tick - now_tick) <= 0)
        {
            next_wake_tick = now_tick + FLIGHT_LINK_TASK_PERIOD_TICKS;
        }
        osDelayUntil(next_wake_tick);
    }
}
