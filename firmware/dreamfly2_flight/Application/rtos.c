/**
 * @file rtos.c
 * @brief FreeRTOS task implementations for the flight controller.
 * @author 梅雨
 * @date 2026-10-08
 */

#include "rtos.h"

#include "cmsis_os2.h"

#include "flight.h"

#include "debug.h"

#define FLIGHT_CONTROL_TASK_PERIOD_TICKS 2U
#define FLIGHT_SYSTEM_TASK_PERIOD_TICKS 10U
#define FLIGHT_SENSOR_TASK_PERIOD_TICKS 10U
#define FLIGHT_DEBUG_TASK_PERIOD_TICKS 100U
#define FLIGHT_LINK_TASK_PERIOD_TICKS 5U
#define FLIGHT_LED_PERIOD_CYCLES 50U

static flight_imu_t s_latest_imu;
static bool s_latest_imu_valid;

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
    flight_imu_t imu;
    uint32_t next_wake_tick;
    bool imu_ok;

    (void)argument;

    imu_ok = flight_init();
    next_wake_tick = osKernelGetTickCount();

    for (;;)
    {
        if (imu_ok && flight_imu_update(&imu))
        {
            s_latest_imu = imu;
            s_latest_imu_valid = true;
        }

        next_wake_tick += FLIGHT_CONTROL_TASK_PERIOD_TICKS;
        osDelayUntil(next_wake_tick);
    }
}

void flight_sensor_task_entry(void *argument)
{
    uint32_t next_wake_tick;

    (void)argument;

    next_wake_tick = osKernelGetTickCount();

    for (;;)
    {
        next_wake_tick += FLIGHT_SENSOR_TASK_PERIOD_TICKS;
        osDelayUntil(next_wake_tick);
    }
}

void flight_debug_task_entry(void *argument)
{
    flight_imu_t imu;
    uint32_t next_wake_tick;
    bool imu_valid;

    (void)argument;

    next_wake_tick = osKernelGetTickCount();

    for (;;)
    {
        (void)osKernelLock();
        imu = s_latest_imu;
        imu_valid = s_latest_imu_valid;
        (void)osKernelUnlock();

        if (imu_valid)
        {
            debug_printf("acc=%.3f,%.3f,%.3f g gyro=%.2f,%.2f,%.2f dps\r\n",
                         imu.accel_x_g,
                         imu.accel_y_g,
                         imu.accel_z_g,
                         imu.gyro_x_dps,
                         imu.gyro_y_dps,
                         imu.gyro_z_dps);
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
