/**
 * @file rtos.c
 * @brief FreeRTOS task implementations for the flight controller.
 * @author 梅雨
 * @date 2026-10-08
 */

#include "rtos.h"

#include <stdint.h>

#include "cmsis_os2.h"

#include "debug.h"
#include "flight.h"
#include "led.h"

#define FLIGHT_SYSTEM_TASK_PERIOD_TICKS 50U
#define FLIGHT_SYSTEM_LED_TOGGLE_COUNT 10U

void flight_system_task_entry(void *argument)
{
    uint32_t next_wake_tick;
    uint8_t led_toggle_count = 0U;
    float battery_voltage_v;

    (void)argument;

    led_init();
    next_wake_tick = osKernelGetTickCount();

    for (;;)
    {
        battery_voltage_v = flight_update_battery();

        ++led_toggle_count;
        if (led_toggle_count >= FLIGHT_SYSTEM_LED_TOGGLE_COUNT)
        {
            led_toggle_count = 0U;
            led_toggle();
            debug_printf(
                "battery=%lu.%03lu V\r\n",
                (unsigned long)battery_voltage_v,
                (unsigned long)(battery_voltage_v * 1000.0f) % 1000U);
        }

        next_wake_tick += FLIGHT_SYSTEM_TASK_PERIOD_TICKS;
        (void)osDelayUntil(next_wake_tick);
    }
}
