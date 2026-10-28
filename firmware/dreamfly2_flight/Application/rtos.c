/**
 * @file rtos.c
 * @brief FreeRTOS task implementations for the flight controller.
 * @author 梅雨
 * @date 2026-10-08
 */

#include "rtos.h"

#include <stddef.h>
#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"
#include "flight.h"

/* Periods are kernel ticks; application algorithms assume a 1 kHz tick. */
#define FLIGHT_CONTROL_TASK_PERIOD_TICKS 2U
#define FLIGHT_SYSTEM_TASK_PERIOD_TICKS 10U
#define FLIGHT_SENSOR_TASK_PERIOD_TICKS 2U
#define FLIGHT_DEBUG_TASK_PERIOD_TICKS 20U
#define FLIGHT_LINK_TASK_PERIOD_TICKS 5U
#define FLIGHT_RADIO_IRQ_FLAG 0x01U

static osThreadId_t volatile s_link_thread;

/** @brief Notify the radio task; keep SPI and protocol work outside the ISR. */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_2 && s_link_thread != NULL)
    {
        (void)osThreadFlagsSet(s_link_thread, FLIGHT_RADIO_IRQ_FLAG);
    }
}

/** @brief Yield until the control task has completed centralized startup. */
static void flight_wait_for_init(void)
{
    while (!flight_is_initialized())
    {
        osDelay(1U);
    }
}

void flight_system_task_entry(void *argument)
{
    (void)argument;
    uint32_t next_wake_tick = osKernelGetTickCount();

    for (;;)
    {
        flight_process_system();
        next_wake_tick += FLIGHT_SYSTEM_TASK_PERIOD_TICKS;
        osDelayUntil(next_wake_tick);
    }
}

void flight_control_task_entry(void *argument)
{
    (void)argument;
    flight_init();
    uint32_t next_wake_tick = osKernelGetTickCount();

    for (;;)
    {
        flight_process_control();
        next_wake_tick += FLIGHT_CONTROL_TASK_PERIOD_TICKS;
        osDelayUntil(next_wake_tick);
    }
}

void flight_sensor_task_entry(void *argument)
{
    (void)argument;
    flight_wait_for_init();
    uint32_t next_wake_tick = osKernelGetTickCount();

    for (;;)
    {
        flight_process_sensors();
        next_wake_tick += FLIGHT_SENSOR_TASK_PERIOD_TICKS;
        osDelayUntil(next_wake_tick);
    }
}

void flight_debug_task_entry(void *argument)
{
    (void)argument;
    flight_wait_for_init();
    uint32_t next_wake_tick = osKernelGetTickCount();

    for (;;)
    {
        flight_process_debug();
        next_wake_tick += FLIGHT_DEBUG_TASK_PERIOD_TICKS;
        osDelayUntil(next_wake_tick);
    }
}

void flight_link_task_entry(void *argument)
{
    (void)argument;
    s_link_thread = osThreadGetId();
    flight_wait_for_init();

    for (;;)
    {
        flight_process_link();
        /* Pending IRQ flags survive until consumed. Timeout polling also handles
         * a missed falling edge or an IRQ line that remains low.
         */
        (void)osThreadFlagsWait(FLIGHT_RADIO_IRQ_FLAG, osFlagsWaitAny,
                                FLIGHT_LINK_TASK_PERIOD_TICKS);
    }
}
