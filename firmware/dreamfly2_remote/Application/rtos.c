/**
 * @file rtos.c
 * @brief Remote input/radio processing and OLED verification tasks.
 * @author 梅雨
 * @date 2026-10-26
 */

#include "rtos.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "button.h"
#include "cmsis_os2.h"
#include "oled.h"
#include "remote.h"
#include "stm32f1xx_hal.h"

/* CubeMX currently configures a 1 kHz kernel tick: one tick equals one ms. */
#define RTOS_CONTROL_PERIOD_TICKS 5U
#define RTOS_DISPLAY_PERIOD_TICKS 100U
#define RTOS_RADIO_IRQ_FLAG 0x01U

/* Created once by the higher-priority control task before display can run. */
static osMessageQueueId_t s_display_queue;
/* Bound before radio initialization, after the scheduler has started. */
static osThreadId_t volatile s_control_thread;

/** @brief Forward PB3 radio IRQ to its owner; never access SPI in the ISR. */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_3 && s_control_thread != NULL)
    {
        (void)osThreadFlagsSet(s_control_thread, RTOS_RADIO_IRQ_FLAG);
    }
}

/** @brief Keep an absolute period; skip missed deadlines instead of busy catch-up. */
static void rtos_delay_until(uint32_t *next_tick, uint32_t period_ticks)
{
    *next_tick += period_ticks;
    uint32_t now = osKernelGetTickCount();
    if ((int32_t)(*next_tick - now) <= 0)
    {
        *next_tick = now + period_ticks;
    }
    (void)osDelayUntil(*next_tick);
}

/** @brief Publish the latest copy without waiting; discard an older queued copy. */
static void rtos_publish_display(const remote_state_t *data)
{
    if (s_display_queue == NULL)
    {
        return;
    }

    if (osMessageQueuePut(s_display_queue, data, 0U, 0U) == osErrorResource)
    {
        remote_state_t discarded;
        (void)osMessageQueueGet(s_display_queue, &discarded, NULL, 0U);
        (void)osMessageQueuePut(s_display_queue, data, 0U, 0U);
    }
}

void remote_control_task_entry(void *argument)
{
    (void)argument;
    remote_state_t data = {0};

    s_control_thread = osThreadGetId();
    s_display_queue = osMessageQueueNew(1U, sizeof(data), NULL);
    button_init();
    (void)remote_init();
    uint32_t next_tick = osKernelGetTickCount();

    bool radio_event = false;
    for (;;)
    {
        uint32_t now = osKernelGetTickCount();
        if ((int32_t)(now - next_tick) >= 0)
        {
            button_result_t buttons;
            button_scan(&buttons);
            remote_handle_buttons(&buttons);
            remote_process();
            remote_get_state(&data);
            rtos_publish_display(&data);

            next_tick += RTOS_CONTROL_PERIOD_TICKS;
            now = osKernelGetTickCount();
            if ((int32_t)(next_tick - now) <= 0)
            {
                next_tick = now + RTOS_CONTROL_PERIOD_TICKS;
            }
        }
        else if (radio_event)
        {
            /* Extra IRQ wakeups service only radio data, preserving the 5 ms
             * button/input/calibration cadence and the 10 ms TX schedule.
             */
            remote_process_radio();
            remote_get_state(&data);
            rtos_publish_display(&data);
        }

        now = osKernelGetTickCount();
        uint32_t timeout = (int32_t)(next_tick - now) > 0 ? next_tick - now : 1U;
        uint32_t flags = osThreadFlagsWait(RTOS_RADIO_IRQ_FLAG, osFlagsWaitAny, timeout);
        radio_event = (flags & osFlagsError) == 0U
            && (flags & RTOS_RADIO_IRQ_FLAG) != 0U;
    }
}

void remote_display_task_entry(void *argument)
{
    (void)argument;
    remote_state_t data = {0};
    bool oled_ready = false;
    uint32_t next_tick = osKernelGetTickCount();

    for (;;)
    {
        if (!oled_ready)
        {
            oled_ready = oled_init();
        }

        if (s_display_queue != NULL)
        {
            (void)osMessageQueueGet(s_display_queue, &data, NULL, 0U);
        }

        if (oled_ready)
        {
            remote_draw_page(&data);
            if (s_display_queue == NULL)
            {
                oled_clear_area(0, 48, OLED_WIDTH, 16);
                oled_show_string(0, 48, "QUEUE ERROR", OLED_8X16);
            }
            oled_ready = oled_update();
        }

        rtos_delay_until(&next_tick, RTOS_DISPLAY_PERIOD_TICKS);
    }
}
