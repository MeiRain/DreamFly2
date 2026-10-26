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

/* CubeMX currently configures a 1 kHz kernel tick: one tick equals one ms. */
#define RTOS_CONTROL_PERIOD_TICKS 5U
#define RTOS_DISPLAY_PERIOD_TICKS 100U

/* Created once by the higher-priority control task before display can run. */
static osMessageQueueId_t s_display_queue;

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

    s_display_queue = osMessageQueueNew(1U, sizeof(data), NULL);
    button_init();
    (void)remote_init();
    uint32_t next_tick = osKernelGetTickCount();

    for (;;)
    {
        button_result_t buttons;
        button_scan(&buttons);
        remote_handle_buttons(&buttons);
        remote_process();
        remote_get_state(&data);

        rtos_publish_display(&data);
        rtos_delay_until(&next_tick, RTOS_CONTROL_PERIOD_TICKS);
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
