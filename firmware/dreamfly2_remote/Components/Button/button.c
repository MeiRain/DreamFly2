/**
 * @file button.c
 * @brief Independent active-low key scanning on PB10..PB15.
 * @author 梅雨
 * @date 2026-10-24
 */

#include "button.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "stm32f1xx_hal.h"

#define BUTTON_COUNT       6U
#define KEY_DEBOUNCE_MS     20U
#define KEY_LONG_PRESS_MS   1500U

typedef struct
{
    uint32_t change_tick; /**< Last raw input transition, HAL milliseconds. */
    uint32_t press_tick;  /**< Debounced press start, HAL milliseconds. */
    bool candidate_pressed;
    bool stable_pressed;
    bool long_sent;
} button_state_t;

/* Each key owns its debounce timer, hold timer and long-event latch. */
static button_state_t s_buttons[BUTTON_COUNT];

/** @brief Read all six active-low inputs into bits 0..5. */
static uint8_t button_read_mask(void)
{
    uint8_t keys = KEY_NONE;
    for (uint8_t i = 0U; i < BUTTON_COUNT; i++)
    {
        if (HAL_GPIO_ReadPin(GPIOB, (uint16_t)(GPIO_PIN_10 << i)) == GPIO_PIN_RESET)
        {
            keys |= (uint8_t)(KEY1 << i);
        }
    }
    return keys;
}

void button_init(void)
{
    memset(s_buttons, 0, sizeof(s_buttons));
}

void button_scan(button_result_t *result)
{
    if (result == NULL)
    {
        return;
    }

    uint32_t now = HAL_GetTick();
    uint8_t keys = button_read_mask();
    button_result_t sample = {0};

    for (uint8_t i = 0U; i < BUTTON_COUNT; i++)
    {
        button_state_t *state = &s_buttons[i];
        uint8_t key = (uint8_t)(KEY1 << i);
        bool pressed = (keys & key) != 0U;

        if (pressed != state->candidate_pressed)
        {
            state->candidate_pressed = pressed;
            state->change_tick = now;
        }

        /* Both press and release must remain stable for at least 20 ms. */
        if ((state->candidate_pressed != state->stable_pressed)
            && ((uint32_t)(now - state->change_tick) >= KEY_DEBOUNCE_MS))
        {
            /* A completed long press must not also produce a short release. */
            if (!state->candidate_pressed && !state->long_sent)
            {
                sample.short_pressed |= key;
            }

            state->stable_pressed = state->candidate_pressed;
            state->press_tick = now;
            state->long_sent = false;
        }

        /* A raw release awaiting debounce must not generate a new long event. */
        if (state->stable_pressed && state->candidate_pressed
            && !state->long_sent
            && ((uint32_t)(now - state->press_tick) >= KEY_LONG_PRESS_MS))
        {
            sample.long_pressed |= key;
            state->long_sent = true;
        }

        if (state->stable_pressed)
        {
            sample.pressed |= key;
        }
    }

    *result = sample;
}
