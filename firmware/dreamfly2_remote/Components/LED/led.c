/**
 * @file led.c
 * @brief Active-low status LED on PA8.
 * @author 梅雨
 * @date 2026-10-24
 */

#include "led.h"

#include "stm32f1xx_hal.h"

#define LED_GPIO_PORT GPIOA
#define LED_GPIO_PIN  GPIO_PIN_8

void led_init(void)
{
    led_off();
}

void led_on(void)
{
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_RESET);
}

void led_off(void)
{
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_SET);
}

void led_toggle(void)
{
    HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_GPIO_PIN);
}
