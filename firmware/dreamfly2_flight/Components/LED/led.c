/**
 * @file led.c
 * @brief On-board status LED driver.
 * @author 梅雨
 * @date 2026-10-08
 */

#include "led.h"

#include "stm32f4xx_hal.h"

#define LED_GPIO_PORT GPIOC
#define LED_GPIO_PIN GPIO_PIN_13

/* The status LED is connected in active-low configuration. */
#define LED_ON_LEVEL GPIO_PIN_RESET
#define LED_OFF_LEVEL GPIO_PIN_SET

void led_init(void)
{
    led_off();
}

void led_on(void)
{
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, LED_ON_LEVEL);
}

void led_off(void)
{
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, LED_OFF_LEVEL);
}

void led_toggle(void)
{
    HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_GPIO_PIN);
}
