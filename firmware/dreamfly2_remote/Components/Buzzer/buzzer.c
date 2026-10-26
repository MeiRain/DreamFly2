/**
 * @file buzzer.c
 * @brief Active-high buzzer control on PB5.
 * @author 梅雨
 * @date 2026-10-24
 */

#include "buzzer.h"

#include "stm32f1xx_hal.h"

#define BUZZER_GPIO_PORT GPIOB
#define BUZZER_GPIO_PIN  GPIO_PIN_5

void buzzer_init(void)
{
    buzzer_off();
}

void buzzer_on(void)
{
    HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN, GPIO_PIN_SET);
}

void buzzer_off(void)
{
    HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN, GPIO_PIN_RESET);
}
