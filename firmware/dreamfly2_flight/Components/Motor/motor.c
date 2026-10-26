/**
 * @file motor.c
 * @brief Four-channel brushed motor driver using TIM3 and TIM4 PWM.
 * @author 梅雨
 * @date 2026-10-10
 */

#include "motor.h"

#include "tim.h"

#define MOTOR_PWM_MIN 0
#define MOTOR_PWM_MAX 900

/** @brief Clamp a requested timer compare value to the configured motor limits. */
static uint16_t motor_limit(int16_t pwm)
{
    if (pwm < MOTOR_PWM_MIN)
    {
        return MOTOR_PWM_MIN;
    }

    if (pwm > MOTOR_PWM_MAX)
    {
        return MOTOR_PWM_MAX;
    }

    return (uint16_t)pwm;
}

void motor_init(void)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0U);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0U);

    (void)HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

void motor_control(const motor_t *motor)
{
    if (motor == 0)
    {
        return;
    }

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, motor_limit(motor->pwm1));
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, motor_limit(motor->pwm2));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, motor_limit(motor->pwm3));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, motor_limit(motor->pwm4));
}
