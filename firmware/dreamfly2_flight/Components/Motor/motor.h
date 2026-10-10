/**
 * @file motor.h
 * @brief Four-channel brushed motor driver interface.
 * @author 梅雨
 * @date 2026-10-10
 */

#ifndef DREAMFLY2_COMPONENTS_MOTOR_H
#define DREAMFLY2_COMPONENTS_MOTOR_H

#include <stdint.h>

typedef struct
{
    int16_t pwm1;
    int16_t pwm2;
    int16_t pwm3;
    int16_t pwm4;
} motor_t;

/** @brief Start the four motor PWM channels with zero output. */
void motor_init(void);

/** @brief Limit and write the four motor PWM outputs. */
void motor_control(const motor_t *motor);

#endif /* DREAMFLY2_COMPONENTS_MOTOR_H */
