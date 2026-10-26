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
    /* Timer compare counts, clamped to 0..900; not a percentage or an RPM value.
     * Channels: M1 TIM4_CH1, M2 TIM4_CH2, M3 TIM3_CH3, M4 TIM3_CH4.
     */
    int16_t pwm1;
    int16_t pwm2;
    int16_t pwm3;
    int16_t pwm4;
} motor_t;

/**
 * @brief Start the four motor PWM channels with zero output.
 * @note TIM3, TIM4 and their GPIOs must be initialized first; HAL start errors
 *       are not reported by this interface.
 */
void motor_init(void);

/**
 * @brief Limit and write the four motor PWM outputs.
 * @param motor Requested compare counts; NULL leaves all outputs unchanged.
 */
void motor_control(const motor_t *motor);

#endif /* DREAMFLY2_COMPONENTS_MOTOR_H */
