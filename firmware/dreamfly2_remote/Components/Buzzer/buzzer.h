/**
 * @file buzzer.h
 * @brief On-board buzzer interface.
 * @author 梅雨
 * @date 2026-10-24
 */

#ifndef DREAMFLY2_REMOTE_COMPONENTS_BUZZER_H
#define DREAMFLY2_REMOTE_COMPONENTS_BUZZER_H

/**
 * @brief Turn off the buzzer.
 * @note Call after MX_GPIO_Init(); GPIO configuration is managed by CubeMX.
 */
void buzzer_init(void);

/**
 * @brief Set the buzzer control output high.
 * @note On/off control for an active buzzer; no audio-frequency PWM is generated.
 */
void buzzer_on(void);

/** @brief Set the buzzer control output low. */
void buzzer_off(void);

#endif /* DREAMFLY2_REMOTE_COMPONENTS_BUZZER_H */
