/**
 * @file led.h
 * @brief On-board status LED interface.
 * @author 梅雨
 * @date 2026-10-24
 */

#ifndef DREAMFLY2_REMOTE_COMPONENTS_LED_H
#define DREAMFLY2_REMOTE_COMPONENTS_LED_H

/**
 * @brief Turn off the status LED.
 * @note Call after MX_GPIO_Init(); GPIO configuration is managed by CubeMX.
 */
void led_init(void);

/** @brief Turn on the active-low status LED. */
void led_on(void);

/** @brief Turn off the status LED. */
void led_off(void);

/** @brief Toggle the status LED output. */
void led_toggle(void);

#endif /* DREAMFLY2_REMOTE_COMPONENTS_LED_H */
