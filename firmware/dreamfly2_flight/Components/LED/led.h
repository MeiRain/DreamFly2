/**
 * @file led.h
 * @brief Public interface for the on-board status LED driver.
 * @author 梅雨
 * @date 2026-10-08
 */

#ifndef DREAMFLY2_COMPONENTS_LED_H
#define DREAMFLY2_COMPONENTS_LED_H

/**
 * @brief Set the status LED to its default off state.
 *
 * @note The GPIO clock and output mode must already be configured by CubeMX.
 */
void led_init(void);

/** @brief Turn on the status LED. */
void led_on(void);

/** @brief Turn off the status LED. */
void led_off(void);

/** @brief Toggle the current status LED output level. */
void led_toggle(void);

#endif /* DREAMFLY2_COMPONENTS_LED_H */
