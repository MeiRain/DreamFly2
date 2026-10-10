/**
 * @file ws2812b.h
 * @brief Public interface for the WS2812B RGB LED driver.
 * @author 梅雨
 * @date 2026-10-09
 */

#ifndef DREAMFLY2_COMPONENTS_WS2812B_H
#define DREAMFLY2_COMPONENTS_WS2812B_H

#include <stdint.h>

#define WS2812B_LED_COUNT 4U

/**
 * @brief Clear the local pixel data and keep the PWM output low.
 *
 * TIM2 channel 3 and its DMA must already be initialized by CubeMX.
 */
void ws2812b_init(void);

/** @brief Set one pixel in the local color buffer. */
void ws2812b_set_pixel(uint8_t index, uint8_t red, uint8_t green, uint8_t blue);

/** @brief Set all pixels in the local color buffer. */
void ws2812b_set_all(uint8_t red, uint8_t green, uint8_t blue);

/** @brief Send the local color buffer using PWM and DMA. */
void ws2812b_show(void);

/** @brief Advance the four-pixel running-light effect by one frame. */
void ws2812b_run(void);

/** @brief Advance the smooth RGB gradient effect by one frame. */
void ws2812b_gradient(void);

void ws2812b_all_red(void);
void ws2812b_all_orange(void);
void ws2812b_all_yellow(void);
void ws2812b_all_green(void);
void ws2812b_all_cyan(void);
void ws2812b_all_blue(void);
void ws2812b_all_violet(void);
void ws2812b_all_white(void);
void ws2812b_all_off(void);

#endif /* DREAMFLY2_COMPONENTS_WS2812B_H */
