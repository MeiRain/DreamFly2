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
 * @note TIM2 channel 3 and its DMA must already be initialized by CubeMX.
 *       Call once before use, not during an active transmission. Call driver
 *       functions from one task; internal buffers are not protected by a mutex.
 */
void ws2812b_init(void);

/**
 * @brief Set one pixel in the local buffer; call ws2812b_show() to transmit.
 * @param index Zero-based LED index; out-of-range indices are ignored.
 * @param red Red intensity, 0..255.
 * @param green Green intensity, 0..255.
 * @param blue Blue intensity, 0..255.
 */
void ws2812b_set_pixel(uint8_t index, uint8_t red, uint8_t green, uint8_t blue);

/** @brief Set all buffered pixels to RGB intensities 0..255; does not transmit. */
void ws2812b_set_all(uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief Start asynchronous GRB transmission followed by the low reset interval.
 * @note Returns immediately; requests while busy are skipped. DMA start failures
 *       are not reported. Completion releases the internal PWM buffer.
 */
void ws2812b_show(void);

/** @brief Advance and send one running-light frame; call interval sets animation speed. */
void ws2812b_run(void);

/** @brief Advance and send one shared-color gradient frame; call interval sets speed. */
void ws2812b_gradient(void);

/** @brief Set all pixels red and request transmission. */
void ws2812b_all_red(void);
/** @brief Set all pixels orange and request transmission. */
void ws2812b_all_orange(void);
/** @brief Set all pixels yellow and request transmission. */
void ws2812b_all_yellow(void);
/** @brief Set all pixels green and request transmission. */
void ws2812b_all_green(void);
/** @brief Set all pixels cyan and request transmission. */
void ws2812b_all_cyan(void);
/** @brief Set all pixels blue and request transmission. */
void ws2812b_all_blue(void);
/** @brief Set all pixels violet and request transmission. */
void ws2812b_all_violet(void);
/** @brief Set all pixels white and request transmission. */
void ws2812b_all_white(void);
/** @brief Clear all pixels and request transmission. */
void ws2812b_all_off(void);

#endif /* DREAMFLY2_COMPONENTS_WS2812B_H */
