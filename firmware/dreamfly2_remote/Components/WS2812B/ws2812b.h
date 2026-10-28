/**
 * @file ws2812b.h
 * @brief Single remote-controller RGB LED, independent of UI and radio state.
 * @author 梅雨
 * @date 2026-10-26
 */

#ifndef DREAMFLY2_REMOTE_COMPONENTS_WS2812B_H
#define DREAMFLY2_REMOTE_COMPONENTS_WS2812B_H

#include <stdint.h>

#define WS2812B_LED_COUNT 1U

/**
 * @brief Clear buffered colors and hold the PWM compare value at zero.
 * @note Call once after CubeMX GPIO/TIM3/DMA initialization, not during a transfer.
 *       PB4 = TIM3_CH1 partial remap, JTAG disabled; 72 MHz TIM3, PSC=0, ARR=89,
 *       PWM1/high polarity. DMA1 Channel6: Normal, Memory-to-Peripheral,
 *       Word/Word, memory increment, IRQ enabled. This channel is exclusive to RGB;
 *       OLED I2C1 TX must not use the same DMA channel.
 *       Call all driver APIs from one task. No GPIO configuration is changed.
 */
void ws2812b_init(void);

/**
 * @brief Buffer one pixel; call ws2812b_show() to send it.
 * @param index Zero-based pixel index; out-of-range values are ignored.
 * @param red Red intensity, 0..255.
 * @param green Green intensity, 0..255.
 * @param blue Blue intensity, 0..255.
 */
void ws2812b_set_pixel(uint8_t index, uint8_t red, uint8_t green, uint8_t blue);

/** @brief Buffer the same RGB values for all pixels; each channel accepts 0..255. */
void ws2812b_set_all(uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief Encode GRB and start asynchronous PWM/DMA, with 250 us low before/after.
 * @note Busy requests are skipped. Buffer remains unchanged until completion.
 *       One LED uses a 1696-byte static DMA buffer; the frame takes about 530 us.
 *       Start errors and DMA errors release the busy state for a later retry;
 *       no transmission result or queue is provided.
 */
void ws2812b_show(void);

/**
 * @brief Advance one smooth color-gradient step and request transmission.
 * @note Call about every 20 ms from the owning task. Speed depends on call period;
 *       brightness is limited to 8/255. Busy calls do not advance the phase.
 */
void ws2812b_gradient(void);

/* Color shortcuts use approximately 8/255 brightness and transmit immediately. */
/** @brief Show red. */
void ws2812b_all_red(void);
/** @brief Show orange. */
void ws2812b_all_orange(void);
/** @brief Show yellow. */
void ws2812b_all_yellow(void);
/** @brief Show green. */
void ws2812b_all_green(void);
/** @brief Show cyan. */
void ws2812b_all_cyan(void);
/** @brief Show blue. */
void ws2812b_all_blue(void);
/** @brief Show violet. */
void ws2812b_all_violet(void);
/** @brief Show white. */
void ws2812b_all_white(void);
/** @brief Clear all pixels and transmit the off frame. */
void ws2812b_all_off(void);

#endif /* DREAMFLY2_REMOTE_COMPONENTS_WS2812B_H */
