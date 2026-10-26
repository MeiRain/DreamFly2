/**
 * @file button.h
 * @brief Six independent keys with one scan-result interface.
 * @author 梅雨
 * @date 2026-10-24
 */

#ifndef DREAMFLY2_REMOTE_COMPONENTS_BUTTON_H
#define DREAMFLY2_REMOTE_COMPONENTS_BUTTON_H

#include <stdint.h>

/* KEY1..KEY6 map to SW2, SW4, SW5, SW6, SW7, SW8 on PB10..PB15.
 * The same bit identifies a key in each result field; multiple bits may be set.
 */
#define KEY_NONE 0x00U
#define KEY1     (1U << 0)
#define KEY2     (1U << 1)
#define KEY3     (1U << 2)
#define KEY4     (1U << 3)
#define KEY5     (1U << 4)
#define KEY6     (1U << 5)

typedef struct
{
    uint8_t pressed;       /**< Debounced held-key bits, suitable for RC packets. */
    uint8_t short_pressed; /**< Short-release event bits from this scan only. */
    uint8_t long_pressed;  /**< One-shot long-press event bits from this scan only. */
} button_result_t;

/**
 * @brief Reset all six key states and timers.
 * @note Call after MX_GPIO_Init(); inputs are active-low with external pull-ups.
 *       GPIO configuration is managed by CubeMX.
 */
void button_init(void);

/**
 * @brief Scan all six keys independently and return held/short/long bitmaps.
 * @param[out] result All fields are overwritten on every scan; NULL is ignored
 *                    without changing internal state.
 * @note Call every 5..10 ms from one task. Debounce is 20 ms for both edges.
 *       Short events occur on debounced release. Long events occur once after
 *       1500 ms from debounced press; release then generates no short event.
 *       Test fields using bitwise AND, for example result->short_pressed & KEY1.
 *       Multiple keys may report events in the same scan. Events are not queued:
 *       handle/copy them before the next scan. Only pressed is a persistent state.
 *       No double-click, combination-key action or long-press repeat is included.
 */
void button_scan(button_result_t *result);

#endif /* DREAMFLY2_REMOTE_COMPONENTS_BUTTON_H */
