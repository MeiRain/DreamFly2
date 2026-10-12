/**
 * @file debug.h
 * @brief Debug data output interface.
 * @author 梅雨
 * @date 2026-10-08
 */

#ifndef DREAMFLY2_APPLICATION_DEBUG_H
#define DREAMFLY2_APPLICATION_DEBUG_H

#include <stdbool.h>
#include <stdint.h>

/** @brief Format and send one debug message through USART1 DMA. */
void debug_printf(const char *format, ...);

/**
 * @brief Copy binary data and start USART2 DMA transmission.
 * @return true when started, false when busy, invalid or failed (no queue).
 * @note Call from one task only. The caller may reuse its buffer on return.
 *       Success does not confirm reception by the upper computer.
 */
bool debug_send(const uint8_t *data, uint16_t length);

#endif /* DREAMFLY2_APPLICATION_DEBUG_H */
