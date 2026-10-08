/**
 * @file debug.h
 * @brief Debug data output interface.
 * @author 梅雨
 * @date 2026-10-08
 */

#ifndef DREAMFLY2_APPLICATION_DEBUG_H
#define DREAMFLY2_APPLICATION_DEBUG_H

/** @brief Format and send one debug message through USART1 DMA. */
void debug_printf(const char *format, ...);

#endif /* DREAMFLY2_APPLICATION_DEBUG_H */
