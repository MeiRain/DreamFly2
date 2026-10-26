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

#include "rcprotocol.h"

/**
 * @brief Print a received RC snapshot through USART1 DMA.
 * @param radio_ready Whether the flight radio is initialized without a transport fault.
 * @param connected Whether a valid control frame arrived within the last 300 ms.
 * @param received_count Total accepted control frames since task startup.
 * @param control Last accepted channels; required only when connected is true.
 * @note Call every 100 ms from the same task as debug_printf(). Stale channels
 *       are not printed as live input. This function does not receive UART data.
 */
void debug_print_remote(bool radio_ready, bool connected, uint32_t received_count,
                        const rcprotocol_control_t *control);

/**
 * @brief Format and send one debug message through USART1 DMA.
 * @param format printf-style format string and following arguments.
 * @note Call from one task after USART1/DMA initialization. NULL or busy requests
 *       are dropped; text is truncated to 127 bytes. Transmission errors are not reported.
 */
void debug_printf(const char *format, ...);

/**
 * @brief Copy binary data and start USART2 DMA transmission.
 * @param data Bytes to copy; must not be NULL.
 * @param length Byte count, 1..128.
 * @return true when started, false when busy, invalid or failed (no queue).
 * @note Call from one task only. The caller may reuse its buffer on return.
 *       Success does not confirm reception by the upper computer.
 */
bool debug_send(const uint8_t *data, uint16_t length);

/**
 * @brief Pack and send ANO attitude through USART2 DMA, negating Pitch and Yaw for display.
 * @param roll_deg Flight-frame roll in degrees.
 * @param pitch_deg Flight-frame pitch in degrees.
 * @param yaw_deg Flight-frame yaw in degrees.
 * @return false for nonfinite/out-of-range angles, busy UART or transmission failure.
 * @note Inputs must be within -327.68..327.67 degrees. Uses the same single-task
 *       transmission contract as debug_send().
 */
bool debug_send_attitude(float roll_deg, float pitch_deg, float yaw_deg);

#endif /* DREAMFLY2_APPLICATION_DEBUG_H */
