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

#include "flight.h"

/** @brief Start UART2 normal RX DMA with IDLE events; call once from flight_init(). */
bool debug_init(void);

/**
 * @brief Read queued UART2 bytes without waiting; restart RX after errors when possible.
 * @return Byte count, 0 if empty, -1 after a transport error/overflow (queue flushed).
 * @note Only the debug task consumes bytes. On -1 reset the protocol accumulator.
 *       ISR callbacks only queue bytes; no parsing, parameter changes or Flash work.
 */
int16_t debug_receive(uint8_t *data, uint16_t capacity);

/**
 * @brief Send one USART1 rate/motor-preview line and USART2 attitude from a snapshot.
 * @param data Stable application snapshot; NULL is ignored.
 * @note Call from the sole debug task every 20 ticks after flight_init().
 *       G is XYZ angular rate (deg/s); U is R/P/Y PID correction (PWM counts);
 *       M is mixed candidate PWM; O is the last M1..M4 command sent to the driver,
 *       not measured RPM. run=1 means the gated rate bench path is driving motors.
 *       Invalid candidates use dashes; unavailable rate data reports readiness
 *       flags and the last output command.
 *       No sensor acquisition, flight-state changes or motor writes occur here.
 */
void debug_report(const flight_data_t *data);

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
