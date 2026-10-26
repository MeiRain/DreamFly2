/**
 * @file debug.c
 * @brief Text debug on USART1 and binary telemetry on USART2 DMA.
 * @author 梅雨
 * @date 2026-10-08
 */

#include "debug.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "usart.h"
#include "anoprotocol.h"

#define DEBUG_TX_BUFFER_SIZE 128U

static char s_debug_tx_buffer[DEBUG_TX_BUFFER_SIZE];
static uint8_t s_telemetry_tx_buffer[DEBUG_TX_BUFFER_SIZE];

void debug_print_remote(bool radio_ready, bool connected, uint32_t received_count,
                        const rcprotocol_control_t *control)
{
    if (!radio_ready)
    {
        debug_printf("rc: radio=ERROR\r\n");
    }
    else if (!connected || control == NULL)
    {
        debug_printf("rc: %s rx=%lu\r\n",
                     received_count == 0U ? "WAIT" : "LOST",
                     (unsigned long)received_count);
    }
    else
    {
        debug_printf("rc: OK rx=%lu seq=%u R=%u P=%u Y=%u T=%u K=%02X\r\n",
                     (unsigned long)received_count, (unsigned)control->sequence,
                     (unsigned)control->roll, (unsigned)control->pitch,
                     (unsigned)control->yaw, (unsigned)control->throttle,
                     (unsigned)control->keys);
    }
}

bool debug_send_attitude(float roll_deg, float pitch_deg, float yaw_deg)
{
    uint8_t buffer[15];
    uint16_t length;

    if (!isfinite(roll_deg) || !isfinite(pitch_deg) || !isfinite(yaw_deg)
        || (roll_deg < -327.68f) || (roll_deg > 327.67f)
        || (pitch_deg < -327.68f) || (pitch_deg > 327.67f)
        || (yaw_deg < -327.68f) || (yaw_deg > 327.67f))
    {
        return false;
    }

    /* Match the legacy ANO display axes without changing the flight frame. */
    length = anoprotocol_pack_attitude((int16_t)(roll_deg * 100.0f),
                               (int16_t)(-pitch_deg * 100.0f),
                               (int16_t)(-yaw_deg * 100.0f),
                               1U, buffer, sizeof(buffer));
    return debug_send(buffer, length);
}

bool debug_send(const uint8_t *data, uint16_t length)
{
    if ((data == NULL) || (length == 0U)
        || (length > sizeof(s_telemetry_tx_buffer))
        || (huart2.gState != HAL_UART_STATE_READY))
    {
        return false;
    }

    /* Keep a private copy alive and unchanged until DMA transmission completes. */
    memcpy(s_telemetry_tx_buffer, data, length);
    return HAL_UART_Transmit_DMA(&huart2, s_telemetry_tx_buffer, length) == HAL_OK;
}

void debug_printf(const char *format, ...)
{
    va_list arguments;
    int length;

    if ((format == NULL) || (huart1.gState != HAL_UART_STATE_READY))
    {
        return;
    }

    va_start(arguments, format);
    length = vsnprintf(s_debug_tx_buffer,
                       sizeof(s_debug_tx_buffer),
                       format,
                       arguments);
    va_end(arguments);

    if (length <= 0)
    {
        return;
    }

    if (length >= (int)sizeof(s_debug_tx_buffer))
    {
        length = (int)sizeof(s_debug_tx_buffer) - 1;
    }

    (void)HAL_UART_Transmit_DMA(&huart1,
                                (uint8_t *)s_debug_tx_buffer,
                                (uint16_t)length);
}
