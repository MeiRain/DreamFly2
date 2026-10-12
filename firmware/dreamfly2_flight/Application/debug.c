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

#include "usart.h"

#define DEBUG_TX_BUFFER_SIZE 128U

static char s_debug_tx_buffer[DEBUG_TX_BUFFER_SIZE];
static uint8_t s_telemetry_tx_buffer[DEBUG_TX_BUFFER_SIZE];

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
