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

#define DEBUG_TX_BUFFER_SIZE 192U

static char s_debug_tx_buffer[DEBUG_TX_BUFFER_SIZE];
static uint8_t s_telemetry_tx_buffer[DEBUG_TX_BUFFER_SIZE];

#define DEBUG_RX_DMA_SIZE 128U
#define DEBUG_RX_RING_SIZE 512U
static uint8_t s_rx_dma[DEBUG_RX_DMA_SIZE];
static uint8_t s_rx_ring[DEBUG_RX_RING_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static volatile bool s_rx_error; /* Shared with UART/DMA callbacks. */

/** @brief Arm the generated normal-mode RX DMA; ignore half-transfer events. */
static bool debug_start_receive(void)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, s_rx_dma, sizeof(s_rx_dma)) != HAL_OK)
    {
        return false;
    }
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    return true;
}

bool debug_init(void)
{
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_rx_error = false;
    return debug_start_receive();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != &huart2 || HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT)
    {
        return;
    }
    uint16_t free_bytes = (s_rx_tail + DEBUG_RX_RING_SIZE - s_rx_head - 1U) % DEBUG_RX_RING_SIZE;
    if (size > DEBUG_RX_DMA_SIZE || size > free_bytes)
    {
        s_rx_error = true;
    }
    else
    {
        for (uint16_t i = 0U; i < size; i++)
        {
            s_rx_ring[s_rx_head] = s_rx_dma[i];
            s_rx_head = (s_rx_head + 1U) % DEBUG_RX_RING_SIZE;
        }
    }
    if (!debug_start_receive())
    {
        s_rx_error = true;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2)
    {
        s_rx_error = true;
    }
}

int16_t debug_receive(uint8_t *data, uint16_t capacity)
{
    if (data == NULL || capacity == 0U)
    {
        return 0;
    }
    if (huart2.RxState == HAL_UART_STATE_READY && !debug_start_receive())
    {
        s_rx_error = true;
    }

    /* Brief IRQ exclusion protects queue indices and error flushes; it does
     * not cover DMA restart, parsing, transmission or any Flash operation.
     */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (s_rx_error)
    {
        s_rx_tail = s_rx_head;
        s_rx_error = false;
        __set_PRIMASK(primask);
        return -1;
    }
    uint16_t count = 0U;
    while (count < capacity && s_rx_tail != s_rx_head)
    {
        data[count++] = s_rx_ring[s_rx_tail];
        s_rx_tail = (s_rx_tail + 1U) % DEBUG_RX_RING_SIZE;
    }
    __set_PRIMASK(primask);
    return (int16_t)count;
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

void debug_report(const flight_data_t *data)
{
    if (data == NULL)
    {
        return;
    }

    /* Keep USART2 attitude streaming independent of the USART1 bench text. */
    if (data->sensors.attitude_6axis_valid)
    {
        (void)debug_send_attitude(data->sensors.attitude_6axis.roll_deg,
                                  data->sensors.attitude_6axis.pitch_deg,
                                  data->sensors.attitude_6axis.yaw_deg);
    }

    uint32_t now = HAL_GetTick();
    bool imu_ready = data->sensor_status.imu && data->sensors.imu_valid
        && data->sensors.attitude_6axis_valid
        && (uint32_t)(now - data->sensors.attitude_last_ms) < FLIGHT_ATTITUDE_TIMEOUT_MS;
    bool connected = data->radio_ready && data->remote_received_count != 0U
        && (uint32_t)(now - data->remote_last_ms) < FLIGHT_LINK_TIMEOUT_MS;
    if (!imu_ready || !data->acc_calibrated || !data->rate_output_valid)
    {
        /* boot: IMU setup and gyro calibration succeeded; cal: level ACC calibration. */
        debug_printf("rate: INVALID boot=%u imu=%u cal=%u rc=%u age=%lu O=%04d,%04d,%04d,%04d run=0\r\n",
                     (unsigned)data->sensor_status.imu, (unsigned)imu_ready, (unsigned)data->acc_calibrated, (unsigned)connected,
                     (unsigned long)(uint32_t)(now - data->rate_last_ms),
                     (int)data->motor_output.pwm1, (int)data->motor_output.pwm2,
                     (int)data->motor_output.pwm3, (int)data->motor_output.pwm4);
        return;
    }

    /* M is the mixed candidate; O is the last command sent to the PWM driver. */
    bool valid = connected && data->motor_preview_valid;
    char motors[24] = "----,----,----,----";
    if (valid)
    {
        (void)snprintf(motors, sizeof(motors), "%04d,%04d,%04d,%04d",
                       (int)data->motor_preview.pwm1, (int)data->motor_preview.pwm2,
                       (int)data->motor_preview.pwm3, (int)data->motor_preview.pwm4);
    }
    bool running = FLIGHT_RATE_TEST_ENABLED && data->armed && data->takeoff_requested
        && (data->motor_output.pwm1 != 0 || data->motor_output.pwm2 != 0
            || data->motor_output.pwm3 != 0 || data->motor_output.pwm4 != 0);
    debug_printf("rate: S=%.1f,%.1f,%.1f G=%.1f,%.1f,%.1f U=%.1f,%.1f,%.1f M=%s O=%04d,%04d,%04d,%04d run=%u valid=%u\r\n",
                 data->rate_target_dps.roll, data->rate_target_dps.pitch,
                 data->rate_target_dps.yaw,
                 data->sensors.imu.gyro_x_dps, data->sensors.imu.gyro_y_dps,
                 data->sensors.imu.gyro_z_dps,
                 data->rate_output.roll, data->rate_output.pitch, data->rate_output.yaw,
                 motors, (int)data->motor_output.pwm1, (int)data->motor_output.pwm2,
                 (int)data->motor_output.pwm3, (int)data->motor_output.pwm4,
                 (unsigned)running, (unsigned)valid);
}
