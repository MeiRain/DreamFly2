/**
 * @file nrf24l01.h
 * @brief NRF24L01 automatic ACK and dynamic-payload radio interface.
 * @author 梅雨
 * @date 2026-10-22
 */
#ifndef DREAMFLY2_COMPONENTS_NRF24L01_H
#define DREAMFLY2_COMPONENTS_NRF24L01_H

#include <stdbool.h>
#include <stdint.h>

#define NRF24L01_MAX_PAYLOAD_SIZE 32U

typedef enum
{
    NRF24L01_MODE_RX = 0, /* Flight controller: listen and return queued ACK data. */
    NRF24L01_MODE_TX      /* Remote controller: initiate transmissions. */
} nrf24l01_mode_t;

typedef enum
{
    NRF24L01_TX_IDLE = 0,
    NRF24L01_TX_PENDING,
    NRF24L01_TX_SUCCESS, /* ACK received, not confirmation of application processing. */
    NRF24L01_TX_FAILED,  /* Automatic retry limit reached. */
    NRF24L01_TX_ERROR    /* Transport error or timeout; reinitialize before reuse. */
} nrf24l01_tx_result_t;

/**
 * @brief Verify SPI access and configure pipe 0, 2 Mbps, CRC16 and ACK payloads.
 * @param mode RX for flight controller, TX for remote controller.
 * @return true on configuration/readback success, false otherwise.
 * @note Initialize GPIO/SPI1 first. Uses blocking startup delays. Address/channel
 *       constants in nrf24l01.c must match both ends. Serialize all calls in one
 *       task; never call from an ISR. Remote mapping: PA3 CE, PA4 CSN, SPI1; poll from one task.
 */
bool nrf24l01_init(nrf24l01_mode_t mode);

/**
 * @brief Switch radio role while retaining link settings.
 * @param mode New RX or TX role.
 * @return false if uninitialized, transmitting, mode invalid or SPI fails.
 * @note Call only while the peer is silent; clears RX and TX/ACK FIFOs.
 *       Fixed-role ACK communication does not need
 *       per-packet role changes. RX needs the hardware settling time before reception.
 */
bool nrf24l01_set_mode(nrf24l01_mode_t mode);

/**
 * @brief Load and start one TX-mode transmission without waiting for its ACK.
 * @param data Payload copied into the radio FIFO; reusable on return.
 * @param size Payload length, 1..32 bytes.
 * @return true if started; false if invalid, busy, wrong mode or SPI fails.
 * @note Poll nrf24l01_get_tx_result() before starting another transmission.
 */
bool nrf24l01_send(const uint8_t *data, uint8_t size);

/**
 * @brief Poll and consume the active transmission result.
 * @return Pending until completion; success/failed once, then idle. Errors require init.
 * @note Hardware performs ACK/retry without CPU waiting. Software timeout is
 *       20 ms; individual HAL SPI transactions still use bounded blocking transfers.
 */
nrf24l01_tx_result_t nrf24l01_get_tx_result(void);

/**
 * @brief Read one received payload, including ACK data in TX mode.
 * @param[out] data Output buffer; discard contents on failure.
 * @param capacity Buffer capacity in bytes.
 * @param[out] size Received byte count; zero on failure when non-NULL.
 * @return true for one payload; false for no data, invalid input or transfer failure.
 * @note Oversized payloads remain queued if capacity is too small. Invalid radio
 *       widths flush RX FIFO. Use a 32-byte buffer and repeat to drain queued data.
 */
bool nrf24l01_receive(uint8_t *data, uint8_t capacity, uint8_t *size);

/**
 * @brief Queue one RX-mode ACK payload if no previous ACK data remains queued.
 * @param data Status bytes copied into the radio FIFO.
 * @param size Payload length, 1..32 bytes.
 * @return true if queued; false if invalid, wrong mode, already queued or SPI fails.
 * @note Hardware sends it with a future ACK, not the packet already received.
 *       A pending payload is retained until consumed. This call does not flush
 *       the TX FIFO or interrupt reception; it keeps at most one payload queued.
 */
bool nrf24l01_write_ack_payload(const uint8_t *data, uint8_t size);

#endif /* DREAMFLY2_COMPONENTS_NRF24L01_H */
