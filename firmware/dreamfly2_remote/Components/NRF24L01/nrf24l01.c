/**
 * @file nrf24l01.c
 * @brief HAL SPI1 remote radio driver with automatic ACK and dynamic payloads.
 * @author 梅雨
 * @date 2026-10-22
 */
#include "nrf24l01.h"

#include <stddef.h>
#include <string.h>

#include "spi.h"
#include "nrf24l01_reg.h"

/* Board mapping; GPIO/SPI initialization belongs to CubeMX. */
#define NRF24L01_SPI_HANDLE hspi1
#define NRF24L01_CE_PORT GPIOA
#define NRF24L01_CE_PIN GPIO_PIN_3
#define NRF24L01_CSN_PORT GPIOA
#define NRF24L01_CSN_PIN GPIO_PIN_4
#define NRF24L01_SPI_TIMEOUT_MS 2U
#define NRF24L01_POWER_ON_DELAY_MS 100U
#define NRF24L01_POWER_UP_DELAY_MS 2U
#define NRF24L01_TX_TIMEOUT_MS 20U
#define NRF24L01_RF_CHANNEL 0x48U
#define NRF24L01_RF_SETUP (RF_DR_2MBPS | RF_PWR_0DBM | LNA_HCURR)
#define NRF24L01_RETRY_SETUP 0x2AU /* 750 us retry delay, at most 10 retransmissions. */
#define NRF24L01_CONFIG_BASE (EN_CRC | CRCO | PWR_UP)
#define NRF24L01_IRQ_FLAGS (BIT_RX_DR | BIT_TX_DS | BIT_MAX_RT)

static const uint8_t s_address[5] = {0x34U, 0x43U, 0x10U, 0x10U, 0xFFU};
static nrf24l01_mode_t s_mode;
static bool s_initialized;
static bool s_tx_busy;
static uint32_t s_tx_start_ms;

/** @brief Drive CE; high listens in RX mode or starts a queued TX payload. */
static void nrf24l01_ce(bool high)
{
    HAL_GPIO_WritePin(NRF24L01_CE_PORT, NRF24L01_CE_PIN,
                      high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief Exchange one command and up to 32 bytes under one CSN assertion.
 * @note NULL tx sends dummy bytes; NULL rx discards data. Stack buffers are safe
 *       because HAL SPI completes synchronously. No ISR may access this bus.
 */
static bool nrf24l01_transfer(uint8_t command, const uint8_t *tx,
                            uint8_t *rx, uint8_t size)
{
    uint8_t tx_buffer[NRF24L01_MAX_PAYLOAD_SIZE + 1U];
    uint8_t rx_buffer[NRF24L01_MAX_PAYLOAD_SIZE + 1U];
    HAL_StatusTypeDef result;

    if (size > NRF24L01_MAX_PAYLOAD_SIZE)
    {
        return false;
    }
    memset(tx_buffer, NOP, sizeof(tx_buffer));
    tx_buffer[0] = command;
    if (tx != NULL)
    {
        memcpy(&tx_buffer[1], tx, size);
    }
    HAL_GPIO_WritePin(NRF24L01_CSN_PORT, NRF24L01_CSN_PIN, GPIO_PIN_RESET);
    result = HAL_SPI_TransmitReceive(&NRF24L01_SPI_HANDLE, tx_buffer, rx_buffer,
                                     (uint16_t)size + 1U, NRF24L01_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(NRF24L01_CSN_PORT, NRF24L01_CSN_PIN, GPIO_PIN_SET);
    if (result != HAL_OK || (rx_buffer[0] & 0x80U) != 0U)
    {
        return false;
    }
    if (rx != NULL)
    {
        memcpy(rx, &rx_buffer[1], size);
    }
    return true;
}

/** @brief Write one register and report SPI transfer success. */
static bool nrf24l01_write_register(uint8_t address, uint8_t value)
{
    return nrf24l01_transfer(W_REGISTER | (address & REGISTER_MASK), &value, NULL, 1U);
}

/** @brief Read one register; value must point to writable memory. */
static bool nrf24l01_read_register(uint8_t address, uint8_t *value)
{
    return nrf24l01_transfer(R_REGISTER | (address & REGISTER_MASK), NULL, value, 1U);
}

/** @brief Enable dynamic/ACK payloads, unlocking extended commands only if needed. */
static bool nrf24l01_enable_features(void)
{
    const uint8_t features = EN_DPL | EN_ACK_PAY;
    uint8_t value;
    uint8_t activate = ACTIVATE_DATA;

    if (!nrf24l01_write_register(FEATURE, features)
        || !nrf24l01_read_register(FEATURE, &value))
    {
        return false;
    }
    if (value != features)
    {
        if (!nrf24l01_transfer(ACTIVATE, &activate, NULL, 1U)
            || !nrf24l01_write_register(FEATURE, features)
            || !nrf24l01_read_register(FEATURE, &value))
        {
            return false;
        }
    }
    return value == features;
}

/** @brief Stop CE and invalidate software state after a transport fault. */
static void nrf24l01_fault(void)
{
    nrf24l01_ce(false);
    s_tx_busy = false;
    s_initialized = false;
}

bool nrf24l01_init(nrf24l01_mode_t mode)
{
    uint8_t address[sizeof(s_address)];
    uint8_t value;

    nrf24l01_fault();
    HAL_GPIO_WritePin(NRF24L01_CSN_PORT, NRF24L01_CSN_PIN, GPIO_PIN_SET);
    if (mode != NRF24L01_MODE_RX && mode != NRF24L01_MODE_TX)
    {
        return false;
    }
    HAL_Delay(NRF24L01_POWER_ON_DELAY_MS);
    if (!nrf24l01_write_register(CONFIG, EN_CRC | CRCO)
        || !nrf24l01_write_register(SETUP_AW, ADDRESS_WIDTH_5)
        || !nrf24l01_transfer(W_REGISTER | TX_ADDR, s_address, NULL, sizeof(s_address))
        || !nrf24l01_transfer(R_REGISTER | TX_ADDR, NULL, address, sizeof(address))
        || memcmp(address, s_address, sizeof(address)) != 0
        || !nrf24l01_transfer(W_REGISTER | RX_ADDR_P0, s_address, NULL, sizeof(s_address))
        || !nrf24l01_write_register(EN_AA, PIPE0_ENABLE)
        || !nrf24l01_write_register(EN_RXADDR, PIPE0_ENABLE)
        || !nrf24l01_write_register(SETUP_RETR, NRF24L01_RETRY_SETUP)
        || !nrf24l01_write_register(RF_CH, NRF24L01_RF_CHANNEL)
        || !nrf24l01_write_register(RF_SETUP, NRF24L01_RF_SETUP)
        || !nrf24l01_enable_features()
        || !nrf24l01_write_register(DYNPD, PIPE0_ENABLE)
        || !nrf24l01_read_register(DYNPD, &value)
        || value != PIPE0_ENABLE)
    {
        return false;
    }
    s_initialized = true;
    if (!nrf24l01_set_mode(mode))
    {
        nrf24l01_fault();
        return false;
    }
    HAL_Delay(NRF24L01_POWER_UP_DELAY_MS);
    return true;
}

bool nrf24l01_set_mode(nrf24l01_mode_t mode)
{
    uint8_t config = NRF24L01_CONFIG_BASE;
    uint8_t readback;

    if (!s_initialized || s_tx_busy
        || (mode != NRF24L01_MODE_RX && mode != NRF24L01_MODE_TX))
    {
        return false;
    }
    nrf24l01_ce(false);
    if (mode == NRF24L01_MODE_RX)
    {
        config |= PRIM_RX;
    }
    if (!nrf24l01_transfer(FLUSH_TX, NULL, NULL, 0U)
        || !nrf24l01_transfer(FLUSH_RX, NULL, NULL, 0U)
        || !nrf24l01_write_register(STATUS, NRF24L01_IRQ_FLAGS)
        || !nrf24l01_write_register(CONFIG, config)
        || !nrf24l01_read_register(CONFIG, &readback)
        || readback != config)
    {
        nrf24l01_fault();
        return false;
    }
    s_mode = mode;
    nrf24l01_ce(mode == NRF24L01_MODE_RX);
    return true;
}

bool nrf24l01_send(const uint8_t *data, uint8_t size)
{
    if (!s_initialized || s_mode != NRF24L01_MODE_TX || s_tx_busy
        || data == NULL || size == 0U || size > NRF24L01_MAX_PAYLOAD_SIZE)
    {
        return false;
    }
    nrf24l01_ce(false);
    if (!nrf24l01_write_register(STATUS, BIT_TX_DS | BIT_MAX_RT)
        || !nrf24l01_transfer(FLUSH_TX, NULL, NULL, 0U)
        || !nrf24l01_transfer(W_TX_PAYLOAD, data, NULL, size))
    {
        nrf24l01_fault();
        return false;
    }
    s_tx_busy = true;
    s_tx_start_ms = HAL_GetTick();
    /* Hold CE high until result polling; no CPU delay is needed for a CE pulse. */
    nrf24l01_ce(true);
    return true;
}

nrf24l01_tx_result_t nrf24l01_get_tx_result(void)
{
    uint8_t status;
    nrf24l01_tx_result_t result;

    if (!s_initialized)
    {
        return NRF24L01_TX_ERROR;
    }
    if (!s_tx_busy)
    {
        return NRF24L01_TX_IDLE;
    }
    if (!nrf24l01_read_register(STATUS, &status))
    {
        nrf24l01_fault();
        return NRF24L01_TX_ERROR;
    }
    if ((status & (BIT_TX_DS | BIT_MAX_RT)) == 0U)
    {
        if ((uint32_t)(HAL_GetTick() - s_tx_start_ms) < NRF24L01_TX_TIMEOUT_MS)
        {
            return NRF24L01_TX_PENDING;
        }
        nrf24l01_fault();
        return NRF24L01_TX_ERROR;
    }
    nrf24l01_ce(false);
    result = (status & BIT_MAX_RT) != 0U ? NRF24L01_TX_FAILED : NRF24L01_TX_SUCCESS;
    if (!nrf24l01_write_register(STATUS, BIT_TX_DS | BIT_MAX_RT)
        || (result == NRF24L01_TX_FAILED && !nrf24l01_transfer(FLUSH_TX, NULL, NULL, 0U)))
    {
        nrf24l01_fault();
        return NRF24L01_TX_ERROR;
    }
    s_tx_busy = false;
    return result;
}

bool nrf24l01_receive(uint8_t *data, uint8_t capacity, uint8_t *size)
{
    uint8_t fifo;
    uint8_t width;

    if (size != NULL)
    {
        *size = 0U;
    }
    if (!s_initialized || data == NULL || size == NULL || capacity == 0U)
    {
        return false;
    }
    if (!nrf24l01_read_register(FIFO_STATUS, &fifo))
    {
        nrf24l01_fault();
        return false;
    }
    if ((fifo & FIFO_RX_EMPTY) != 0U)
    {
        return false;
    }
    if (!nrf24l01_transfer(R_RX_PL_WID, NULL, &width, 1U))
    {
        nrf24l01_fault();
        return false;
    }
    if (width == 0U || width > NRF24L01_MAX_PAYLOAD_SIZE)
    {
        if (!nrf24l01_transfer(FLUSH_RX, NULL, NULL, 0U)
            || !nrf24l01_write_register(STATUS, BIT_RX_DR))
        {
            nrf24l01_fault();
        }
        return false;
    }
    if (width > capacity)
    {
        return false;
    }
    /* Clear before popping; new arrivals may assert RX_DR again. Always inspect
     * FIFO_STATUS on the next call, regardless of the IRQ pin level.
     */
    if (!nrf24l01_write_register(STATUS, BIT_RX_DR)
        || !nrf24l01_transfer(R_RX_PAYLOAD, NULL, data, width))
    {
        nrf24l01_fault();
        return false;
    }
    *size = width;
    return true;
}

bool nrf24l01_write_ack_payload(const uint8_t *data, uint8_t size)
{
    uint8_t fifo;

    if (!s_initialized || s_mode != NRF24L01_MODE_RX
        || data == NULL || size == 0U || size > NRF24L01_MAX_PAYLOAD_SIZE)
    {
        return false;
    }
    if (!nrf24l01_read_register(FIFO_STATUS, &fifo))
    {
        nrf24l01_fault();
        return false;
    }
    if ((fifo & FIFO_TX_EMPTY) == 0U)
    {
        return false;
    }
    if (!nrf24l01_write_register(STATUS, BIT_TX_DS)
        || !nrf24l01_transfer(W_ACK_PAYLOAD, data, NULL, size))
    {
        nrf24l01_fault();
        return false;
    }
    return true;
}
