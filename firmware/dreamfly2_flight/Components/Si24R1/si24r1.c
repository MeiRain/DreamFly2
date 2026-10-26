/**
 * @file si24r1.c
 * @brief HAL SPI2 radio driver with automatic ACK and dynamic payloads.
 * @author 梅雨
 * @date 2026-10-22
 */
#include "si24r1.h"

#include <stddef.h>
#include <string.h>

#include "spi.h"
#include "si24r1_reg.h"

/* Board mapping; GPIO/SPI initialization belongs to CubeMX. */
#define SI24R1_SPI_HANDLE hspi2
#define SI24R1_CE_PORT GPIOA
#define SI24R1_CE_PIN GPIO_PIN_8
#define SI24R1_CSN_PORT GPIOB
#define SI24R1_CSN_PIN GPIO_PIN_12
#define SI24R1_SPI_TIMEOUT_MS 2U
#define SI24R1_POWER_ON_DELAY_MS 100U
#define SI24R1_POWER_UP_DELAY_MS 2U
#define SI24R1_TX_TIMEOUT_MS 20U
#define SI24R1_RF_CHANNEL 0x48U
#define SI24R1_RF_SETUP 0x0FU /* Legacy SI24R1 setting: 2 Mbps and RF power bits. */
#define SI24R1_RETRY_SETUP 0x2AU /* 750 us retry delay, at most 10 retransmissions. */
#define SI24R1_CONFIG_BASE (EN_CRC | CRCO | PWR_UP)
#define SI24R1_IRQ_FLAGS (BIT_RX_DR | BIT_TX_DS | BIT_MAX_RT)

static const uint8_t s_address[5] = {0x34U, 0x43U, 0x10U, 0x10U, 0xFFU};
static si24r1_mode_t s_mode;
static bool s_initialized;
static bool s_tx_busy;
static uint32_t s_tx_start_ms;

/** @brief Drive CE; high listens in RX mode or starts a queued TX payload. */
static void si24r1_ce(bool high)
{
    HAL_GPIO_WritePin(SI24R1_CE_PORT, SI24R1_CE_PIN,
                      high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief Exchange one command and up to 32 bytes under one CSN assertion.
 * @note NULL tx sends dummy bytes; NULL rx discards data. Stack buffers are safe
 *       because HAL SPI completes synchronously. No ISR may access this bus.
 */
static bool si24r1_transfer(uint8_t command, const uint8_t *tx,
                            uint8_t *rx, uint8_t size)
{
    uint8_t tx_buffer[SI24R1_MAX_PAYLOAD_SIZE + 1U];
    uint8_t rx_buffer[SI24R1_MAX_PAYLOAD_SIZE + 1U];
    HAL_StatusTypeDef result;

    if (size > SI24R1_MAX_PAYLOAD_SIZE)
    {
        return false;
    }
    memset(tx_buffer, NOP, sizeof(tx_buffer));
    tx_buffer[0] = command;
    if (tx != NULL)
    {
        memcpy(&tx_buffer[1], tx, size);
    }
    HAL_GPIO_WritePin(SI24R1_CSN_PORT, SI24R1_CSN_PIN, GPIO_PIN_RESET);
    result = HAL_SPI_TransmitReceive(&SI24R1_SPI_HANDLE, tx_buffer, rx_buffer,
                                     (uint16_t)size + 1U, SI24R1_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(SI24R1_CSN_PORT, SI24R1_CSN_PIN, GPIO_PIN_SET);
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
static bool si24r1_write_register(uint8_t address, uint8_t value)
{
    return si24r1_transfer(W_REGISTER | (address & REGISTER_MASK), &value, NULL, 1U);
}

/** @brief Read one register; value must point to writable memory. */
static bool si24r1_read_register(uint8_t address, uint8_t *value)
{
    return si24r1_transfer(R_REGISTER | (address & REGISTER_MASK), NULL, value, 1U);
}

/** @brief Enable dynamic/ACK payloads, unlocking extended commands only if needed. */
static bool si24r1_enable_features(void)
{
    const uint8_t features = EN_DPL | EN_ACK_PAY;
    uint8_t value;
    uint8_t activate = ACTIVATE_DATA;

    if (!si24r1_write_register(FEATURE, features)
        || !si24r1_read_register(FEATURE, &value))
    {
        return false;
    }
    if (value != features)
    {
        if (!si24r1_transfer(ACTIVATE, &activate, NULL, 1U)
            || !si24r1_write_register(FEATURE, features)
            || !si24r1_read_register(FEATURE, &value))
        {
            return false;
        }
    }
    return value == features;
}

/** @brief Stop CE and invalidate software state after a transport fault. */
static void si24r1_fault(void)
{
    si24r1_ce(false);
    s_tx_busy = false;
    s_initialized = false;
}

bool si24r1_init(si24r1_mode_t mode)
{
    uint8_t address[sizeof(s_address)];
    uint8_t value;

    si24r1_fault();
    HAL_GPIO_WritePin(SI24R1_CSN_PORT, SI24R1_CSN_PIN, GPIO_PIN_SET);
    if (mode != SI24R1_MODE_RX && mode != SI24R1_MODE_TX)
    {
        return false;
    }
    HAL_Delay(SI24R1_POWER_ON_DELAY_MS);
    if (!si24r1_write_register(CONFIG, EN_CRC | CRCO)
        || !si24r1_write_register(SETUP_AW, ADDRESS_WIDTH_5)
        || !si24r1_transfer(W_REGISTER | TX_ADDR, s_address, NULL, sizeof(s_address))
        || !si24r1_transfer(R_REGISTER | TX_ADDR, NULL, address, sizeof(address))
        || memcmp(address, s_address, sizeof(address)) != 0
        || !si24r1_transfer(W_REGISTER | RX_ADDR_P0, s_address, NULL, sizeof(s_address))
        || !si24r1_write_register(EN_AA, PIPE0_ENABLE)
        || !si24r1_write_register(EN_RXADDR, PIPE0_ENABLE)
        || !si24r1_write_register(SETUP_RETR, SI24R1_RETRY_SETUP)
        || !si24r1_write_register(RF_CH, SI24R1_RF_CHANNEL)
        || !si24r1_write_register(RF_SETUP, SI24R1_RF_SETUP)
        || !si24r1_enable_features()
        || !si24r1_write_register(DYNPD, PIPE0_ENABLE)
        || !si24r1_read_register(DYNPD, &value)
        || value != PIPE0_ENABLE)
    {
        return false;
    }
    s_initialized = true;
    if (!si24r1_set_mode(mode))
    {
        si24r1_fault();
        return false;
    }
    HAL_Delay(SI24R1_POWER_UP_DELAY_MS);
    return true;
}

bool si24r1_set_mode(si24r1_mode_t mode)
{
    uint8_t config = SI24R1_CONFIG_BASE;
    uint8_t readback;

    if (!s_initialized || s_tx_busy
        || (mode != SI24R1_MODE_RX && mode != SI24R1_MODE_TX))
    {
        return false;
    }
    si24r1_ce(false);
    if (mode == SI24R1_MODE_RX)
    {
        config |= PRIM_RX;
    }
    if (!si24r1_transfer(FLUSH_TX, NULL, NULL, 0U)
        || !si24r1_transfer(FLUSH_RX, NULL, NULL, 0U)
        || !si24r1_write_register(STATUS, SI24R1_IRQ_FLAGS)
        || !si24r1_write_register(CONFIG, config)
        || !si24r1_read_register(CONFIG, &readback)
        || readback != config)
    {
        si24r1_fault();
        return false;
    }
    s_mode = mode;
    si24r1_ce(mode == SI24R1_MODE_RX);
    return true;
}

bool si24r1_send(const uint8_t *data, uint8_t size)
{
    if (!s_initialized || s_mode != SI24R1_MODE_TX || s_tx_busy
        || data == NULL || size == 0U || size > SI24R1_MAX_PAYLOAD_SIZE)
    {
        return false;
    }
    si24r1_ce(false);
    if (!si24r1_write_register(STATUS, BIT_TX_DS | BIT_MAX_RT)
        || !si24r1_transfer(FLUSH_TX, NULL, NULL, 0U)
        || !si24r1_transfer(W_TX_PAYLOAD, data, NULL, size))
    {
        si24r1_fault();
        return false;
    }
    s_tx_busy = true;
    s_tx_start_ms = HAL_GetTick();
    /* Hold CE high until result polling; no CPU delay is needed for a CE pulse. */
    si24r1_ce(true);
    return true;
}

si24r1_tx_result_t si24r1_get_tx_result(void)
{
    uint8_t status;
    si24r1_tx_result_t result;

    if (!s_initialized)
    {
        return SI24R1_TX_ERROR;
    }
    if (!s_tx_busy)
    {
        return SI24R1_TX_IDLE;
    }
    if (!si24r1_read_register(STATUS, &status))
    {
        si24r1_fault();
        return SI24R1_TX_ERROR;
    }
    if ((status & (BIT_TX_DS | BIT_MAX_RT)) == 0U)
    {
        if ((uint32_t)(HAL_GetTick() - s_tx_start_ms) < SI24R1_TX_TIMEOUT_MS)
        {
            return SI24R1_TX_PENDING;
        }
        si24r1_fault();
        return SI24R1_TX_ERROR;
    }
    si24r1_ce(false);
    result = (status & BIT_MAX_RT) != 0U ? SI24R1_TX_FAILED : SI24R1_TX_SUCCESS;
    if (!si24r1_write_register(STATUS, BIT_TX_DS | BIT_MAX_RT)
        || (result == SI24R1_TX_FAILED && !si24r1_transfer(FLUSH_TX, NULL, NULL, 0U)))
    {
        si24r1_fault();
        return SI24R1_TX_ERROR;
    }
    s_tx_busy = false;
    return result;
}

bool si24r1_receive(uint8_t *data, uint8_t capacity, uint8_t *size)
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
    if (!si24r1_read_register(FIFO_STATUS, &fifo))
    {
        si24r1_fault();
        return false;
    }
    if ((fifo & FIFO_RX_EMPTY) != 0U)
    {
        return false;
    }
    if (!si24r1_transfer(R_RX_PL_WID, NULL, &width, 1U))
    {
        si24r1_fault();
        return false;
    }
    if (width == 0U || width > SI24R1_MAX_PAYLOAD_SIZE)
    {
        if (!si24r1_transfer(FLUSH_RX, NULL, NULL, 0U)
            || !si24r1_write_register(STATUS, BIT_RX_DR))
        {
            si24r1_fault();
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
    if (!si24r1_write_register(STATUS, BIT_RX_DR)
        || !si24r1_transfer(R_RX_PAYLOAD, NULL, data, width))
    {
        si24r1_fault();
        return false;
    }
    *size = width;
    return true;
}

bool si24r1_write_ack_payload(const uint8_t *data, uint8_t size)
{
    uint8_t fifo;

    if (!s_initialized || s_mode != SI24R1_MODE_RX
        || data == NULL || size == 0U || size > SI24R1_MAX_PAYLOAD_SIZE)
    {
        return false;
    }
    if (!si24r1_read_register(FIFO_STATUS, &fifo))
    {
        si24r1_fault();
        return false;
    }
    if ((fifo & FIFO_TX_EMPTY) == 0U)
    {
        return false;
    }
    if (!si24r1_write_register(STATUS, BIT_TX_DS)
        || !si24r1_transfer(W_ACK_PAYLOAD, data, NULL, size))
    {
        si24r1_fault();
        return false;
    }
    return true;
}
