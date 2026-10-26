/**
 * @file nrf24l01_reg.h
 * @brief NRF24L01 SPI commands, register addresses and configuration values.
 * @author 梅雨
 * @date 2026-10-22
 */
#ifndef DREAMFLY2_COMPONENTS_NRF24L01_REG_H
#define DREAMFLY2_COMPONENTS_NRF24L01_REG_H

/* SPI commands. Register commands use the low five bits as the address. */
#define R_REGISTER      0x00U
#define W_REGISTER      0x20U
#define ACTIVATE        0x50U
#define R_RX_PL_WID     0x60U
#define R_RX_PAYLOAD    0x61U
#define W_TX_PAYLOAD    0xA0U
#define W_ACK_PAYLOAD   0xA8U /* OR with the pipe number; driver uses pipe 0. */
#define W_TX_PAYLOAD_NOACK 0xB0U
#define FLUSH_TX        0xE1U
#define FLUSH_RX        0xE2U
#define REUSE_TX_PL     0xE3U
#define NOP             0xFFU

/* Register addresses. */
#define CONFIG          0x00U
#define EN_AA           0x01U
#define EN_RXADDR       0x02U
#define SETUP_AW        0x03U
#define SETUP_RETR      0x04U
#define RF_CH           0x05U
#define RF_SETUP        0x06U
#define STATUS          0x07U
#define OBSERVE_TX      0x08U
#define CD              0x09U /* Carrier detection, not a numeric RSSI. */
#define RX_ADDR_P0      0x0AU
#define RX_ADDR_P1      0x0BU
#define RX_ADDR_P2      0x0CU
#define RX_ADDR_P3      0x0DU
#define RX_ADDR_P4      0x0EU
#define RX_ADDR_P5      0x0FU
#define TX_ADDR         0x10U
#define RX_PW_P0        0x11U
#define RX_PW_P1        0x12U
#define RX_PW_P2        0x13U
#define RX_PW_P3        0x14U
#define RX_PW_P4        0x15U
#define RX_PW_P5        0x16U
#define FIFO_STATUS     0x17U
#define DYNPD           0x1CU
#define FEATURE         0x1DU

/* Configuration values and masks, not register addresses. */
#define REGISTER_MASK   0x1FU
#define EN_CRC          0x08U
#define CRCO            0x04U
#define PWR_UP          0x02U
#define PRIM_RX         0x01U
#define BIT_RX_DR       0x40U /* Write 1 to clear. */
#define BIT_TX_DS       0x20U /* Write 1 to clear. */
#define BIT_MAX_RT      0x10U /* Write 1 to clear. */
#define BIT_TX_FULL     0x01U /* STATUS bit 0. */
#define FIFO_RX_EMPTY   0x01U /* FIFO_STATUS bit 0. */
#define FIFO_TX_EMPTY   0x10U /* FIFO_STATUS bit 4; ACK payloads use the TX FIFO. */
#define PIPE0_ENABLE    0x01U
#define ADDRESS_WIDTH_5 0x03U
/* RF_SETUP: 2 Mbps, 0 dBm output power and high LNA gain (nRF24L01). */
#define RF_DR_2MBPS     0x08U
#define RF_PWR_0DBM     0x06U
#define LNA_HCURR       0x01U
#define EN_DPL          0x04U
#define EN_ACK_PAY      0x02U
#define ACTIVATE_DATA   0x73U

#endif /* DREAMFLY2_COMPONENTS_NRF24L01_REG_H */
