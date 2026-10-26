/**
 * @file softi2c.c
 * @brief Blocking, open-drain software I2C with cycle-counter timing.
 * @author 梅雨
 * @date 2026-10-23
 */
#include "softi2c.h"

#include <stddef.h>

#include "stm32f4xx_hal.h"

#define SOFTI2C_GPIO_PORT GPIOB
#define SOFTI2C_SCL_PIN GPIO_PIN_4
#define SOFTI2C_SDA_PIN GPIO_PIN_5
#define SOFTI2C_HALF_PERIOD_US 5U
#define SOFTI2C_CLOCK_TIMEOUT_US 1000U

static uint32_t s_cycles_per_us; /* Zero until timing and bus initialization succeed. */

/** @brief Busy-wait for one half-period; interrupts may extend the wait. */
static void softi2c_delay(void)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = s_cycles_per_us * SOFTI2C_HALF_PERIOD_US;

    while ((uint32_t)(DWT->CYCCNT - start) < cycles)
    {
        __NOP();
    }
}

/** @brief Drive SDA low or release it; open-drain mode stays enabled during reads. */
static void softi2c_sda(bool high)
{
    HAL_GPIO_WritePin(SOFTI2C_GPIO_PORT, SOFTI2C_SDA_PIN,
                      high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/** @brief Drive SCL low to prepare the next data bit. */
static void softi2c_scl_low(void)
{
    HAL_GPIO_WritePin(SOFTI2C_GPIO_PORT, SOFTI2C_SCL_PIN, GPIO_PIN_RESET);
}

/** @brief Release SCL and wait for its actual high level with a bounded stretch timeout. */
static bool softi2c_scl_high(void)
{
    uint32_t start;
    uint32_t timeout_cycles = s_cycles_per_us * SOFTI2C_CLOCK_TIMEOUT_US;

    HAL_GPIO_WritePin(SOFTI2C_GPIO_PORT, SOFTI2C_SCL_PIN, GPIO_PIN_SET);
    start = DWT->CYCCNT;
    while (HAL_GPIO_ReadPin(SOFTI2C_GPIO_PORT, SOFTI2C_SCL_PIN) == GPIO_PIN_RESET)
    {
        if ((uint32_t)(DWT->CYCCNT - start) >= timeout_cycles)
        {
            return false;
        }
    }
    return true;
}

/** @brief Generate START or repeated START, rejecting a held-low SDA line. */
static bool softi2c_start(void)
{
    softi2c_sda(true);
    softi2c_delay();
    if (!softi2c_scl_high())
    {
        return false;
    }
    softi2c_delay();
    if (HAL_GPIO_ReadPin(SOFTI2C_GPIO_PORT, SOFTI2C_SDA_PIN) == GPIO_PIN_RESET)
    {
        return false;
    }
    softi2c_sda(false);
    softi2c_delay();
    softi2c_scl_low();
    return true;
}

/** @brief Attempt STOP and release both lines even when SCL is stuck low. */
static bool softi2c_stop(void)
{
    softi2c_scl_low();
    softi2c_sda(false);
    softi2c_delay();
    if (!softi2c_scl_high())
    {
        softi2c_sda(true);
        return false;
    }
    softi2c_delay();
    softi2c_sda(true);
    softi2c_delay();
    return HAL_GPIO_ReadPin(SOFTI2C_GPIO_PORT, SOFTI2C_SDA_PIN) == GPIO_PIN_SET;
}

/** @brief Send a byte MSB first and sample its ninth-clock ACK; SCL finishes low. */
static bool softi2c_write_byte(uint8_t value)
{
    bool acknowledged;

    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
        softi2c_sda((value & 0x80U) != 0U);
        softi2c_delay();
        if (!softi2c_scl_high())
        {
            return false;
        }
        softi2c_delay();
        softi2c_scl_low();
        value <<= 1U;
    }
    softi2c_sda(true);
    softi2c_delay();
    if (!softi2c_scl_high())
    {
        return false;
    }
    softi2c_delay();
    acknowledged = HAL_GPIO_ReadPin(SOFTI2C_GPIO_PORT, SOFTI2C_SDA_PIN) == GPIO_PIN_RESET;
    softi2c_scl_low();
    return acknowledged;
}

/** @brief Read a byte, then send ACK for another byte or NACK for the final byte. */
static bool softi2c_read_byte(uint8_t *value, bool acknowledge)
{
    uint8_t received = 0U;

    softi2c_sda(true);
    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
        softi2c_delay();
        if (!softi2c_scl_high())
        {
            return false;
        }
        softi2c_delay();
        received <<= 1U;
        if (HAL_GPIO_ReadPin(SOFTI2C_GPIO_PORT, SOFTI2C_SDA_PIN) == GPIO_PIN_SET)
        {
            received |= 1U;
        }
        softi2c_scl_low();
    }
    softi2c_sda(!acknowledge);
    softi2c_delay();
    if (!softi2c_scl_high())
    {
        return false;
    }
    softi2c_delay();
    softi2c_scl_low();
    softi2c_sda(true);
    *value = received;
    return true;
}

/** @brief Validate initialization, shifted device address, register width and buffer length. */
static bool softi2c_parameters_valid(uint8_t device_address, uint16_t register_address,
                                     uint8_t register_size, const void *data, uint16_t size)
{
    return s_cycles_per_us != 0U && (device_address & 1U) == 0U
           && data != NULL && size != 0U
           && ((register_size == SOFTI2C_REG_8BIT && register_address <= 0xFFU)
               || register_size == SOFTI2C_REG_16BIT);
}

/** @brief Start a write-address phase and send the register address, checking every ACK. */
static bool softi2c_select_register(uint8_t device_address, uint16_t register_address,
                                    uint8_t register_size)
{
    if (!softi2c_start() || !softi2c_write_byte(device_address))
    {
        return false;
    }
    if (register_size == SOFTI2C_REG_16BIT
        && !softi2c_write_byte((uint8_t)(register_address >> 8U)))
    {
        return false;
    }
    return softi2c_write_byte((uint8_t)register_address);
}

bool softi2c_init(void)
{
    uint32_t start;

    s_cycles_per_us = 0U;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    if ((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) != 0U)
    {
        return false;
    }
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    __DSB();
    __ISB();
    start = DWT->CYCCNT;
    __NOP();
    __NOP();
    if (DWT->CYCCNT == start)
    {
        return false;
    }
    s_cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000U;
    if (s_cycles_per_us == 0U)
    {
        return false;
    }
    softi2c_sda(true);
    if (!softi2c_scl_high())
    {
        s_cycles_per_us = 0U;
        return false;
    }
    softi2c_delay();
    if (HAL_GPIO_ReadPin(SOFTI2C_GPIO_PORT, SOFTI2C_SDA_PIN) == GPIO_PIN_RESET)
    {
        s_cycles_per_us = 0U;
        return false;
    }
    return true;
}

bool softi2c_write(uint8_t device_address, uint16_t register_address,
                   uint8_t register_size, const uint8_t *data, uint16_t size)
{
    bool success;
    bool stopped;

    if (!softi2c_parameters_valid(device_address, register_address, register_size, data, size))
    {
        return false;
    }
    success = softi2c_select_register(device_address, register_address, register_size);
    for (uint16_t index = 0U; success && index < size; index++)
    {
        success = softi2c_write_byte(data[index]);
    }
    stopped = softi2c_stop();
    return success && stopped;
}

bool softi2c_read(uint8_t device_address, uint16_t register_address,
                  uint8_t register_size, uint8_t *data, uint16_t size)
{
    bool success;
    bool stopped;

    if (!softi2c_parameters_valid(device_address, register_address, register_size, data, size))
    {
        return false;
    }
    success = softi2c_select_register(device_address, register_address, register_size);
    if (success)
    {
        success = softi2c_start() && softi2c_write_byte(device_address | 1U);
    }
    for (uint16_t index = 0U; success && index < size; index++)
    {
        success = softi2c_read_byte(&data[index], index + 1U < size);
    }
    stopped = softi2c_stop();
    return success && stopped;
}
