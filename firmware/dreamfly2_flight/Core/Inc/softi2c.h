/**
 * @file softi2c.h
 * @brief Software I2C master on PB4/SCL and PB5/SDA.
 * @author 梅雨
 * @date 2026-10-23
 */
#ifndef DREAMFLY2_CORE_SOFTI2C_H
#define DREAMFLY2_CORE_SOFTI2C_H

#include <stdbool.h>
#include <stdint.h>

#define SOFTI2C_REG_8BIT 1U
#define SOFTI2C_REG_16BIT 2U

/**
 * @brief Enable cycle-counter timing, release the bus and check its idle state.
 * @return true when timing works and both bus lines are high; false otherwise.
 * @note Call after system-clock and GPIO initialization. PB4/PB5 must be open-drain
 *       outputs with external pull-ups. Does not configure GPIO or reset a device.
 *       Reinitialize after changing HCLK. The DWT counter is enabled but not reset.
 */
bool softi2c_init(void);

/**
 * @brief Write consecutive bytes starting at a device register.
 * @param device_address HAL-style shifted address with bit 0 clear; VL53L1X uses 0x52.
 * @param register_address First register; 16-bit addresses are sent high byte first.
 * @param register_size SOFTI2C_REG_8BIT or SOFTI2C_REG_16BIT.
 * @param data Source buffer containing size bytes.
 * @param size Number of bytes to write; must be nonzero.
 * @return true if every byte is acknowledged and STOP succeeds; false otherwise.
 * @note Requires softi2c_init(). Blocking, single-master bus; call from one task
 *       only, never an ISR. Interrupts remain enabled. No automatic retries.
 *       A failed write may already have changed earlier device registers.
 */
bool softi2c_write(uint8_t device_address, uint16_t register_address,
                   uint8_t register_size, const uint8_t *data, uint16_t size);

/**
 * @brief Read consecutive register bytes using a repeated START.
 * @param device_address HAL-style shifted address with bit 0 clear; VL53L1X uses 0x52.
 * @param register_address First register; 16-bit addresses are sent high byte first.
 * @param register_size SOFTI2C_REG_8BIT or SOFTI2C_REG_16BIT.
 * @param[out] data Buffer with at least size bytes; discard its contents on failure.
 * @param size Number of bytes to read; must be nonzero.
 * @return true when address/register ACKs, reading and STOP succeed; false otherwise.
 * @note Same single-task contract as softi2c_write(). Last byte is followed by NACK.
 *       Nominal clock is at most about 100 kHz; GPIO overhead and preemption slow it.
 *       Each SCL release allows up to 1 ms for clock stretching; stuck buses are
 *       reported as failures, without automatic recovery clock pulses.
 */
bool softi2c_read(uint8_t device_address, uint16_t register_address,
                  uint8_t register_size, uint8_t *data, uint16_t size);

#endif /* DREAMFLY2_CORE_SOFTI2C_H */
