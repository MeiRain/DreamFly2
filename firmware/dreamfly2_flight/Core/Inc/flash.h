/**
 * @file flash.h
 * @brief Internal Flash storage interface.
 * @author 梅雨
 * @date 2026-10-22
 */

#ifndef DREAMFLY2_CORE_FLASH_H
#define DREAMFLY2_CORE_FLASH_H

#include <stdbool.h>
#include <stdint.h>

#define FLASH_STORAGE_SIZE_BYTES (128U * 1024U)

/**
 * @brief Read data from the reserved Flash sector.
 *
 * @param data Destination buffer.
 * @param size Number of bytes to read.
 * @return true if the parameters are valid; otherwise false.
 */
bool flash_read(void *data, uint32_t size);

/**
 * @brief Erase the reserved Flash sector.
 *
 * @return true if the sector was erased successfully; otherwise false.
 * @note Call only while the aircraft is disarmed.
 */
bool flash_erase(void);

/**
 * @brief Erase the reserved sector, write data and verify the result.
 *
 * @param data Source buffer.
 * @param size Number of bytes to write.
 * @return true if writing and verification succeeded; otherwise false.
 * @note Call only while the aircraft is disarmed.
 */
bool flash_write(const void *data, uint32_t size);

#endif /* DREAMFLY2_CORE_FLASH_H */
