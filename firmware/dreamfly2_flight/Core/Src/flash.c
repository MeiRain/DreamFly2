/**
 * @file flash.c
 * @brief Internal Flash storage using STM32F411 sector 7.
 * @author 梅雨
 * @date 2026-10-22
 */

#include "flash.h"

#include <string.h>

#include "stm32f4xx_hal.h"

#define FLASH_STORAGE_ADDRESS 0x08060000U
#define FLASH_STORAGE_SECTOR FLASH_SECTOR_7

static bool flash_parameters_valid(const void *data, uint32_t size)
{
    return data != NULL && size > 0U && size <= FLASH_STORAGE_SIZE_BYTES;
}

static bool flash_erase_unlocked(void)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t sector_error = 0U;

    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = FLASH_STORAGE_SECTOR;
    erase.NbSectors = 1U;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    return HAL_FLASHEx_Erase(&erase, &sector_error) == HAL_OK;
}

bool flash_read(void *data, uint32_t size)
{
    if (!flash_parameters_valid(data, size))
    {
        return false;
    }

    memcpy(data, (const void *)FLASH_STORAGE_ADDRESS, size);
    return true;
}

bool flash_erase(void)
{
    bool success;

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return false;
    }

    success = flash_erase_unlocked();
    HAL_FLASH_Lock();

    return success;
}

bool flash_write(const void *data, uint32_t size)
{
    const uint8_t *bytes = data;
    uint32_t address = FLASH_STORAGE_ADDRESS;
    bool success = true;

    if (!flash_parameters_valid(data, size))
    {
        return false;
    }

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return false;
    }

    if (!flash_erase_unlocked())
    {
        success = false;
    }

    for (uint32_t i = 0U; success && i < size; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address, bytes[i]) != HAL_OK)
        {
            success = false;
        }

        address++;
    }

    HAL_FLASH_Lock();

    if (success && memcmp((const void *)FLASH_STORAGE_ADDRESS, data, size) != 0)
    {
        success = false;
    }

    return success;
}
