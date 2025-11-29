/*
 * configFlash.c
 *
 *  Created on: Aug 17, 2025
 *      Author: cezar
 */

#include "flashEeprom.h"

#define CONFIG_FLASH_ADDRESS 0x0801F000

static uint32_t calcCrc32(const void *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFF;
    const uint8_t *p = (const uint8_t *)data;

    while (length--)
    {
        crc ^= *p++;

        for (uint8_t i = 0; i < 8; i++)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320;
            else
                crc >>= 1;
        }
    }

    return ~crc;
}

HAL_StatusTypeDef flashSave(configFlash_t *config)
{
    if (config == NULL)
        return HAL_ERROR;

    config->crc32 = calcCrc32(config, sizeof(configFlash_t) - sizeof(uint32_t));

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {
        .TypeErase   = FLASH_TYPEERASE_PAGES,
        .PageAddress = CONFIG_FLASH_ADDRESS,
        .NbPages     = 1
    };

    uint32_t pageError = 0;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &pageError);

    if (status != HAL_OK)
    {
        HAL_FLASH_Lock();
        return status;
    }

    const uint32_t *src = (const uint32_t *)config;
    uint32_t *dst = (uint32_t *)CONFIG_FLASH_ADDRESS;

    for (size_t i = 0; i < sizeof(configFlash_t) / 4; i++)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)(dst + i), src[i]);

        if (status != HAL_OK)
        {
            HAL_FLASH_Lock();
            return status;
        }
    }

    HAL_FLASH_Lock();
    return HAL_OK;
}

HAL_StatusTypeDef flashRead(configFlash_t *config)
{
    if (config == NULL)
        return HAL_ERROR;

    const configFlash_t *stored = (const configFlash_t *)CONFIG_FLASH_ADDRESS;

    uint32_t crc = calcCrc32(stored, sizeof(configFlash_t) - sizeof(uint32_t));

    if (stored->crc32 == crc)
    {
        *config = *stored;
        return HAL_OK;
    }

    return HAL_ERROR;
}
