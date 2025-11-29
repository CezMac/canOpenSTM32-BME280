/*
 * configFlash.h
 *
 *  Created on: Aug 17, 2025
 *      Author: cezar
 */

#ifndef INC_CONFIGFLASH_H_
#define INC_CONFIGFLASH_H_

#include "main.h"

typedef struct
{
    uint32_t param1;
    uint32_t param2;
    uint32_t param3;
    uint32_t param4;
    uint32_t param5;
    uint32_t param6;
    uint32_t param7;
    uint32_t param8;

    uint32_t crc32;
} configFlash_t;

HAL_StatusTypeDef flashSave(configFlash_t *config);
HAL_StatusTypeDef flashRead(configFlash_t *config);

#endif /* INC_CONFIGFLASH_H_ */
