/*
 * Non_Volatile.h
 *
 *  Created on: 26 Feb 2021
 *      Author: thomas
 */

#ifndef INC_NON_VOLATILE_H_
#define INC_NON_VOLATILE_H_

#include "main.h"
#include <stdint.h>
#include <string.h>

#define FLASH_PAGE 0x0803F800
#define FLASH_PAGE_SIZE_SOC 0x800

typedef struct AMS_NVMem {
	float SoC;
	uint8_t imp1;
	uint16_t imp2;
} AMS_NVMem_t;

void flash_write(uint8_t* data); /**< Helper for writing flash */
void flash_read(uint8_t*data); /**< Helper for reading flash */
void flash_reset(); /**< Helper for wiping flash */

void flash_push(AMS_NVMem_t packet); /**< push a new AMS_NVMem_t packet to the flash */
void flash_pull(AMS_NVMem_t *packet); /**< pull the existing AMS_NVMem_t packet from the flash */

#endif /* INC_NON_VOLATILE_H_ */
