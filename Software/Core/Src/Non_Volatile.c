/*
 * Non_Volatile.c
 *
 *  Created on: 26 Feb 2021
 *      Author: thomas
 */

#include "Non_Volatile.h"

void flash_write(uint8_t*data)
{
	volatile uint32_t dataToFlash[(strlen((char*)data)/4) + (int)((strlen((char*)data) % 4) != 0)];
	memset((uint8_t*)dataToFlash, 0, strlen((char*)dataToFlash));
	strcpy((char*)dataToFlash, (char*)data);

	volatile uint32_t dataLength = (strlen((char*)dataToFlash)/4) + (int)((strlen((char*)dataToFlash) % 4) != 0);
	volatile uint32_t pages = (strlen((char*)data) / FLASH_PAGE_SIZE_SOC) + (int)((strlen((char*)data) % FLASH_PAGE_SIZE_SOC) != 0);

	/** Time to: https://open.spotify.com/album/19hFTVuLUtQfYf4Z5FdFgj */
	HAL_FLASH_Unlock();
	HAL_FLASH_OB_Unlock();

	FLASH_EraseInitTypeDef eraseInitStruct = {
			.TypeErase = FLASH_TYPEERASE_PAGES,
			.PageAddress = FLASH_PAGE,
			.NbPages = pages
	};
	uint32_t pageError;

	volatile uint32_t writeCount = 0, index = 0;

	volatile HAL_StatusTypeDef status;
	status = HAL_FLASHEx_Erase(&eraseInitStruct, &pageError);
	while(index < dataLength)
	{
		if(status == HAL_OK)
		{
			status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_PAGE+writeCount, dataToFlash[index]);
			if(status == HAL_OK)
			{
				writeCount += 4;
				index++;
			}
		}
	}

	HAL_FLASH_Lock();
	HAL_FLASH_OB_Lock();
}

void flash_read(uint8_t*data)
{
	volatile uint32_t readData;
	volatile uint32_t readCount = 0;
	do {
		readData = *(int32_t*)(FLASH_PAGE + readCount);
		if(readData != 0xFFFFFFFF)
		{
			data[readCount] = (uint8_t)readData;
			data[readCount + 1] = (uint8_t)(readData >> 8);
			data[readCount + 2] = (uint8_t)(readData >> 16);
			data[readCount + 3] = (uint8_t)(readData >> 24);
			readCount += 4;
		}
	} while(readData != 0xFFFFFFFF);
}

void flash_reset()
{
	/** Time to: https://open.spotify.com/album/19hFTVuLUtQfYf4Z5FdFgj */
	HAL_FLASH_Unlock();
	HAL_FLASH_OB_Unlock();

	FLASH_EraseInitTypeDef eraseInitStruct = {
			.TypeErase = FLASH_TYPEERASE_PAGES,
			.PageAddress = FLASH_PAGE,
			.NbPages = FLASH_PAGE_SIZE_SOC
	};
	uint32_t pageError;

	HAL_FLASHEx_Erase(&eraseInitStruct, &pageError);

	HAL_FLASH_Lock();
	HAL_FLASH_OB_Lock();
}

void flash_push(AMS_NVMem_t packet)
{
	flash_write((uint8_t*)&packet);
}

void flash_pull(AMS_NVMem_t *packet)
{
	flash_read((uint8_t*)packet);
}
