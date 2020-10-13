/*
 * BitBangedSD.h
 *
 *  Created on: Oct 12, 2020
 *      Author: thomas
 */

#ifndef INC_BITBANGEDSD_H_
#define INC_BITBANGEDSD_H_

#include "main.h"
#include <stdlib.h>
#include <memory.h>

typedef struct bbspi
{
	union
	{
		GPIO_TypeDef Ports[4];
		struct {
			GPIO_TypeDef CLK_Port;
			GPIO_TypeDef MISO_Port;
			GPIO_TypeDef MOSI_Port;
			GPIO_TypeDef CS_Port;
		};
	};

	union
	{
		uint16_t Pins[4];
		struct {
			uint16_t CLK_Pin;
			uint16_t MISO_Pin;
			uint16_t MOSI_Pin;
			uint16_t CS_Pin;
		};
	};

} bbspi_t;

/**
 * @brief Creates a new Bitbanged SPI object
 * @param clk Clock Pin
 * @param miso Master In Slave Out Pin
 * @param mosi Master Out Slave In Pin
 * @param cs Slave Chip Select
 * @return The Bitbanged SPI device
 */
bbspi_t *new_bbspi(GPIO_TypeDef ports[], uint16_t pins[]);

/**
 * @brief Transfer a byte via Bitbanged SPI
 * @param dev The Bigbanged SPI Device
 * @param data Byte to send
 * @return Byte recieved
 */
uint8_t bbspi_transferByte(bbspi_t *dev, uint8_t data);

/**
 * @brief Delete Bitbanged SPI Device
 * @param dev The Bigbanged SPI Device
 */
void bbspi_delete(bbspi_t *dev);




#endif /* INC_BITBANGEDSD_H_ */
