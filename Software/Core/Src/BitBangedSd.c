/*
 * BitBangedSd.c
 *
 *  Created on: Oct 12, 2020
 *      Author: thomas
 */

#include "BitBangedSD.h"

bbspi_t *new_bbspi(GPIO_TypeDef ports[], uint16_t pins[])
{
	bbspi_t *dev = malloc(sizeof(bbspi_t));
	memset(dev, 0, sizeof(bbspi_t));

	// Unions are magical
	dev->CLK_Port = 	ports[0];
	dev->MISO_Port = 	ports[1];
	dev->MOSI_Port = 	ports[2];
	dev->CS_Port = 		ports[3];

	dev->CLK_Pin = 		pins[0];
	dev->MISO_Pin = 	pins[1];
	dev->MOSI_Pin = 	pins[2];
	dev->CS_Pin = 		pins[3];

	// Setup GPIO
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = dev->CLK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(&(dev->CLK_Port), &GPIO_InitStruct);

	GPIO_InitStruct.Pin = dev->MISO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(&(dev->MISO_Port), &GPIO_InitStruct);

	GPIO_InitStruct.Pin = dev->MOSI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(&(dev->MOSI_Port), &GPIO_InitStruct);

	GPIO_InitStruct.Pin = dev->CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(&(dev->CS_Port), &GPIO_InitStruct);

	return dev;
}

uint8_t bbspi_transferByte(bbspi_t *dev, uint8_t data)
{
	uint8_t dataIn = 0;
	for(int i = 0x80; i; i >>= 1)
	{
		HAL_GPIO_WritePin(&(dev->MOSI_Port), dev->MOSI_Pin, (data & i) ? GPIO_PIN_SET : GPIO_PIN_RESET);

		for(int j = 0; j < 500; j++);	// TODO, calc delay time

		HAL_GPIO_WritePin(&(dev->CLK_Port), dev->CLK_Pin, GPIO_PIN_SET);
		if(HAL_GPIO_ReadPin(&(dev->MISO_Port), dev->MISO_Pin) == GPIO_PIN_SET)
		{
			dataIn |= i;
		}

		for(int j = 0; j < 500; j++);	// TODO, calc delay time

		HAL_GPIO_WritePin(&(dev->CLK_Port), dev->CLK_Pin, GPIO_PIN_RESET);
	}
	return dataIn;
}

void bbspi_delete(bbspi_t *dev)
{
	free(dev);
}
