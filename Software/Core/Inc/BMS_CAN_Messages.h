/*
 * BMS_CAN_Messages.h
 *
 *  Created on: Oct 1, 2020
 *      Author: Thomas Fraser
 */

#ifndef INC_BMS_CAN_MESSAGES_H_
#define INC_BMS_CAN_MESSAGES_H_

#include "main.h"
#include "can.h"

/* General Structure for BMS CAN Messages:
 * id: The 32bit can id for can packet header.
 * data: sets of 8 bits of information, max of 64 bits per CAN packet
 */

typedef struct BMS_BadCellVoltage
{
	uint32_t id;
	uint8_t data[2];
} BMS_BadCellVoltage_t;
BMS_BadCellVoltage_t Compose_BMS_BadCellVoltage(uint8_t BMSId, uint8_t cellNumber, uint8_t voltage);
void Parse_BMS_BadCellVoltage(BMS_BadCellVoltage_t packet, uint8_t* BMSId, uint8_t* cellNumber, uint8_t* voltage);

typedef struct BMS_BadCellTemperature
{
	uint32_t id;
	uint8_t data[2];
} BMS_BadCellTemperature_t;
BMS_BadCellTemperature_t Compose_BMS_BadCellTemperature(uint8_t BMSId, uint8_t cellNumber, uint8_t temperature);
void Parse_BMS_BadCellTemperature(BMS_BadCellTemperature_t packet, uint8_t* BMSId, uint8_t* cellNumber, uint8_t* temperature);

typedef struct BMS_TransmitVoltage
{
	uint32_t id;
	uint8_t data[8];
} BMS_TransmitVoltage_t;
BMS_TransmitVoltage_t Compose_BMS_TransmitVoltage(uint8_t BMSId, uint8_t vMsgId, uint16_t voltages[4]);
void Parse_BMS_TransmitVoltage(BMS_TransmitVoltage_t packet, uint8_t* BMSId, uint8_t* vMsgId, uint16_t* voltages[4]);

typedef struct BMS_TransmitTemperature
{
	uint32_t id;
	uint8_t data[7];
} BMS_TransmitTemperature_t;
BMS_TransmitTemperature_t Compose_BMS_TransmitTemperature(uint8_t BMSId, uint8_t tMsgId, uint8_t temperatures[6]);
void Parse_BMS_TransmitTemperature(BMS_TransmitTemperature_t packet, uint8_t* BMSId, uint8_t* tMsgId, uint8_t* temperatures[6]);

typedef struct BMS_ChargeEnabled
{
	uint32_t id;
} BMS_ChargeEnabled_t;
BMS_ChargeEnabled_t Compose_BMS_ChargeEnabled(uint8_t BMSId);
void Parse_ChargeEnabled(BMS_ChargeEnabled_t packet, uint8_t* BMSId);

typedef struct BMS_TransmitDeviceId
{
	uint32_t id;
	uint8_t data[4];
} BMS_TransmitDeviceId_t;
BMS_TransmitDeviceId_t Compose_BMS_TransmitDeviceId(uint8_t BMSId, uint32_t uid);
void Parse_BMS_TransmitDeviceId(BMS_TransmitDeviceId_t packet, uint8_t* BMSId, uint32_t* uid);

typedef struct BMS_SetBMSId
{
	uint32_t id;
	uint8_t data[5];
} BMS_SetBMSId_t;
BMS_SetBMSId_t Compose_BMS_SetBMSId(uint8_t BMSId, uint32_t uid);
void Parse_BMS_SetBMSId(BMS_SetBMSId_t packet, uint8_t* BMSId, uint32_t* uid);

#endif /* INC_BMS_CAN_MESSAGES_H_ */
