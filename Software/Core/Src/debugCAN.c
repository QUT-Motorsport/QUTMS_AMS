/*
 * debugCAN.c
 *
 *  Created on: Oct 12, 2021
 *      Author: Calvin
 */
#include "debugCAN.h"
#include <CAN_Debug.h>
#include "can.h"

void debugCAN_enterState(uint8_t stateID) {
	uint32_t CANmailbox = 0;

	DEBUG_EnterState_t msg = Compose_DEBUG_EnterState(CAN_SRC_ID_AMS, 0, stateID);

	CAN_TxHeaderTypeDef header = { .ExtId = msg.id, .IDE = CAN_ID_EXT,
				.RTR = CAN_RTR_DATA, .DLC = sizeof(msg.data), .TransmitGlobalTime = DISABLE, };

	HAL_CAN_AddTxMessage(&CANBUS2, &header, msg.data,
				&CANmailbox);
}

void debugCAN_exitState(uint8_t stateID) {
	uint32_t CANmailbox = 0;

	DEBUG_ExitState_t msg = Compose_DEBUG_ExitState(CAN_SRC_ID_AMS, 0, stateID);

	CAN_TxHeaderTypeDef header = { .ExtId = msg.id, .IDE = CAN_ID_EXT,
				.RTR = CAN_RTR_DATA, .DLC = sizeof(msg.data), .TransmitGlobalTime = DISABLE, };

	HAL_CAN_AddTxMessage(&CANBUS2, &header, msg.data,
				&CANmailbox);
}

