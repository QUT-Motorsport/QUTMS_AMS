/*
 * sendyne.c
 *
 *  Created on: Jan 2, 2022
 *      Author: Calvin
 */

#include "sendyne.h"

#include <CAN_Sendyne.h>

#include "can.h"
#include "heartbeat.h"

ms_timer_t timer_sendyne_CAN;
ms_timer_t timer_sendyne_voltage;
ms_timer_t timer_sendyne_current;

sendyne_state_t sendyne;

void setup_sendyne() {
	timer_sendyne_CAN = timer_init(5, true, sendyne_CAN_timer_cb);
	timer_sendyne_voltage = timer_init(200, true, sendyne_voltage_timer_cb);
	timer_sendyne_current = timer_init(200, true, sendyne_current_timer_cb);

	timer_start(&timer_sendyne_CAN);
	timer_start(&timer_sendyne_voltage);
	timer_start(&timer_sendyne_current);
}

void sendyne_handleCurrent(CAN_MSG_Generic_t *msg) {
	if (msg->ID == CS_1_RESPONSE_EXTID) {
		// HVA
		sendyne.HVACurrent_uA = 0;
		sendyne.HVACurrent_uA |= (int32_t)msg->data[1] << 24;
		sendyne.HVACurrent_uA |= (int32_t)msg->data[2] << 16;
		sendyne.HVACurrent_uA |= (int32_t)msg->data[3] << 8;
		sendyne.HVACurrent_uA |= (int32_t)msg->data[4] << 0;

		sendyne.HVACurrent = sendyne.HVACurrent_uA / SENDYNE_CURRENT_SCALE;
	} else if (msg->ID == CS_2_RESPONSE_EXTID) {
		// HVA
		sendyne.HVBCurrent_uA = 0;
		sendyne.HVBCurrent_uA |= (int32_t)msg->data[1] << 24;
		sendyne.HVBCurrent_uA |= (int32_t)msg->data[2] << 16;
		sendyne.HVBCurrent_uA |= (int32_t)msg->data[3] << 8;
		sendyne.HVBCurrent_uA |= (int32_t)msg->data[4] << 0;

		sendyne.HVBCurrent = sendyne.HVBCurrent_uA / SENDYNE_CURRENT_SCALE;
	}
}

void sendyne_handleVoltage(CAN_MSG_Generic_t *msg) {
	sendyne.voltage_uV = 0;
	sendyne.voltage_uV |= (int32_t)msg->data[1] << 24;
	sendyne.voltage_uV |= (int32_t)msg->data[2] << 16;
	sendyne.voltage_uV |= (int32_t)msg->data[3] << 8;
	sendyne.voltage_uV |= (int32_t)msg->data[4] << 0;

	sendyne.voltage = sendyne.voltage_uV / SENDYNE_VOLTAGE_SCALE;
	AMS_heartbeatState.voltage = sendyne.voltage;
}



void sendyne_timer_update() {
	timer_update(&timer_sendyne_CAN, NULL);
	timer_update(&timer_sendyne_voltage, NULL);
	timer_update(&timer_sendyne_current, NULL);
}

void sendyne_CAN_timer_cb(void *args) {
	CAN_MSG_Generic_t msg;
	while (queue_next(&queue_CAN_SENDYNE, &msg)) {
		if (msg.ID == CS_1_RESPONSE_EXTID) {
			heartbeats.hb_SENDYNE1_start = HAL_GetTick();
			heartbeats.SENDYNE1 = true;
			AMS_heartbeatState.flags.HB_SENDYNE1 = 0;

			if (msg.data[0] == CS_V1) {
				sendyne_handleVoltage(&msg);
			} else if (msg.data[0] == CS_CURRENT) {
				sendyne_handleCurrent(&msg);
			}

		} else if (msg.ID == CS_2_RESPONSE_EXTID) {
			heartbeats.hb_SENDYNE2_start = HAL_GetTick();
			heartbeats.SENDYNE2 = true;
			AMS_heartbeatState.flags.HB_SENDYNE2 = 0;

			if (msg.data[0] == CS_CURRENT) {
				sendyne_handleCurrent(&msg);
			}
		}
	}
}

void sendyne_voltage_timer_cb(void *args) {
	Sendyne_RequestData_t msg = Compose_Sendyne_RequestData(0, CS_V1);

	CAN_TxHeaderTypeDef header = { .ExtId = msg.id, .IDE =
	CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC = sizeof(msg.data),
			.TransmitGlobalTime = DISABLE };

	// send request
	AMS_send_can_msg(&CANBUS4, &header, msg.data);
}

void sendyne_current_timer_cb(void *args) {
	Sendyne_RequestData_t msg = Compose_Sendyne_RequestData(0, CS_CURRENT);

	CAN_TxHeaderTypeDef header = { .ExtId = msg.id, .IDE =
	CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC = sizeof(msg.data),
			.TransmitGlobalTime = DISABLE };

	// send request
	AMS_send_can_msg(&CANBUS4, &header, msg.data);

	HAL_Delay(1);

	msg = Compose_Sendyne_RequestData(1, CS_CURRENT);
	header.ExtId = msg.id;

	// send request
	AMS_send_can_msg(&CANBUS4, &header, msg.data);
}
