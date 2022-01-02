/*
 * sendyne.c
 *
 *  Created on: Jan 2, 2022
 *      Author: Calvin
 */

#include "sendyne.h"
#include "CAN_Sendyne.h"
#include "can.h"

ms_timer_t timer_sendyne;
sendyne_state_t sendyne;

void sendyne_handleCurrent(CAN_MSG_Generic_t *msg) {

}

void sendyne_handleVoltage(CAN_MSG_Generic_t *msg) {

}

void setup_sendyne() {

}

void sendyne_timer_update() {

}

void sendyne_CAN_timer_cb(void *args) {
	CAN_MSG_Generic_t msg;
	while (queue_next(&queue_CAN_SENDYNE, &msg)) {

	}
}

void sendyne_voltage_timer_cb(void *args) {
	Sendyne_RequestData_t msg = Compose_Sendyne_RequestData(1, CS_V1);

	CAN_TxHeaderTypeDef header = { .ExtId = msg.id, .IDE =
	CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC = sizeof(msg.data),
			.TransmitGlobalTime = DISABLE };

	// send request
	AMS_send_can_msg(&CANBUS4, &header, msg.data);
}

void sendyne_current_timer_cb(void *args) {
	Sendyne_RequestData_t msg = Compose_Sendyne_RequestData(1, CS_CURRENT);

	CAN_TxHeaderTypeDef header = { .ExtId = msg.id, .IDE =
	CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC = sizeof(msg.data),
			.TransmitGlobalTime = DISABLE };

	// send request
	AMS_send_can_msg(&CANBUS4, &header, msg.data);

	msg = Compose_Sendyne_RequestData(2, CS_CURRENT);
	header.ExtId = msg.id;

	// send request
	AMS_send_can_msg(&CANBUS4, &header, msg.data);
}
