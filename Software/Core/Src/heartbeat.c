/*
 * heartbeat.c
 *
 *  Created on: Dec 30, 2021
 *      Author: Calvin
 */


#include "heartbeat.h"
#include "can.h"

heartbeat_states_t heartbeats;

AMS_HeartbeatState_t AMS_heartbeatState;
CC_HeartbeatState_t CC_heartbeatState;

ms_timer_t timer_heartbeat;

void setup_heartbeat() {
	// send heartbeat every 100ms
	timer_heartbeat = timer_init(100, true, heartbeat_timer_cb);

	// setup constants
	heartbeats.heartbeat_timeout = HEARTBEAT_TIMEOUT;

	// reset heartbeat timers to default
	heartbeat_timeout_reset();

	// start timer
	timer_start(&timer_heartbeat);
}

void heartbeat_timer_cb(void *args) {
	AMS_Heartbeat_t msg = Compose_AMS_Heartbeat(&AMS_heartbeatState);
	CAN_TxHeaderTypeDef header = { .ExtId = msg.id, .IDE =
	CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC = sizeof(msg.data), .TransmitGlobalTime = DISABLE };

	// send heartbeat on all CAN lines
	AMS_send_can_msg(&CANBUS2, &header, msg.data);
	AMS_send_can_msg(&CANBUS4, &header, msg.data);
}
