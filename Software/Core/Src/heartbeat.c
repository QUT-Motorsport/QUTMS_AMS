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

uint32_t ams_heartbeat_timer_start;

void setup_heartbeat() {
	// send heartbeat every 100ms
	timer_heartbeat = timer_init(100, true, heartbeat_timer_cb);

	// setup constants
	heartbeats.heartbeat_timeout = HEARTBEAT_TIMEOUT;

	// reset heartbeat timers to default
	heartbeat_timeout_reset();

	// start timer
	timer_start(&timer_heartbeat);

	ams_heartbeat_timer_start = HAL_GetTick();
}

void heartbeat_timer_cb(void *args) {
	AMS_Heartbeat_t msg = Compose_AMS_Heartbeat(&AMS_heartbeatState);
	CAN_TxHeaderTypeDef header = { .ExtId = msg.id, .IDE =
	CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC = sizeof(msg.data),
			.TransmitGlobalTime = DISABLE };

	// send heartbeat on all CAN lines
	AMS_send_can_msg(&CANBUS2, &header, msg.data);
	AMS_send_can_msg(&CANBUS4, &header, msg.data);

	if ((HAL_GetTick() - ams_heartbeat_timer_start) > HEARTBEAT_PRINT_TIME) {
		ams_heartbeat_timer_start = HAL_GetTick();

		printf("HB: State: 0x%02X, Flags: 0x%04X, BMS: 0x%04X\r\n", AMS_heartbeatState.stateID, AMS_heartbeatState.flags.rawMem, AMS_heartbeatState.bmsStatus);
	}
}

void heartbeat_timeout_reset() {

}

bool check_heartbeat_msg(CAN_MSG_Generic_t *msg) {
	bool hb_message = false;

	uint8_t idx = (msg->ID & 0xF);
	uint32_t masked_id = (msg->ID & ~0xF);

	if (masked_id == CC_Heartbeat_ID) {
		hb_message = true;

		heartbeats.hb_CC_start = HAL_GetTick();
		heartbeats.CC = true;

		// have heartbeat so clear error flag if it's set
		AMS_heartbeatState.flags.HB_CC = 0;

		Parse_CC_Heartbeat(msg->data, &CC_heartbeatState);
	}

	// check all BMS boards for valid heartbeats
	bool bms_hb_good = true;

	for (int i = 0; i < BMS_COUNT; i++) {
		if (!heartbeats.BMS[i]) {
			bms_hb_good = false;
			break;
		}
	}

	if (bms_hb_good) {
		// valid heartbeat for all BMS boards so clear error flag if it's set
		AMS_heartbeatState.flags.HB_BMS = 0;
	}

	return hb_message;
}

bool check_CAN2_heartbeat() {
	bool success = true;

	// CC
	if ((HAL_GetTick() - heartbeats.hb_CC_start)
			> heartbeats.heartbeat_timeout) {
		heartbeats.CC = false;
		AMS_heartbeatState.flags.HB_CC = 1;
		success = false;
	}

	return success;
}

bool check_bms_heartbeat() {
	bool success = true;

	// BMS
	for (int i = 0; i < BMS_COUNT; i++) {
		if ((HAL_GetTick() - heartbeats.hb_BMS_start[i])
				> heartbeats.heartbeat_timeout) {
			heartbeats.BMS[i] = false;
			AMS_heartbeatState.flags.HB_BMS = 1;
			success = false;
		}
	}

	return success;
}

bool check_sendyne_heartbeat() {
	bool success = true;

	if ((HAL_GetTick() - heartbeats.hb_SENDYNE1_start)
			> heartbeats.heartbeat_timeout) {
		heartbeats.SENDYNE1 = false;
		AMS_heartbeatState.flags.HB_SENDYNE1 = 1;
		success = false;
	}

	if ((HAL_GetTick() - heartbeats.hb_SENDYNE2_start)
			> heartbeats.heartbeat_timeout) {
		heartbeats.SENDYNE2 = false;
		AMS_heartbeatState.flags.HB_SENDYNE2 = 1;
		success = false;
	}

	return success;
}
