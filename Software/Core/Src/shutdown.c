/*
 * shutdown.c
 *
 *  Created on: 12 Jan. 2022
 *      Author: Calvin J
 */

#include "shutdown.h"
#include "states.h"

#include <CAN_AMS.h>
#include <CAN_VCU.h>
#include <CAN_SHDN.h>

#include "profet.h"
#include "heartbeat.h"

bool shutdown_triggered;

state_t state_trig_shutdown = { &state_trig_shutdown_enter,
		&state_trig_shutdown_body, AMS_STATE_TRIG_SHUTDOWN };
state_t state_shutdown = { &state_shutdown_enter, &state_shutdown_body,
		AMS_STATE_SHUTDOWN };

uint32_t shutdown_rqst_timer_start;

void state_trig_shutdown_enter(fsm_t *fsm) {
	shutdown_rqst_timer_start = HAL_GetTick();

}

void state_trig_shutdown_body(fsm_t *fsm) {
	if ((HAL_GetTick() - shutdown_rqst_timer_start) > SHUTDOWN_BUFFER_PERIOD) {
		fsm_changeState(fsm, &state_shutdown, "Shutdown transition");
		return;
	}
}

void state_shutdown_enter(fsm_t *fsm) {
	profet_open_all();
}

void state_shutdown_body(fsm_t *fsm) {
	bool shutdown_status = false;
	CAN_MSG_Generic_t msg;

	while (queue_next(&queue_CAN, &msg)) {
		// check for heartbeats
		if (check_heartbeat_msg(&msg)) {

		}

		else if (check_shutdown_msg(&msg, &shutdown_status)) {

		}
	}

	check_CAN2_heartbeat();

	check_bms_heartbeat();
	check_sendyne_heartbeat();

	if (shutdown_status) {
		// shutdown is good now, go back to AMS ready
		fsm_changeState(fsm, &state_ready, "Shutdown fixed");
		return;
	}
}

bool check_shutdown_msg(CAN_MSG_Generic_t *msg, bool *shdn_triggered) {
	bool has_msg = false;
	*shdn_triggered = false;

	if ((msg->ID & ~0xF) == VCU_ShutdownStatus_ID) {
		has_msg = true;

		uint8_t line;
		bool status;
		Parse_VCU_ShutdownStatus(msg->data, &line, &line, &line, &line,
				&status);

		if (!status) {
			// shutdown triggered
			*shdn_triggered = true;
		}
	} else if (msg->ID == SHDN_ShutdownTriggered_ID) {
		has_msg = true;

		// shutdown triggered
		*shdn_triggered = true;
	}

	return has_msg;
}
