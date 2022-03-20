/*
 * states_charging.c
 *
 *  Created on: 31 Dec. 2021
 *      Author: Calvin
 */

#include "states.h"
#include "heartbeat.h"
#include "profet.h"
#include "bms.h"
#include "sendyne.h"
#include "shutdown.h"

#include <CAN_AMS.h>
#include <CAN_VCU.h>
#include <CAN_SHDN.h>

#include <math.h>

state_t state_charging_ready = { &state_charging_ready_enter,
		&state_charging_ready_body, AMS_STATE_CHARGING_READY };
state_t state_charging_precharge = { &state_charging_precharge_enter,
		&state_charging_precharge_body, AMS_STATE_CHARGING_PRECHARGE };
state_t state_charging_tsActive = { &state_charging_tsActive_enter,
		&state_charging_tsActive_body, AMS_STATE_CHARGING_TS_ACTIVE };

uint32_t precharge_start_time; // NOTE: will this override?

void state_charging_ready_enter(fsm_t *fsm) {
	profet_open_all();
}

void state_charging_ready_body(fsm_t *fsm) {
	CAN_MSG_Generic_t msg;

	while (queue_next(&queue_CAN, &msg)) {
		// check for heartbeats
		if (check_heartbeat_msg(&msg)) {

		}
	}

	check_CAN2_heartbeat();
	check_sendyne_heartbeat();
	check_CHRG_heartbeat();

	if (CHRG_CTRL_hbState.stateID == CHRGCTRL_STATE_PRECHARGE_REQUEST) {
		// precharge has been requested, so start
		fsm_changeState(fsm, &state_charging_precharge, "Precharge requested");
		return;
	}
}

void state_charging_precharge_enter(fsm_t *fsm) {
	// set contactors to precharge state
	profet_precharge();

	precharge_start_time = HAL_GetTick();

	// clear precharge timeout flag as precharge is starting
	AMS_hbState.flags.PCHRG_TIMEOUT = 0;
}

void state_charging_precharge_body(fsm_t *fsm) {
	CAN_MSG_Generic_t msg;

	while (queue_next(&queue_CAN, &msg)) {
		// check for heartbeats
		if (check_heartbeat_msg(&msg)) {

		}
	}

	check_CAN2_heartbeat();
	check_sendyne_heartbeat();
	check_CHRG_heartbeat();

	if ((HAL_GetTick() - precharge_start_time) > CHARGING_PRECHARGE_TIME) {
		// precharge has completed
		fsm_changeState(fsm, &state_charging_tsActive, "Precharge complete");
		return;
	}
}

void state_charging_tsActive_enter(fsm_t *fsm) {
	// close contactors for TS
	profet_TS_active();
}

void state_charging_tsActive_body(fsm_t *fsm) {
	CAN_MSG_Generic_t msg;

	while (queue_next(&queue_CAN, &msg)) {
		// check for heartbeats
		if (check_heartbeat_msg(&msg)) {

		}
	}

	check_CAN2_heartbeat();
	check_sendyne_heartbeat();
	check_CHRG_heartbeat();

	if (CHRG_CTRL_hbState.stateID == CHRGCTRL_STATE_STOP_CHARGE) {
		// charge controller says finished charging
		fsm_changeState(fsm, &state_charging_ready, "Charging finished");
		return;
	}
}
