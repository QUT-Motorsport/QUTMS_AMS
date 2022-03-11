/*
 * states_TS.c
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
#include "vesc_inc.h"

#include <math.h>

state_t state_ready = { &state_ready_enter, &state_ready_body, AMS_STATE_READY };
state_t state_precharge = { &state_precharge_enter, &state_precharge_body,
		AMS_STATE_PRECHARGE };
state_t state_tsActive = { &state_tsActive_enter, &state_tsActive_body,
		AMS_STATE_TS_ACTIVE };

uint32_t precharge_start_time;

void state_ready_enter(fsm_t *fsm) {
	profet_open_all();
}

void state_ready_body(fsm_t *fsm) {
	CAN_MSG_Generic_t msg;

	while (queue_next(&queue_CAN, &msg)) {
		// check for heartbeats
		if (check_heartbeat_msg(&msg)) {

		}

		else if (check_shutdown_msg(&msg, &shutdown_triggered)) {
			if (shutdown_triggered) {
				fsm_changeState(fsm, &state_shutdown, "Shutdown triggered");
				return;
			}
		}

		// TODO: should you be able to start charging with a sendyne plugged in???

		/*
		 else if (msg.ID == AMS_StartCharging_ID) {
		 // charge mode has been requested, so switch into charge mode
		 fsm_changeState(fsm, &state_charging, "Starting charging");
		 return;
		 }
		 */
	}

	// TODO: check dis?
	check_CAN2_heartbeat();

	check_bms_heartbeat();
	check_sendyne_heartbeat();

	if (VCU_CTRL_hbState.stateID == VCU_STATE_SHUTDOWN) {
		fsm_changeState(fsm, &state_shutdown, "VCU CTRL in shutdown");
		return;
	} else if (VCU_CTRL_hbState.stateID == VCU_STATE_PRECHARGE_REQUEST) {
		// chassis controller has requested precharge, so start precharging
		fsm_changeState(fsm, &state_precharge, "Precharge requested");
		return;
	}
}

void state_precharge_enter(fsm_t *fsm) {
	// set contactors to precharge state
	profet_precharge();

	precharge_start_time = HAL_GetTick();

	// clear precharge timeout flag as precharge is starting
	AMS_hbState.flags.PCHRG_TIMEOUT = 0;
}

void state_precharge_body(fsm_t *fsm) {
	CAN_MSG_Generic_t msg;

	while (queue_next(&queue_CAN, &msg)) {
		// check for heartbeats
		if (check_heartbeat_msg(&msg)) {

		}

		else if (check_shutdown_msg(&msg, &shutdown_triggered)) {
			if (shutdown_triggered) {
				fsm_changeState(fsm, &state_shutdown, "Shutdown triggered");
				return;
			}
		} else if (check_vesc_msg(&msg)) {

		}
	}

	// TODO: check dis?
	check_CAN2_heartbeat();

	check_bms_heartbeat();
	check_sendyne_heartbeat();

	// check voltage level

	// determine average brick voltage
	float brick_av_voltage = 0;
	for (int i = 0; i < BMS_COUNT; i++) {
		uint16_t brick_voltage = 0;
		for (int j = 0; j < BMS_VOLT_COUNT; j++) {
			brick_voltage += bms.voltages[i][j];
		}
		// convert mV to V and average
		brick_av_voltage += (brick_voltage / (1000.0f));
	}
	brick_av_voltage = brick_av_voltage / BMS_COUNT;

	// accumulator is 2s4p, so average brick voltage * 2 should be av accumulator voltage
	float av_accumulator_voltage = 2 * brick_av_voltage;

	// check all vescs are reading the correct voltage levle
	bool success = true;


	for (int i = 0; i < NUM_VESC; i++) {
		if ((fabs(vesc_inv.voltage[i] - av_accumulator_voltage) > PRECHARGE_VDIFF) || (vesc_inv.voltage[i] < ACCCUMULATOR_MIN_VOLTAGE) ){
			success = false;
		}
	}

	if (success) {
		// precharge is complete, go to TS ACTIVE
		fsm_changeState(fsm, &state_tsActive, "Precharge complete");
		return;
	}

	/*
	if ((fabs((fabs(sendyne.voltage) - av_accumulator_voltage))
			< PRECHARGE_VDIFF)
			&& (fabs(sendyne.voltage) > ACCCUMULATOR_MIN_VOLTAGE)) {
		// precharge is complete, go to TS ACTIVE
		fsm_changeState(fsm, &state_tsActive, "Precharge complete");
		return;
	}
	*/

	// check precharge time out
	if ((HAL_GetTick() - precharge_start_time) > PRECHARGE_TIME_OUT) {
		AMS_hbState.flags.PCHRG_TIMEOUT = 1;
		fsm_changeState(fsm, &state_ready, "Precharge timed out");
		return;
	}
}

void state_tsActive_enter(fsm_t *fsm) {
	// close contactors for TS
	profet_TS_active();
}

void state_tsActive_body(fsm_t *fsm) {
	CAN_MSG_Generic_t msg;

	while (queue_next(&queue_CAN, &msg)) {
		// check for heartbeats
		if (check_heartbeat_msg(&msg)) {

		}

		else if (check_shutdown_msg(&msg, &shutdown_triggered)) {
			if (shutdown_triggered) {
				fsm_changeState(fsm, &state_shutdown, "Shutdown triggered");
				return;
			}
		}
	}

	// TODO: check dis?
	check_CAN2_heartbeat();

	check_bms_heartbeat();
	check_sendyne_heartbeat();
}

