/*
 * states_init.c
 *
 *  Created on: 31 Dec. 2021
 *      Author: Calvin
 */

#include "states.h"
#include "can_dict.h"
#include "heartbeat.h"
#include "profet.h"
#include "bms.h"
#include "sendyne.h"

state_t state_start = { &state_start_enter, &state_start_body, AMS_STATE_START };
state_t state_initPeripherals = { &state_initPeripherals_enter,
		&state_initPeripherals_body, AMS_STATE_INIT_PERIPHERAL };
state_t state_initBMS = { &state_initBMS_enter, &state_initBMS_body,
		AMS_STATE_INIT_BMS };
state_t state_initCAN4 = { &state_initCAN4_enter, &state_initCAN4_body,
		AMS_STATE_INIT_CAN4 };
state_t state_checkBMS = { &state_checkBMS_enter, &state_checkBMS_body,
		AMS_STATE_CHECK_BMS };
state_t state_checkSendyne = { &state_checkSendyne_enter,
		&state_checkSendyne_body, AMS_STATE_CHECK_SENDYNE };
state_t state_error = { &state_error_enter, &state_error_body, AMS_STATE_ERROR };

uint32_t peripheral_retry_start = 0;
uint32_t peripheral_timeout_start = 0;

int init_bms_count = 0;
uint32_t bms_boot_start = 0;

int init_CAN_count = 0;
uint32_t CAN_init_start = 0;

void state_start_enter(fsm_t *fsm) {
	// init object dictionary
	AMS_OD_init();

	AMS_hbState.flags.rawMem = 0;

	// make sure BMS line is off
	bms_ctrl_off();

	// open all contactors
	profet_open_all();

	// set counters
	init_bms_count = 0;

	// go to peripheral init
	fsm_changeState(fsm, &state_initPeripherals, "Init Peripherals");
	return;
}

void state_start_body(fsm_t *fsm) {
	return;
}

void state_initPeripherals_enter(fsm_t *fsm) {
	bool success = true;

	// setup CAN message queues
	setup_CAN();

	if (!init_CAN2()) {
		AMS_hbState.flags.P_CAN2 = 1;
		success = false;
	} else {
		// CAN2 has started successfully so we're good to start heartbeats
		setup_heartbeat();
	}

	if (success) {
		fsm_changeState(fsm, &state_initBMS, "Peripherals initialized");
		peripheral_retry_start = 0;
		peripheral_timeout_start = 0;
		return;
	} else {
		// something failed, so start timers so we can retry
		peripheral_retry_start = HAL_GetTick();
		peripheral_timeout_start = HAL_GetTick();
	}
}

void state_initPeripherals_body(fsm_t *fsm) {
	if (peripheral_timeout_start == 0) {
		// everything is initialized so skip this iteration
		return;
	}

	// if we're here, something didn't initialize
	if ((peripheral_retry_start - HAL_GetTick()) > PERIPHERAL_RETRY) {
		uint8_t success = 0;
		if (AMS_hbState.flags.P_CAN2 == 1) {
			success |= (1 << 0);

			// retry CAN
			if (init_CAN2()) {
				success &= ~(1 << 0);
				AMS_hbState.flags.P_CAN2 = 0;

				// CAN has started successfully so heartbeat machine go brr
				setup_heartbeat();
			}
		}

		if (success == 0) {
			// everything has initialized correctly

			// disable timeout
			peripheral_timeout_start = 0;

			fsm_changeState(fsm, &state_initBMS, "Peripherals initialized");
			return;
		} else {
			// something failed, so lets retry again in 100ms
			peripheral_retry_start = HAL_GetTick();
		}
	}

	if ((peripheral_timeout_start - HAL_GetTick()) > PERIPHERAL_TIMEOUT) {
		// something is clearly broken and hasn't been fixed so go to error state
		fsm_changeState(fsm, &state_error, "Peripherals failed");
		return;
	}
}

void state_initBMS_enter(fsm_t *fsm) {
	if (init_bms_count > MAX_BMS_INIT_TRY) {
		// BMS initialization not working, go to error state
		fsm_changeState(fsm, &state_error, "BMS init failed");
		return;
	}

	printf("BMS init #%d\r\n", init_bms_count);

	init_bms_count++;

	// turn on BMS
	bms_ctrl_on();

	// wait 75ms for all BMS to boot and init their CAN
	bms_boot_start = HAL_GetTick();

	return;
}

void state_initBMS_body(fsm_t *fsm) {
	CAN_MSG_Generic_t msg;
	while (queue_next(&queue_CAN, &msg)) {
		// check for heartbeats
		if (check_heartbeat_msg(&msg)) {

		}
	}

	// don't care about return value of this until we're in ready mode
	// could be about to charge where accumulator is not in car
	check_CAN2_heartbeat();

	// wait 75ms for all BMS to boot and init their CAN
	if ((HAL_GetTick() - bms_boot_start) > BMS_BOOT_TIME) {
		bms_boot_start = 0;

		// turn ctrl line off to not burn out BMS resistors
		bms_ctrl_off();

		fsm_changeState(fsm, &state_initCAN4, "BMS power on");
		return;
	}
}

void state_initCAN4_enter(fsm_t *fsm) {
	if (!init_CAN4()) {
		AMS_hbState.flags.P_CAN4 = 1;
		CAN_init_start = HAL_GetTick();
		init_CAN_count = 0;
	} else {
		AMS_hbState.flags.P_CAN4 = 0;
		fsm_changeState(fsm, &state_checkBMS, "CAN4 initialized");
		return;
	}
}

void state_initCAN4_body(fsm_t *fsm) {
	CAN_MSG_Generic_t msg;
	while (queue_next(&queue_CAN, &msg)) {
		// check for heartbeats
		if (check_heartbeat_msg(&msg)) {

		}
	}

	// don't care about return value of this until we're in ready mode
	// could be about to charge where accumulator is not in car
	check_CAN2_heartbeat();

	// retry init CAN4 until fail
	if (AMS_hbState.flags.P_CAN4 == 1) {
		// CAN failed to init
		if ((HAL_GetTick() - CAN_init_start) > CAN_RETRY_TIME) {
			printf("CAN init #%d\r\n", init_CAN_count);

			if (!init_CAN4()) {
				AMS_hbState.flags.P_CAN4 = 1;
				CAN_init_start = HAL_GetTick();
				init_CAN_count++;
			} else {
				AMS_hbState.flags.P_CAN4 = 0;
				fsm_changeState(fsm, &state_checkBMS, "CAN4 initialized");
				return;
			}
		}
	}

	// if failed 5 times, go back to init BMS
	if (init_CAN_count > MAX_CAN_INIT_TRY) {
		fsm_changeState(fsm, &state_initBMS, "CAN4 failed to init");
		return;
	}
}

void state_checkBMS_enter(fsm_t *fsm) {
	// CAN4 is working properly now, so clear failure counter
	init_bms_count = 0;
	bms_setup();

}

void state_checkBMS_body(fsm_t *fsm) {
	CAN_MSG_Generic_t msg;
	while (queue_next(&queue_CAN, &msg)) {
		// check for heartbeats
		if (check_heartbeat_msg(&msg)) {

		}
	}

	// don't care about return value of this until we're in ready mode
	// could be about to charge where accumulator is not in car
	check_CAN2_heartbeat();

	check_bms_heartbeat();

	bool BMS_missing = false;

	for (int i = 0; i < BMS_COUNT; i++) {
		if (!heartbeats.BMS[i]) {
			BMS_missing = true;
		}
	}

	if (!BMS_missing) {
		fsm_changeState(fsm, &state_checkSendyne, "BMS Present");
	}
}

void state_checkSendyne_enter(fsm_t *fsm) {
	// start sendyne timers
	setup_sendyne();
}

void state_checkSendyne_body(fsm_t *fsm) {

	CAN_MSG_Generic_t msg;
	while (queue_next(&queue_CAN, &msg)) {
		// check for heartbeats
		if (check_heartbeat_msg(&msg)) {

		} else if (msg.ID == AMS_StartCharging_ID) {
			// charge mode has been requested, so switch into charge mode
			fsm_changeState(fsm, &state_charging, "Starting charging");
			return;
		}
	}

	// don't care about return value of this until we're in ready mode
	// could be about to charge where accumulator is not in car
	check_CAN2_heartbeat();

	check_bms_heartbeat();
	check_sendyne_heartbeat();

	bool sendyne_missing = false;

	if (!heartbeats.SENDYNE1) {
		sendyne_missing = true;
	}

	if (!heartbeats.SENDYNE2) {
		sendyne_missing = true;
	}

	if (!sendyne_missing) {
		fsm_changeState(fsm, &state_ready, "Sendyne Present");
		return;
	}
}

void state_error_enter(fsm_t *fsm) {

}

void state_error_body(fsm_t *fsm) {

}
