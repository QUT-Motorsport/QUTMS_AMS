/*
 * states_init.c
 *
 *  Created on: 31 Dec. 2021
 *      Author: Calvin
 */


#include "states.h"
#include "can_dict.h"
#include "heartbeat.h"

state_t state_start = { &state_start_enter, &state_start_body, AMS_STATE_START };
state_t state_initPeripherals = { &state_initPeripherals_enter, &state_initPeripherals_body, AMS_STATE_INIT_PERIPHERAL };
state_t state_initBMS = { &state_initBMS_enter, &state_initBMS_body, AMS_STATE_INIT_BMS };
state_t state_initCAN4 = { &state_initCAN4_enter, &state_initCAN4_body, AMS_STATE_INIT_CAN4 };
state_t state_checkBMS = { &state_checkBMS_enter, &state_checkBMS_body, AMS_STATE_CHECK_BMS };
state_t state_checkSendyne = { &state_checkSendyne_enter, &state_checkSendyne_body, AMS_STATE_CHECK_SENDYNE };
state_t state_error = { &state_error_enter, &state_error_body, AMS_STATE_ERROR };


void state_start_enter(fsm_t *fsm) {
	// init object dictionary
	AMS_OD_init();

	AMS_heartbeatState.flags.rawMem = 0;

	// go to peripheral init
	fsm_changeState(fsm, &state_initPeripherals, "Init Peripherals");
	return;
}

void state_start_body(fsm_t *fsm) {
	return;
}

void state_initPeripherals_enter(fsm_t *fsm) {

}

void state_initPeripherals_body(fsm_t *fsm) {

}

void state_initBMS_enter(fsm_t *fsm) {

}

void state_initBMS_body(fsm_t *fsm) {

}

void state_initCAN4_enter(fsm_t *fsm) {

}

void state_initCAN4_body(fsm_t *fsm) {

}

void state_checkBMS_enter(fsm_t *fsm) {

}

void state_checkBMS_body(fsm_t *fsm) {

}

void state_checkSendyne_enter(fsm_t *fsm) {

}

void state_checkSendyne_body(fsm_t *fsm) {

}

void state_error_enter(fsm_t *fsm) {

}

void state_error_body(fsm_t *fsm) {

}
