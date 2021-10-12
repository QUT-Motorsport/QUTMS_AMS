/*
 * AMS_FSM_Dead.c
 *
 *  Created on: 6 Jul 2021
 *      Author: Thomas Fraser
 */

#include <AMS_FSM_States.h>

state_t deadState = { &state_dead_enter, &state_dead_iterate, &state_dead_exit,
		"Dead_s" };

void state_dead_enter(fsm_t *fsm) {
	debugCAN_enterState(AMS_STATE_ID_Dead);
	return;
}

void state_dead_iterate(fsm_t *fsm) {
	Error_Handler();
	return;
}

void state_dead_exit(fsm_t *fsm) {
	debugCAN_exitState(AMS_STATE_ID_Dead);
	return;
}
