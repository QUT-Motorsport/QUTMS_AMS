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

state_t state_ready = { &state_ready_enter, &state_ready_body, AMS_STATE_READY };

void state_ready_enter(fsm_t *fsm) {

}

void state_ready_body(fsm_t *fsm) {

}
