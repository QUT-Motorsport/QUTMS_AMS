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

state_t state_charging = { &state_charging_enter, &state_charging_body, AMS_STATE_READY };

void state_charging_enter(fsm_t *fsm) {

}

void state_charging_body(fsm_t *fsm) {

}
