/*
 * AMS_FSM_States.h
 *
 *  Created on: Oct 5, 2020
 *      Author: Thomas Fraser
 */

#ifndef INC_AMS_FSM_STATES_H_
#define INC_AMS_FSM_STATES_H_

#include <fsm.h>
#include "main.h"
#include "cmsis_os.h"
#include <memory.h>

typedef struct
{
	//TODO, what's in the AMS global state?
	// Sounds like an Alistair & Calvin Problem to me
	osTimerId_t heartbeatTimer;
} AMS_GlobalState_t;

AMS_GlobalState_t *AMS_GlobalState;

void heartbeatTimer_cb(void *fsm);

void state_dead_enter(fsm_t *fsm);
void state_dead_iterate(fsm_t *fsm);
void state_dead_exit(fsm_t *fsm);
state_t deadState;

void state_idle_enter(fsm_t *fsm);
void state_idle_iterate(fsm_t *fsm);
void state_idle_exit(fsm_t *fsm);
state_t idleState;

void state_precharge_enter(fsm_t *fsm);
void state_precharge_iterate(fsm_t *fsm);
void state_precharge_exit(fsm_t *fsm);
void prechargeTimer_cb(void *fsm);
state_t prechargeState;
osTimerId_t prechargeTimer;

void state_driving_enter(fsm_t *fsm);
void state_driving_iterate(fsm_t *fsm);
void state_driving_exit(fsm_t *fsm);
state_t drivingState;

void state_error_enter(fsm_t *fsm);
void state_error_iterate(fsm_t *fsm);
void state_error_exit(fsm_t *fsm);
state_t errorState;

#endif /* INC_AMS_FSM_STATES_H_ */
