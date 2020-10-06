/*
 * fsm.h
 *
 *  Created on: Oct 5, 2020
 *      Author: Thomas Fraser
 */

#ifndef INC_FSM_H_
#define INC_FSM_H_

#include "main.h"
#include <stdlib.h>
#include <memory.h>
#include "cmsis_os.h"

typedef struct state state_t;
typedef struct fsm fsm_t;

typedef void (*fsm_function)(fsm_t*);

struct state {
	fsm_function enter;
	fsm_function iter;
	fsm_function exit;
	char *stateName;
};

struct fsm {
	state_t *currentState;
	osSemaphoreId_t sem;
	osSemaphoreId_t updating;
};

extern fsm_t *stateMachine;

// Create new FSM
fsm_t *fsm_new(state_t *beginState);

//Update FSM
void fsm_update(fsm_t *fsm);

//Change FSM state
void fsm_changeState(fsm_t *fsm, state_t *newState);

//Get current FSM state
state_t *fsm_getState_t(fsm_t *fsm);

//Get current FSM state
char* fsm_getState(fsm_t *fsm);

//Reset FSM
void fsm_reset(fsm_t *fsm, state_t *resetState);

//Log FSM
void fsm_log(fsm_t *fsm);

//Delete FSM
void fsm_delete(fsm_t *fsm);

#endif /* INC_FSM_H_ */
