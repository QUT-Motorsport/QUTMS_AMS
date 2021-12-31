/*
 * states.h
 *
 *  Created on: 31 Dec. 2021
 *      Author: Calvin
 */

#ifndef INC_STATES_H_
#define INC_STATES_H_

#include <FSM.h>

void state_start_enter(fsm_t *fsm);
void state_start_body(fsm_t *fsm);
extern state_t state_start;

void state_initPeripherals_enter(fsm_t *fsm);
void state_initPeripherals_body(fsm_t *fsm);
extern state_t state_initPeripherals;

void state_initBMS_enter(fsm_t *fsm);
void state_initBMS_body(fsm_t *fsm);
extern state_t state_initBMS;

void state_initCAN4_enter(fsm_t *fsm);
void state_initCAN4_body(fsm_t *fsm);
extern state_t state_initCAN4;

void state_checkBMS_enter(fsm_t *fsm);
void state_checkBMS_body(fsm_t *fsm);
extern state_t state_checkBMS;

void state_checkSendyne_enter(fsm_t *fsm);
void state_chekcSendyne_body(fsm_t *fsm);
extern state_t state_checkSendyne;

void state_ready_enter(fsm_t *fsm);
void state_ready_body(fsm_t *fsm);
extern state_t state_ready;

void state_charging_enter(fsm_t *fsm);
void state_charging_body(fsm_t *fsm);
extern state_t state_charging;

void state_precharge_enter(fsm_t *fsm);
void state_precharge_body(fsm_t *fsm);
extern state_t state_precharge;

void state_tsActive_enter(fsm_t *fsm);
void state_tsActive_body(fsm_t *fsm);
extern state_t state_tsActive;

void state_shutdown_enter(fsm_t *fsm);
void state_shutdown_body(fsm_t *fsm);
extern state_t state_shutdown;

void state_error_enter(fsm_t *fsm);
void state_error_body(fsm_t *fsm);
extern state_t state_error;

#endif /* INC_STATES_H_ */
