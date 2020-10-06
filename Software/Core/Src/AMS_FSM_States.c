/*
 * AMS_FSM_States.c
 *
 *  Created on: Oct 5, 2020
 *      Author: Thomas Fraser
 */

#include "AMS_FSM_States.h"

// Dead State, IE nothing
state_t deadState = {&state_dead_enter, &state_dead_update, &state_dead_exit, "Dead_s"};

void state_dead_enter(fsm_t *fsm)
{
	return;
}

void state_dead_update(fsm_t *fsm)
{
	Error_Handler();
	return;
}

void state_dead_exit(fsm_t *fsm)
{
	return;
}

// Idle State, HeartBeat + waiting for RTD CAN
state_t idleState = {&state_idle_enter, &state_idle_update, &state_idle_exit, "Idle_s"};

void state_idle_enter(fsm_t *fsm)
{
	//TODO, first time startup
	AMS_GlobalState_t *AMS_GlobalState = malloc(sizeof(AMS_GlobalState_t));
	memset(AMS_GlobalState, 0, sizeof(AMS_GlobalState_t));

	//Set Initial PROFET Pin Positions
	HAL_GPIO_WritePin(HVA_N_GPIO_Port, HVA_N_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HVB_N_GPIO_Port, HVB_N_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PRECHG_GPIO_Port, PRECHG_Pin, GPIO_PIN_SET);

	// LOW - HVA+, HVB+
	HAL_GPIO_WritePin(HVA_P_GPIO_Port, HVA_P_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVB_P_GPIO_Port, HVB_P_Pin, GPIO_PIN_RESET);

	// FAN
	HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_SET);

	// BMS Control - HIGH (Turn on all BMS)
	HAL_GPIO_WritePin(BMS_CTRL_GPIO_Port, BMS_CTRL_Pin, GPIO_PIN_SET);

	// ALARM Line - HIGH
	HAL_GPIO_WritePin(ALARM_CTRL_GPIO_Port, ALARM_CTRL_Pin, GPIO_PIN_SET);
}

void state_idle_update(fsm_t *fsm)
{
	//TODO, check for RTD, send Heartbeat, query BMSs
}

void state_idle_exit(fsm_t *fsm)
{
	//TODO, send RTD heath check
}

// Precharge State
state_t prechargeState = {&state_precharge_enter, &state_precharge_update, &state_precharge_exit, "Precharge_s"};

void state_precharge_enter(fsm_t *fsm)
{
	//TODO, setup timer for precharge, perform precharge
	prechargeTimer = osTimerNew(&prechargeTimer_cb, osTimerOnce, fsm, NULL);
	if(osTimerStart(prechargeTimer, PRECHARGE_DELAY * MStoTICKS) != osOK)
	{
		Error_Handler();
	}
}

void state_precharge_update(fsm_t *fsm)
{
	return; // In precharge state, wait for timer cb.
}

void state_precharge_exit(fsm_t *fsm)
{
	//TODO, delete timer.
	if(osTimerDelete(prechargeTimer) != osOK)
	{
		Error_Handler();
	}
}

// Callback to change the state of the FSM to driving once the timer has completed
void prechargeTimer_cb(void *fsm)
{
	fsm_changeState((fsm_t *)fsm, &drivingState);
}

// Driving State, Heartbeat + checking BMS + SoC + Check 4 soft shutdown
state_t drivingState = {&state_driving_enter, &state_driving_update, &state_driving_exit, "Driving_s"};

void state_driving_enter(fsm_t *fsm)
{
	//TODO, Close Connectors, Heartbeat, RTD Health(?)
}

void state_driving_update(fsm_t *fsm)
{
	//TODO, Heartbeat, RTD Health(?), query BMSs
}

void state_driving_exit(fsm_t *fsm)
{
	//TODO, Open connectors, broadcast state
}
