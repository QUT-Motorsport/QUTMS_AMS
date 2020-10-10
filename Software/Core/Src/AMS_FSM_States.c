/**
 ******************************************************************************
 * @file AMS_FSM_States.c
 * @brief AMS FSM States
 ******************************************************************************
 */

#include <AMS_FSM_States.h>

state_t deadState = {&state_dead_enter, &state_dead_iterate, &state_dead_exit, "Dead_s"};

void state_dead_enter(fsm_t *fsm)
{
	return;
}

void state_dead_iterate(fsm_t *fsm)
{
	Error_Handler();
	return;
}

void state_dead_exit(fsm_t *fsm)
{
	return;
}

state_t idleState = {&state_idle_enter, &state_idle_iterate, &state_idle_exit, "Idle_s"};

void state_idle_enter(fsm_t *fsm)
{
	//TODO, first time startup
	if(AMS_GlobalState == NULL)
	{
		AMS_GlobalState = malloc(sizeof(AMS_GlobalState_t));
		memset(AMS_GlobalState, 0, sizeof(AMS_GlobalState_t));

		// As AMS_GlobalState is accessable across threads, we need to use a semaphore to access it
		AMS_GlobalState->sem = osSemaphoreNew(3U, 3U, NULL);
		if(osSemaphoreAcquire(AMS_GlobalState->sem, MStoTICKS(SEM_ACQUIRE_TIMEOUT)) == osOK)
		{
			AMS_GlobalState->heartbeatTimer = osTimerNew(&heartbeatTimer_cb, osTimerPeriodic, fsm, NULL);
			if(osTimerStart(AMS_GlobalState->heartbeatTimer, MStoTICKS(AMS_HEARTBEAT_PERIOD)) != osOK)
			{
				Error_Handler();
			}
			osSemaphoreRelease(AMS_GlobalState->sem);
		}
	}

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

void state_idle_iterate(fsm_t *fsm)
{
	//TODO, check for RTD, send Heartbeat, query BMSs

	// On RTD, change state
	fsm_changeState(fsm, &prechargeState);
}

void state_idle_exit(fsm_t *fsm)
{
	//TODO, send RTD heath check
}

void heartbeatTimer_cb(void *fsm)
{
	// Send an AMS heartbeat
	return;
}

state_t prechargeState = {&state_precharge_enter, &state_precharge_iterate, &state_precharge_exit, "Precharge_s"};

void state_precharge_enter(fsm_t *fsm)
{
	//TODO, setup timer for precharge, perform precharge
	prechargeTimer = osTimerNew(&prechargeTimer_cb, osTimerOnce, fsm, NULL);
	if(osTimerStart(prechargeTimer, MStoTICKS(PRECHARGE_DELAY)) != osOK)
	{
		Error_Handler();
	}
}

void state_precharge_iterate(fsm_t *fsm)
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

void prechargeTimer_cb(void *fsm)
{
	fsm_changeState((fsm_t *)fsm, &drivingState);
}

state_t drivingState = {&state_driving_enter, &state_driving_iterate, &state_driving_exit, "Driving_s"};

void state_driving_enter(fsm_t *fsm)
{
	//TODO, Close Connectors, Heartbeat, RTD Health(?)

	// PROFET Positions AFTER Precharge
	// HIGH - HVA+, HVA-, HVB+, HVB-
	HAL_GPIO_WritePin(HVA_N_GPIO_Port, HVA_N_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HVB_N_GPIO_Port, HVB_N_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HVA_P_GPIO_Port, HVA_P_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HVB_P_GPIO_Port, HVB_P_Pin, GPIO_PIN_SET);

	// LOW - PRECHG
	HAL_GPIO_WritePin(PRECHG_GPIO_Port, PRECHG_Pin, GPIO_PIN_SET);
}

void state_driving_iterate(fsm_t *fsm)
{
	//TODO, Heartbeat, RTD Health(?), query BMSs
}

void state_driving_exit(fsm_t *fsm)
{
	//TODO, Open connectors, broadcast state
}

state_t errorState = {&state_error_enter, &state_error_iterate, &state_error_exit, "Error_s"};

void state_error_enter(fsm_t *fsm)
{
	//TODO, broadcast error over CAN, break alarm line(?)
}

void state_error_iterate(fsm_t *fsm)
{
	do{
		//TODO, broadcast error over CAN
	} while(0);
}

void state_error_exit(fsm_t *fsm)
{
	Error_Handler();
	return; // We should never get here.
}
