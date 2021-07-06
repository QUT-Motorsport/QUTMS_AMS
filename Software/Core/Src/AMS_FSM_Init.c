/*
 * AMS_FSM_Init.c
 *
 *  Created on: 6 Jul 2021
 *      Author: Thomas Fraser
 */

#include <AMS_FSM_States.h>

state_t initState = { &state_init_enter, &state_init_iterate, &state_init_exit,
		"Init_s" };

void state_init_enter(fsm_t *fsm) {
	if (AMS_GlobalState == NULL) {
		AMS_GlobalState = malloc(sizeof(AMS_GlobalState_t));
		memset(AMS_GlobalState, 0, sizeof(AMS_GlobalState_t));

		AMS_GlobalState->heartbeatTimer = timer_init(AMS_HEARTBEAT_PERIOD, true,
				&heartbeatTimer_cb);
		timer_start(&AMS_GlobalState->heartbeatTimer);

		AMS_GlobalState->IDC_AlarmTimer = timer_init(AMS_IDC_PERIOD, true,
				&IDC_Alarm_cb);
		timer_start(&AMS_GlobalState->IDC_AlarmTimer);

#ifdef DEBUG_CB
		AMS_GlobalState->debugTimer = timer_init(DEBUG_PERIOD, true,
				&debugTimer_cb);
		timer_start(&AMS_GlobalState->debugTimer);
#endif

		queue_init(&AMS_GlobalState->CANQueue, sizeof(AMS_CAN_Generic_t),
				AMS_CAN_QUEUESIZE);
		queue_init(&AMS_GlobalState->CANForwardQueue, sizeof(AMS_CAN_Generic_t),
				AMS_CAN_QUEUESIZE);

		AMS_GlobalState->startupTicks = HAL_GetTick();
	}

	/* Set initial pin states */
	// ALARM Line - HIGH
	HAL_GPIO_WritePin(ALARM_CTRL_GPIO_Port, ALARM_CTRL_Pin, GPIO_PIN_RESET);

	// BMS Control - HIGH (Turn on all BMS)
//	HAL_GPIO_WritePin(BMS_CTRL_GPIO_Port, BMS_CTRL_Pin, GPIO_PIN_SET);

	//Set Initial PROFET Pin Positions (All Off)
	// Contactors
	HAL_GPIO_WritePin(HVA_N_GPIO_Port, HVA_N_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVB_N_GPIO_Port, HVB_N_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVB_P_GPIO_Port, HVA_P_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVB_P_GPIO_Port, HVB_P_Pin, GPIO_PIN_RESET);
	// Precharge
	HAL_GPIO_WritePin(PRECHG_GPIO_Port, PRECHG_Pin, GPIO_PIN_RESET);

	// FAN On
	HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_SET);
}

void state_init_iterate(fsm_t *fsm) {
	return;
}

void state_init_exit(fsm_t *fsm) {
	return;
}
