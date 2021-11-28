/*
 * AMS_FSM_Error.c
 *
 *  Created on: 6 Jul 2021
 *      Author: Thomas Fraser
 */

#include <AMS_FSM_States.h>

state_t errorState = { &state_error_enter, &state_error_iterate,
		&state_error_exit, "Error_s" };

void state_error_enter(fsm_t *fsm) {

	debugCAN_enterState(AMS_STATE_ID_Error);

	AMS_GlobalState->shutdown_state = 0;

	// LOW - PRECHG
	HAL_GPIO_WritePin(PRECHG_GPIO_Port, PRECHG_Pin, GPIO_PIN_RESET);

	// LOW - HVA+, HVA-, HVB+, HVB-
	HAL_GPIO_WritePin(HVA_N_GPIO_Port, HVA_N_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVB_N_GPIO_Port, HVB_N_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVA_P_GPIO_Port, HVA_P_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVB_P_GPIO_Port, HVB_P_Pin, GPIO_PIN_RESET);

	// Trip Shutdown Alarm Line
	AMS_GlobalState->shutdown_state = 0;
	HAL_GPIO_WritePin(ALARM_CTRL_GPIO_Port, ALARM_CTRL_Pin, GPIO_PIN_SET);

//	timer_delete(&AMS_GlobalState->IDC_AlarmTimer);
//	timer_delete(&AMS_GlobalState->heartbeatTimer);
//	timer_delete(&AMS_GlobalState->heartbeatTimerAMS);
//	timer_delete(&AMS_GlobalState->ccTimer);
//	timer_delete(&AMS_GlobalState->cTimer);

//	if (timer_isRunning(&AMS_GlobalState->debugTimer)) {
//		timer_delete(&AMS_GlobalState->debugTimer);
//	}
	/*
	queue_delete(&AMS_GlobalState->CANQueue);
	queue_delete(&AMS_GlobalState->CANForwardQueue);

	// Disable CAN Interrupts
	HAL_CAN_DeactivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_DeactivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	*/
}

void state_error_iterate(fsm_t *fsm) {
	// do {
		// We cannot escape from here. We are forever hitting UART.
		// Trip Shutdown Alarm Line
		AMS_GlobalState->shutdown_state = 0;
		HAL_GPIO_WritePin(ALARM_CTRL_GPIO_Port, ALARM_CTRL_Pin, GPIO_PIN_SET);
		AMS_LogErr("Stuck in error state", strlen("Stuck in error state"));
	/*
		HAL_Delay(2000);
	} while (1);
	*/
}

void state_error_exit(fsm_t *fsm) {
	debugCAN_enterState(AMS_STATE_ID_Error);

	Error_Handler();
	return; // We should never get here.
}
