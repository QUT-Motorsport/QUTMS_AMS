/*
 * AMS_FSM_Idle.c
 *
 *  Created on: 6 Jul 2021
 *      Author: Thomas Fraser
 */

#include <AMS_FSM_States.h>

state_t idleState = { &state_idle_enter, &state_idle_iterate, &state_idle_exit,
		"Idle_s" };

void state_idle_enter(fsm_t *fsm) {


	// Start requesting Sendyne Currents and Voltages
	AMS_GlobalState->ccTimer = timer_init(AMS_CS_PERIOD, true, &ccTimer_cb);
	timer_start(&AMS_GlobalState->ccTimer);

	AMS_GlobalState->cTimer = timer_init(AMS_CS_PERIOD_FAST, true, &cTimer_cb);
	timer_start(&AMS_GlobalState->cTimer);
	/* Set initial pin states */

	// ALARM Line - HIGH
	AMS_GlobalState->shutdown_state = 1;
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



	debugCAN_enterState(AMS_STATE_ID_Idle);
}

void state_idle_iterate(fsm_t *fsm) {
	int i = 0;
	while ((!queue_empty(&AMS_GlobalState->CANQueue)) && (i < 10)) {
		i++;
		AMS_CAN_Generic_t msg;
		if (queue_next(&AMS_GlobalState->CANQueue, &msg)) {
			/** Handle the packet */
			/**
			 * @brief Packets idle is looking for
			 * CHASSIS_RTD, [BMS_BadCellVoltage], [BMS_BadCellTemperature], AMS2_ChargeEnabled,
			 * [BMS_TransmitVoltages], [BMS_TransmitTemperatures]
			 */

			if (msg.header.IDE == CAN_ID_EXT) {
				//printf("id: %i\r\n", msg.header.ExtId);
				switch (msg.header.ExtId & BMS_ID_MASK) {
				case AMS_StartUp_ID:
					/** Got AMS Startup, move to precharge and await RTD */
					fsm_changeState(fsm, &prechargeState,
							"Moving to precharge by command of CC");
					break;
				case BMS_TransmitVoltage_ID:
					/** BMS_TransmitVoltage with BMSID masked off*/
					BMS_handleVoltage(fsm, msg);
					break;
				case BMS_TransmitTemperature_ID:
					/** BMS_TransmitTemperatures with BMSID masked off*/
					BMS_handleTemperature(fsm, msg);
					break;

				case CC_FatalShutdown_ID:
					fsm_changeState(fsm, &errorState,
							"Fatal Shutdown received from CC");
					break;

				case CC_SoftShutdown_ID:
					fsm_changeState(fsm, &resetState,
							"Soft Shutdown received from CC");
					break;
				}

				BMS_handleBadCellVoltage(fsm, msg);

				BMS_handleBadCellTemperature(fsm, msg);

				Sendyne_handleColoumbCount(fsm, msg);

				Sendyne_handleCurrent(fsm, msg);
			}
		}
	}
}

void state_idle_exit(fsm_t *fsm) {

	debugCAN_exitState(AMS_STATE_ID_Idle);

	return;
}
