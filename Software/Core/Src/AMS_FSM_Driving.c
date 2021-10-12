/*
 * AMS_FSM_Driving.c
 *
 *  Created on: 6 Jul 2021
 *      Author: Thomas Fraser
 */

#include <AMS_FSM_States.h>

state_t drivingState = { &state_driving_enter, &state_driving_iterate,
		&state_driving_exit, "Driving_s" };

void state_driving_enter(fsm_t *fsm) {

	debugCAN_enterState(AMS_STATE_ID_Driving);

	// Close Connectors

	// LOW - PRECHG
	HAL_GPIO_WritePin(PRECHG_GPIO_Port, PRECHG_Pin, GPIO_PIN_RESET);

	// PROFET Positions AFTER Precharge
	// HIGH - HVA+, HVA-, HVB+, HVB-
	HAL_GPIO_WritePin(HVA_N_GPIO_Port, HVA_N_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HVB_N_GPIO_Port, HVB_N_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HVA_P_GPIO_Port, HVA_P_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HVB_P_GPIO_Port, HVB_P_Pin, GPIO_PIN_SET);

}

void state_driving_iterate(fsm_t *fsm) {
	while (!queue_empty(&AMS_GlobalState->CANQueue)) {
		AMS_CAN_Generic_t msg;
		if (queue_next(&AMS_GlobalState->CANQueue, &msg)) {
			/** Handle the packet */
			/**
			 * @brief Packets driving state is looking for
			 * BMS_BadCellVoltage, BMS_BadCellTemperature,
			 * BMS_TransmitVoltages, BMS_TransmitTemperatures
			 */

			if (msg.header.IDE == CAN_ID_EXT) {
				switch (msg.header.ExtId & BMS_ID_MASK) {
				case BMS_TransmitVoltage_ID:
					BMS_handleVoltage(fsm, msg);
					break;

				case BMS_TransmitTemperature_ID:
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
			}

			BMS_handleBadCellVoltage(fsm, msg);

			BMS_handleBadCellTemperature(fsm, msg);

			Sendyne_handleColoumbCount(fsm, msg);

			Sendyne_handleCurrent(fsm, msg);
		}
	}
}

void state_driving_exit(fsm_t *fsm) {
	debugCAN_exitState(AMS_STATE_ID_Driving);

	// LOW - PRECHG
	HAL_GPIO_WritePin(PRECHG_GPIO_Port, PRECHG_Pin, GPIO_PIN_RESET);

	// LOW - HVA+, HVA-, HVB+, HVB-
	HAL_GPIO_WritePin(HVA_N_GPIO_Port, HVA_N_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVB_N_GPIO_Port, HVB_N_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVA_P_GPIO_Port, HVA_P_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVB_P_GPIO_Port, HVB_P_Pin, GPIO_PIN_RESET);
}
