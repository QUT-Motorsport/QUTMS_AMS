/*
 * AMS_FSM_Charging.c
 *
 *  Created on: 6 Jul 2021
 *      Author: Thomas Fraser
 */

#include <AMS_FSM_States.h>

state_t chargingState = { &state_charging_enter, &state_charging_iterate,
		&state_charging_exit, "Charging_s" };

void state_charging_enter(fsm_t *fsm) {
	printf("Sending BMS Charge Enabled Message to BMSs\r\n");
	BMS_ChargeEnabled_t p = Compose_BMS_ChargeEnabled(BMS_COUNT); // Send a BMSId of 0 as it doesnt matter to the BMSs

	CAN_TxHeaderTypeDef header = { .ExtId = p.id, .IDE = CAN_ID_EXT, .RTR =
	CAN_RTR_DATA, .DLC = 1, .TransmitGlobalTime = DISABLE, };

	// Send charge message
	HAL_CAN_AddTxMessage(&CANBUS4, &header, p.data,
			&AMS_GlobalState->CAN4_TxMailbox);

	// Close Connectors

	// LOW - PRECHG
	HAL_GPIO_WritePin(PRECHG_GPIO_Port, PRECHG_Pin, GPIO_PIN_RESET);

	// PROFET Positions AFTER Precharge
	// HIGH - HVA+, HVA-, HVB+, HVB-
	HAL_GPIO_WritePin(HVA_N_GPIO_Port, HVA_N_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HVB_N_GPIO_Port, HVB_N_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HVA_P_GPIO_Port, HVA_P_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HVB_P_GPIO_Port, HVB_P_Pin, GPIO_PIN_SET);
	printf("Entering Charging State\r\n");
}

void state_charging_iterate(fsm_t *fsm) {
	while (!queue_empty(&AMS_GlobalState->CANQueue)) {
		AMS_CAN_Generic_t msg;
		if (queue_next(&AMS_GlobalState->CANQueue, &msg)) {
			switch (msg.header.ExtId & BMS_ID_MASK) {
			case BMS_BadCellVoltage_ID:
				BMS_handleBadCellVoltage(fsm, msg);
				break;
			case BMS_BadCellTemperature_ID:
				BMS_handleBadCellTemperature(fsm, msg);
				break;
			case BMS_TransmitVoltage_ID:
				BMS_handleVoltage(fsm, msg);
				break;
			case BMS_TransmitTemperature_ID:
				BMS_handleTemperature(fsm, msg);
				break;
			case BMS_TransmitBalancing_ID: { // Give the label a scope
				uint8_t BMSId;
				uint16_t balancingVoltage;
				uint16_t balancingState;
				Parse_TransmitBalancing(msg.header.ExtId, msg.data, &BMSId,
						&balancingVoltage, &balancingState);
				printf(
						"{\"BalanceInfo\":{\"RT\": %.3f, \"BMS\": %i, \"BalanceVoltage\": %i, \"BalanceState\": %i}}\r\n",
						getRuntime(), BMSId, balancingVoltage, balancingState);
				break;
			}
			}
		}
	}
}

void state_charging_exit(fsm_t *fsm) {
	// Open Contactors

	// LOW - PRECHG
	HAL_GPIO_WritePin(PRECHG_GPIO_Port, PRECHG_Pin, GPIO_PIN_RESET);

	// PROFET Positions AFTER Precharge
	// HIGH - HVA+, HVA-, HVB+, HVB-
	HAL_GPIO_WritePin(HVA_N_GPIO_Port, HVA_N_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVB_N_GPIO_Port, HVB_N_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVA_P_GPIO_Port, HVA_P_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVB_P_GPIO_Port, HVB_P_Pin, GPIO_PIN_RESET);
	return;
}
