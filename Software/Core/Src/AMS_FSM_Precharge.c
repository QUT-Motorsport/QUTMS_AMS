/*
 * AMS_FSM_Precharge.c
 *
 *  Created on: 6 Jul 2021
 *      Author: Thomas Fraser
 */

#include <AMS_FSM_States.h>

state_t prechargeState = { &state_precharge_enter, &state_precharge_iterate,
		&state_precharge_exit, "Precharge_s" };

void state_precharge_enter(fsm_t *fsm) {
	debugCAN_enterState(AMS_STATE_ID_Precharge);

	// Set Contractors to precharge mode
	HAL_GPIO_WritePin(HVA_N_GPIO_Port, HVA_N_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HVB_N_GPIO_Port, HVB_N_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HVB_P_GPIO_Port, HVA_P_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVB_P_GPIO_Port, HVB_P_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(PRECHG_GPIO_Port, PRECHG_Pin, GPIO_PIN_SET);

	AMS_GlobalState->prechargeTimer = timer_init(PRECHARGE_DELAY, true,
			&prechargeTimer_cb);
	timer_start(&AMS_GlobalState->prechargeTimer);
}

void state_precharge_iterate(fsm_t *fsm) {
	while (!queue_empty(&AMS_GlobalState->CANQueue)) {
		AMS_CAN_Generic_t msg;
		if (queue_next(&AMS_GlobalState->CANQueue, &msg)) {
			/** Handle the packet */
			/**
			 * @brief Packets precharge is looking for
			 * [BMS_BadCellVoltage], [BMS_BadCellTemperature],
			 * [BMS_TransmitVoltages], [BMS_TransmitTemperatures]
			 */
			if (msg.header.IDE == CAN_ID_EXT) {
				switch (msg.header.ExtId & BMS_ID_MASK) {
				case CC_ReadyToDrive_ID: {
					printf("RTD\r\n");

					if ((fabs(
							fabs(AMS_GlobalState->Voltage) - ACCUMULATOR_VOLTAGE)
							< PRECHARGE_VDIFF)
							|| (fabs(AMS_GlobalState->Voltage)
									> (ACCUMULATOR_VOLTAGE))) {
						char x[80];
						snprintf(x, 80, "HV Voltage Drop: %f, RTD.",
								fabs(
										fabs(
												AMS_GlobalState->Voltage) - ACCUMULATOR_VOLTAGE));
						fsm_changeState(fsm, &drivingState, x);
					}
				}
					break;

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

				Sendyne_handleVoltage(fsm, msg);

				Sendyne_handleColoumbCount(fsm, msg);

				Sendyne_handleCurrent(fsm, msg);

				BMS_handleBadCellVoltage(fsm, msg);

				BMS_handleBadCellTemperature(fsm, msg);
			}
		}
	}

	/** Monitor the Precharge Voltage */

	float accumulatorVoltage = 0;
	for (int i = 0; i < BMS_COUNT; i++) {
		for (int j = 0; j < BMS_VOLTAGE_COUNT; j++) {
			accumulatorVoltage += (float) (AMS_GlobalState->BMSVoltages[i][j]
					/ 1000.0f);
		}
		accumulatorVoltage = ACCUMULATOR_VOLTAGE;

		if (((fabs(
							fabs(AMS_GlobalState->Voltage) - ACCUMULATOR_VOLTAGE)
							< PRECHARGE_VDIFF)
							|| (fabs(AMS_GlobalState->Voltage)
									> (ACCUMULATOR_VOLTAGE)))) {
			/** Notify CC of the Prechage completion, but dont change into RTD yet */
			AMS_Ready_t notifyCCofPrechage = Compose_AMS_Ready();
			CAN_TxHeaderTypeDef header = { .ExtId = notifyCCofPrechage.id,
					.IDE = CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC = 0,
					.TransmitGlobalTime = DISABLE, };
			if (AMS_GlobalState->readyCount < 5) {
				HAL_CAN_AddTxMessage(&CANBUS2, &header, NULL,
						&AMS_GlobalState->CAN2_TxMailbox);
				AMS_GlobalState->readyCount++;
			}
			printf("precharge done %i\r\n", AMS_GlobalState->readyCount);
		}
	}
}

void state_precharge_exit(fsm_t *fsm) {
	debugCAN_exitState(AMS_STATE_ID_Precharge);

}

void prechargeTimer_cb(void *fsm) {
	Sendyne_requestVoltage(CS_V1);
}

