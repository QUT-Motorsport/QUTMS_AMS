/**
 ******************************************************************************
 * @file AMS_FSM_States.c
 * @brief AMS FSM States
 ******************************************************************************
 */

#include <AMS_FSM_States.h>
#include <stdlib.h>

state_t deadState = { &state_dead_enter, &state_dead_iterate, &state_dead_exit,
		"Dead_s" };

void state_dead_enter(fsm_t *fsm) {
	return;
}

void state_dead_iterate(fsm_t *fsm) {
	Error_Handler();
	return;
}

void state_dead_exit(fsm_t *fsm) {
	return;
}

state_t initState = { &state_init_enter, &state_init_iterate, &state_init_exit,
		"Init_s" };

void state_init_enter(fsm_t *fsm) {
	if (AMS_GlobalState == NULL) {
		AMS_GlobalState = malloc(sizeof(AMS_GlobalState_t));
		memset(AMS_GlobalState, 0, sizeof(AMS_GlobalState_t));

		AMS_GlobalState->heartbeatTimer = timer_init(AMS_HEARTBEAT_PERIOD, true, &heartbeatTimer_cb);
		timer_start(&AMS_GlobalState->heartbeatTimer);

		AMS_GlobalState->heartbeatTimerAMS = timer_init(AMS_HEARTBEATBMS_PERIOD, true, &heartbeatTimerBMS_cb);
		timer_start(&AMS_GlobalState->heartbeatTimerAMS);

		AMS_GlobalState->IDC_AlarmTimer = timer_init(AMS_IDC_PERIOD, true, &IDC_Alarm_cb);
		timer_start(&AMS_GlobalState->IDC_AlarmTimer);

#ifdef DEBUG_CB
		AMS_GlobalState->debugTimer = timer_init(DEBUG_PERIOD, true, &debugTimer_cb);
		timer_start(&AMS_GlobalState->debugTimer);
#endif

		queue_init(&AMS_GlobalState->CANQueue,
				sizeof(AMS_CAN_Generic_t), AMS_CAN_QUEUESIZE);
		queue_init(&AMS_GlobalState->CANForwardQueue,
				sizeof(AMS_CAN_Generic_t), AMS_CAN_QUEUESIZE);

		AMS_GlobalState->startupTicks = HAL_GetTick();
	}

	/* Set initial pin states */
	// ALARM Line - HIGH
	HAL_GPIO_WritePin(ALARM_CTRL_GPIO_Port, ALARM_CTRL_Pin, GPIO_PIN_RESET);

	// BMS Control - HIGH (Turn on all BMS)
	HAL_GPIO_WritePin(BMS_CTRL_GPIO_Port, BMS_CTRL_Pin, GPIO_PIN_SET);

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

state_t idleState = { &state_idle_enter, &state_idle_iterate, &state_idle_exit,
		"Idle_s" };

void state_idle_enter(fsm_t *fsm) {

	// Start requesting Sendyne Currents and Voltages
	AMS_GlobalState->ccTimer = timer_init(AMS_CS_PERIOD, true, &ccTimer_cb);
	timer_start(&AMS_GlobalState->ccTimer);

	AMS_GlobalState->cTimer = timer_init(AMS_CS_PERIOD/2, true, &cTimer_cb);
	timer_start(&AMS_GlobalState->cTimer);
	/* Set initial pin states */
	// ALARM Line - HIGH
	HAL_GPIO_WritePin(ALARM_CTRL_GPIO_Port, ALARM_CTRL_Pin, GPIO_PIN_RESET);

	// BMS Control - HIGH (Turn on all BMS)
	HAL_GPIO_WritePin(BMS_CTRL_GPIO_Port, BMS_CTRL_Pin, GPIO_PIN_SET);

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

void state_idle_iterate(fsm_t *fsm) {
	while (!queue_empty(&AMS_GlobalState->CANQueue)) {
		AMS_CAN_Generic_t msg;
		if (queue_next(&AMS_GlobalState->CANQueue, &msg)) {
			/** Handle the packet */
			/**
			 * @brief Packets idle is looking for
			 * CHASSIS_RTD, [BMS_BadCellVoltage], [BMS_BadCellTemperature], AMS2_ChargeEnabled,
			 * [BMS_TransmitVoltages], [BMS_TransmitTemperatures]
			 */

			if (msg.header.IDE == CAN_ID_EXT) {
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
	return;
}

state_t prechargeState = { &state_precharge_enter, &state_precharge_iterate,
		&state_precharge_exit, "Precharge_s" };

void state_precharge_enter(fsm_t *fsm) {
	// Set Contractors to precharge mode
	HAL_GPIO_WritePin(HVA_N_GPIO_Port, HVA_N_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HVB_N_GPIO_Port, HVB_N_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HVB_P_GPIO_Port, HVA_P_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVB_P_GPIO_Port, HVB_P_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(PRECHG_GPIO_Port, PRECHG_Pin, GPIO_PIN_SET);

	AMS_GlobalState->prechargeTimer = timer_init(PRECHARGE_DELAY, true, &prechargeTimer_cb);
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
					if (fabs(
							fabs(AMS_GlobalState->Voltage) - ACCUMULATOR_VOLTAGE) < PRECHARGE_VDIFF) {
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
			accumulatorVoltage +=
					(float) (AMS_GlobalState->BMSVoltages[i][j] / 1000.0f);
		}
		accumulatorVoltage = ACCUMULATOR_VOLTAGE;

		if (fabs(
				fabs(AMS_GlobalState->Voltage) - ACCUMULATOR_VOLTAGE) < PRECHARGE_VDIFF) {
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
		}
	}
}

void state_precharge_exit(fsm_t *fsm) {
	timer_delete(&AMS_GlobalState->prechargeTimer);
}

void prechargeTimer_cb(void *fsm) {
	Sendyne_requestVoltage(CS_V1);
}

state_t drivingState = { &state_driving_enter, &state_driving_iterate,
		&state_driving_exit, "Driving_s" };

void state_driving_enter(fsm_t *fsm) {
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
	// LOW - PRECHG
	HAL_GPIO_WritePin(PRECHG_GPIO_Port, PRECHG_Pin, GPIO_PIN_RESET);

	// LOW - HVA+, HVA-, HVB+, HVB-
	HAL_GPIO_WritePin(HVA_N_GPIO_Port, HVA_N_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVB_N_GPIO_Port, HVB_N_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVA_P_GPIO_Port, HVA_P_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVB_P_GPIO_Port, HVB_P_Pin, GPIO_PIN_RESET);
}

state_t errorState = { &state_error_enter, &state_error_iterate,
		&state_error_exit, "Error_s" };

void state_error_enter(fsm_t *fsm) {

	// LOW - PRECHG
	HAL_GPIO_WritePin(PRECHG_GPIO_Port, PRECHG_Pin, GPIO_PIN_RESET);

	// LOW - HVA+, HVA-, HVB+, HVB-
	HAL_GPIO_WritePin(HVA_N_GPIO_Port, HVA_N_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVB_N_GPIO_Port, HVB_N_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVA_P_GPIO_Port, HVA_P_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVB_P_GPIO_Port, HVB_P_Pin, GPIO_PIN_RESET);

	// Trip Shutdown Alarm Line
	HAL_GPIO_WritePin(ALARM_CTRL_GPIO_Port, ALARM_CTRL_Pin, GPIO_PIN_SET);

//	timer_delete(&AMS_GlobalState->IDC_AlarmTimer);
//	timer_delete(&AMS_GlobalState->heartbeatTimer);
//	timer_delete(&AMS_GlobalState->heartbeatTimerAMS);
//	timer_delete(&AMS_GlobalState->ccTimer);
//	timer_delete(&AMS_GlobalState->cTimer);

//	if (timer_isRunning(&AMS_GlobalState->debugTimer)) {
//		timer_delete(&AMS_GlobalState->debugTimer);
//	}
	queue_delete(&AMS_GlobalState->CANQueue);
	queue_delete(&AMS_GlobalState->CANForwardQueue);


	// Disable CAN Interrupts
	HAL_CAN_DeactivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_DeactivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void state_error_iterate(fsm_t *fsm) {
	do {
		// We cannot escape from here. We are forever hitting UART.
		// Trip Shutdown Alarm Line
		HAL_GPIO_WritePin(ALARM_CTRL_GPIO_Port, ALARM_CTRL_Pin, GPIO_PIN_SET);
		AMS_LogErr("Stuck in error state", strlen("Stuck in error state"));
		HAL_Delay(2000);
	} while (1);
}

void state_error_exit(fsm_t *fsm) {
	Error_Handler();
	return; // We should never get here.
}

state_t resetState = { &state_reset_enter, &state_reset_iterate,
		&state_reset_exit, "Reset_s" };

void state_reset_enter(fsm_t *fsm) {
	// LOW - PRECHG
	HAL_GPIO_WritePin(PRECHG_GPIO_Port, PRECHG_Pin, GPIO_PIN_RESET);

	// LOW - HVA+, HVA-, HVB+, HVB-
	HAL_GPIO_WritePin(HVA_N_GPIO_Port, HVA_N_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVB_N_GPIO_Port, HVB_N_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVA_P_GPIO_Port, HVA_P_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVB_P_GPIO_Port, HVB_P_Pin, GPIO_PIN_RESET);
}

void state_reset_iterate(fsm_t *fsm) {
	fsm_changeState(fsm, &idleState, "Resetting to Idle");
}

void state_reset_exit(fsm_t *fsm) {
	return;
}

state_t SoCState = { &state_SoC_enter, &state_SoC_iterate, &state_SoC_exit,
		"SoC_s" };

void state_SoC_enter(fsm_t *fsm) {
	/** We need 1 voltage packet from each BMS */
	AMS_GlobalState->bmsWakeupTimer = timer_init(BMS_WAKEUP_TIMEOUT, false, &wakeupTimerBMS_cb);
	timer_start(&AMS_GlobalState->bmsWakeupTimer);
}

void state_SoC_iterate(fsm_t *fsm) {
	while (!queue_empty(&AMS_GlobalState->CANQueue))  {
		AMS_CAN_Generic_t msg;
		if (queue_next(&AMS_GlobalState->CANQueue, &msg))
		{
			/** Handle the packet */
			/**
			 * @brief Packets driving state is looking for
			 * BMS_BadCellVoltage, BMS_BadCellTemperature,
			 * BMS_TransmitVoltages, BMS_TransmitTemperatures CC_Voltage
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

				BMS_handleBadCellVoltage(fsm, msg);

				BMS_handleBadCellTemperature(fsm, msg);
			}
		}
	}

	int bms_count = 0;
	int bms = 0;
	for (int i = 0; i < BMS_COUNT; i++) {
		if (AMS_GlobalState->BMSStartupSoc[i]) {
			bms_count++;
			bms |= (1<<i);
		}
	}

	if (bms_count == BMS_COUNT) {
		if (charge) {
			fsm_changeState(fsm, &chargingState, "Charging, all BMSs awake");
		} else {
			fsm_changeState(fsm, &idleState, "All BMSs awake, moving to idle");
		}
	} else {
		printf("bms: %d %x\r\n", bms_count, bms);
	}
}

void state_SoC_exit(fsm_t *fsm) {
	/** Now we have all BMSs, disable boot pin */
	HAL_GPIO_WritePin(BMS_CTRL_GPIO_Port, BMS_CTRL_Pin, GPIO_PIN_RESET);

	/** Stop and delete the BMS Wakeup Timer */
	timer_stop(&AMS_GlobalState->bmsWakeupTimer);
	timer_delete(&AMS_GlobalState->bmsWakeupTimer);
	return;
}

state_t chargingState = { &state_charging_enter, &state_charging_iterate,
		&state_charging_exit, "Charging_s" };

void state_charging_enter(fsm_t *fsm) {
	printf("Sending BMS Charge Enabled Message to BMSs\r\n");
	BMS_ChargeEnabled_t p = Compose_BMS_ChargeEnabled(BMS_COUNT); // Send a BMSId of 0 as it doesnt matter to the BMSs

	CAN_TxHeaderTypeDef header = { .ExtId = p.id, .IDE = CAN_ID_EXT, .RTR =
			CAN_RTR_DATA, .DLC = 0, .TransmitGlobalTime = DISABLE, };

	// Send charge message
	HAL_CAN_AddTxMessage(&CANBUS4, &header, NULL,
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
		if (queue_next(&AMS_GlobalState->CANQueue, &msg))
		{
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

void BMS_handleVoltage(fsm_t *fsm, AMS_CAN_Generic_t msg) {
	uint8_t BMSId;
	uint8_t vMsgId;
	uint16_t voltages[4];
	Parse_BMS_TransmitVoltage(msg.header.ExtId, msg.data, &BMSId, &vMsgId,
			voltages);
	if (BMSId > BMS_COUNT - 1) {
		char msg[] = "BMS ID Outside of acceptable range!";
		AMS_LogErr(msg, strlen(msg));
		return;
	}
	uint8_t voltageIndexStart = vMsgId * 4; // vMsgId : start | 0:0->3, 1:4->7, 2:8->9
	AMS_GlobalState->BMSStartupSoc[BMSId] = true;
	for (int i = 0; i < 4; i++) {
		AMS_GlobalState->BMSVoltages[BMSId][voltageIndexStart + i] =
				voltages[i];
	}
	/** If last message, log all voltages to SD*/
	if (vMsgId == 2) {
#if BMS_LOG_V
		printf(
				"{\"VoltageInfo\":{\"RT\": %.3f, \"BMS\": %i, \"Voltages\": [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]}}\r\n",
				getRuntime(), BMSId,
				AMS_GlobalState->BMSVoltages[BMSId][0] / 1000.f,
				AMS_GlobalState->BMSVoltages[BMSId][1] / 1000.f,
				AMS_GlobalState->BMSVoltages[BMSId][2] / 1000.f,
				AMS_GlobalState->BMSVoltages[BMSId][3] / 1000.f,
				AMS_GlobalState->BMSVoltages[BMSId][4] / 1000.f,
				AMS_GlobalState->BMSVoltages[BMSId][5] / 1000.f,
				AMS_GlobalState->BMSVoltages[BMSId][6] / 1000.f,
				AMS_GlobalState->BMSVoltages[BMSId][7] / 1000.f,
				AMS_GlobalState->BMSVoltages[BMSId][8] / 1000.f,
				AMS_GlobalState->BMSVoltages[BMSId][9] / 1000.f);
#endif
	}
}

void BMS_handleTemperature(fsm_t *fsm, AMS_CAN_Generic_t msg) {
	uint8_t BMSId;
	uint8_t tMsgId;
	uint8_t temperatures[6];
	Parse_BMS_TransmitTemperature(msg.header.ExtId, msg.data, &BMSId, &tMsgId,
			temperatures);
	if (BMSId > BMS_COUNT - 1) {
		char msg[] = "BMS ID Outside of acceptable range!";
		AMS_LogErr(msg, strlen(msg));
		return;
	}
	uint8_t temperatureIndexStart = tMsgId * 6; // tMsgId : start | 0:0->5, 1:11
	for (int i = 0; i < 6; i++) {
		AMS_GlobalState->BMSTemperatures[BMSId][temperatureIndexStart + i] =
				temperatures[i];
	}
	/** If last message, log all temperatures to SD*/
	if (tMsgId == 2) {
#if BMS_LOG_T
		printf(
				"{\"TemperatureInfo\":{\"RT\": %.3f, \"BMS\": %i, \"Temperatures\": [%i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i]}}\r\n",
				getRuntime(), BMSId,
				AMS_GlobalState->BMSTemperatures[BMSId][0],
				AMS_GlobalState->BMSTemperatures[BMSId][1],
				AMS_GlobalState->BMSTemperatures[BMSId][2],
				AMS_GlobalState->BMSTemperatures[BMSId][3],
				AMS_GlobalState->BMSTemperatures[BMSId][4],
				AMS_GlobalState->BMSTemperatures[BMSId][5],
				AMS_GlobalState->BMSTemperatures[BMSId][6],
				AMS_GlobalState->BMSTemperatures[BMSId][7],
				AMS_GlobalState->BMSTemperatures[BMSId][8],
				AMS_GlobalState->BMSTemperatures[BMSId][9],
				AMS_GlobalState->BMSTemperatures[BMSId][10],
				AMS_GlobalState->BMSTemperatures[BMSId][11],
				AMS_GlobalState->BMSTemperatures[BMSId][12],
				AMS_GlobalState->BMSTemperatures[BMSId][13]);
#endif
	}
}

void BMS_handleBadCellVoltage(fsm_t *fsm, AMS_CAN_Generic_t msg) {
	/** BMS_BadCellVoltage With BMSID masked off */
	if ((msg.header.ExtId & BMS_ID_MASK)
			== Compose_CANId(0x0, 0x12, 0x0, 0x0, 0x00, 0x0)) {
		uint8_t BMSId;
		uint8_t cellNum;
		uint8_t voltage;
		Parse_BMS_BadCellVoltage(msg.data, &BMSId, &cellNum, &voltage);

		AMS_CellVoltageShutdown_t cVS = Compose_AMS_CellVoltageShutdown(cellNum,
				BMSId, voltage);
		CAN_TxHeaderTypeDef header = { .ExtId = cVS.id, .IDE = CAN_ID_EXT,
				.RTR = CAN_RTR_DATA, .DLC = sizeof(cVS.data),
				.TransmitGlobalTime = DISABLE, };

		// Notify Chassis we have a bad cell voltage.
		HAL_CAN_AddTxMessage(&hcan1, &header, cVS.data,
				&AMS_GlobalState->CAN2_TxMailbox);

		char x[80];
		int len = snprintf(x, 80,
				"Found Bad Cell Voltage: Cell:%i, Voltage: %i", cellNum,
				voltage);

		// Bad BMS cell voltage found, we need to change to errorState.
		AMS_LogErr(x, len);
		fsm_changeState(fsm, &errorState, "Found Bad BMS Cell Voltage");
	}
}

uint8_t bTempCount[BMS_COUNT][BMS_TEMPERATURE_COUNT];

void BMS_handleBadCellTemperature(fsm_t *fsm, AMS_CAN_Generic_t msg) {
	/** BMS_BadCellTemperature With BMSID masked off */
	if ((msg.header.ExtId & BMS_ID_MASK)
			== Compose_CANId(0x0, 0x12, 0x0, 0x0, 0x01, 0x0)) {
		uint8_t BMSId;
		uint8_t cellNum;
		uint8_t temperature;
		Parse_BMS_BadCellTemperature(msg.data, &BMSId, &cellNum, &temperature);

		AMS_CellTemperatureShutdown_t cTS = Compose_AMS_CellTemperatureShutdown(
				cellNum, BMSId, temperature);
		CAN_TxHeaderTypeDef header = { .ExtId = cTS.id, .IDE = CAN_ID_EXT,
				.RTR = CAN_RTR_DATA, .DLC = sizeof(cTS.data),
				.TransmitGlobalTime = DISABLE, };

		// Notify Chassis we have a bad cell temperature.
		HAL_CAN_AddTxMessage(&hcan1, &header, cTS.data,
				&AMS_GlobalState->CAN2_TxMailbox);

		//		char x[80];
		//		int len = snprintf(x, 80, "Found Bad Cell Temperature: Cell:%i, Temp: %i", cellNum, temperature);

		// Bad BMS cell temperature found, we need to change to errorState
		//		AMS_LogErr(x, len);
		//		fsm_changeState(fsm, &errorState, "Found Bad BMS Cell Temperature");
		//memset(bTempCount, 0, BMS_COUNT*BMS_TEMPERATURE_COUNT * sizeof(uint8_t));
		for (int i = 0; i < BMS_COUNT; i++) {
			for (int j = 0; j < BMS_TEMPERATURE_COUNT; j++) {
				if (AMS_GlobalState->BMSTemperatures[i][j] > 55
						&& AMS_GlobalState->BMSTemperatures[i][j] < 75
						&& !(j >= 6)) {
					//							char x[80];
					//							int len = snprintf(x, 80, "Found Bad Cell Temperature, BMS-%i, %iC", i, AMS_GlobalState->BMSTemperatures[i][j]);
					//							AMS_LogErr(x, len);
					bTempCount[i][j]++;

				} else {
					bTempCount[i][j] = 0;
				}
				if (bTempCount[i][j] > 5) {
					fsm_changeState(fsm, &errorState,
							"Found Bad BMS Cell Temperature");
					break;
				}
			}

		}

		/*if (bTempCount > 2) {
			 //fsm_changeState(fsm, &errorState,
			 //		"Found Bad BMS Cell Temperature");
			 }*/
	}
}

void Sendyne_handleVoltage(fsm_t *fsm, AMS_CAN_Generic_t msg) {
	/** CS */
	if ((msg.header.ExtId) == CS_1_RESPONSE_EXTID) {
		/** Voltage */
		if (msg.data[0] == CS_V1) {

			AMS_GlobalState->VoltageuV = 0;
			AMS_GlobalState->VoltageuV |= (int32_t) msg.data[1] << 24;
			AMS_GlobalState->VoltageuV |= (int32_t) msg.data[2] << 16;
			AMS_GlobalState->VoltageuV |= (int32_t) msg.data[3] << 8;
			AMS_GlobalState->VoltageuV |= (int32_t) msg.data[4] << 0;
			AMS_GlobalState->Voltage = AMS_GlobalState->VoltageuV
					/ 1000000.0f;

			AMS_GlobalState->Voltage = (float) (AMS_GlobalState->VoltageuV
					/ 1000000.f);
			//				printf("[%li] Voltage: %f\r\n", getRuntime(), AMS_GlobalState->Voltage);


			/** Send Voltage Log Msg to CC */
			uint8_t priority;
			uint16_t sourceId;
			uint8_t autonomous;
			uint8_t type;
			uint16_t extra;
			uint8_t BMSId;
			Parse_CANId(msg.header.ExtId, &priority, &sourceId, &autonomous,
					&type, &extra, &BMSId);

			CAN_TxHeaderTypeDef h = { .ExtId = Compose_CANId(
					CAN_PRIORITY_DEBUG, sourceId, autonomous, type, extra,
					BMSId), .IDE = CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC =
							msg.header.DLC, .TransmitGlobalTime = DISABLE };

			if (HAL_CAN_AddTxMessage(&CANBUS2, &h, msg.data,
					&AMS_GlobalState->CAN2_TxMailbox) != HAL_OK) {
				char msg[] = "Failed to send Sendyne Voltage log msg";
				AMS_LogErr(msg, strlen(msg));
			}
		}
	}
}

void Sendyne_handleColoumbCount(fsm_t *fsm, AMS_CAN_Generic_t msg) {
	/** Current Sensor Coulomb Counting */
	if (msg.header.ExtId == CS_1_RESPONSE_EXTID) {
		if (msg.data[0] == CURRENT_SENSOR_CC_LOW) {

			AMS_GlobalState->CoulombCountuA = 0;
			AMS_GlobalState->CoulombCountuA |= (int64_t) msg.data[1] << 24;
			AMS_GlobalState->CoulombCountuA |= (int64_t) msg.data[2] << 16;
			AMS_GlobalState->CoulombCountuA |= (int64_t) msg.data[3] << 8;
			AMS_GlobalState->CoulombCountuA |= (int64_t) msg.data[4] << 0;

		} else if (msg.data[0] == CURRENT_SENSOR_CC_HIGH) {
			AMS_GlobalState->CoulombCountuA |= (int64_t) msg.data[1] << 56;
			AMS_GlobalState->CoulombCountuA |= (int64_t) msg.data[2] << 48;
			AMS_GlobalState->CoulombCountuA |= (int64_t) msg.data[3] << 40;
			AMS_GlobalState->CoulombCountuA |= (int64_t) msg.data[4] << 32;

			AMS_GlobalState->CoulombCount =
					(float) (AMS_GlobalState->CoulombCountuA / 1000000.f);

#if CS_LOG_CC
			printf(x, "[%li] Coloumb Count: %f\r\n", getRuntime(), AMS_GlobalState->CoulombCount);
#endif
		}
	}
}

void Sendyne_handleCurrent(fsm_t *fsm, AMS_CAN_Generic_t msg) {
	if (msg.header.ExtId == CS_1_RESPONSE_EXTID) {
		if (msg.data[0] == CURRENT_SENSOR_CURRENT) {
			AMS_GlobalState->HVACurrentuA = 0;
			AMS_GlobalState->HVACurrentuA |= (int64_t) msg.data[1] << 24;
			AMS_GlobalState->HVACurrentuA |= (int64_t) msg.data[2] << 16;
			AMS_GlobalState->HVACurrentuA |= (int64_t) msg.data[3] << 8;
			AMS_GlobalState->HVACurrentuA |= (int64_t) msg.data[4] << 0;

			AMS_GlobalState->HVACurrent =
					(float) (AMS_GlobalState->HVACurrentuA / 1000000.f);

			/** Send Voltage Log Msg to CC */

			uint8_t priority;
			uint16_t sourceId;
			uint8_t autonomous;
			uint8_t type;
			uint16_t extra;
			uint8_t BMSId;
			Parse_CANId(msg.header.ExtId, &priority, &sourceId, &autonomous,
					&type, &extra, &BMSId);

			CAN_TxHeaderTypeDef h = { .ExtId = Compose_CANId(
					CAN_PRIORITY_DEBUG, sourceId, autonomous, type, extra,
					BMSId), .IDE = CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC =
							msg.header.DLC, .TransmitGlobalTime = DISABLE };

			if (HAL_CAN_AddTxMessage(&CANBUS2, &h, msg.data,
					&AMS_GlobalState->CAN2_TxMailbox) != HAL_OK) {
				char msg[] = "Failed to send Sendyne Current log msg";
				AMS_LogErr(msg, strlen(msg));
			}
		}
	} else if (msg.header.ExtId == CS_2_RESPONSE_EXTID) {
		if (msg.data[0] == CURRENT_SENSOR_CURRENT) {

			AMS_GlobalState->HVBCurrentuA = 0;
			AMS_GlobalState->HVBCurrentuA |= (int64_t) msg.data[1] << 24;
			AMS_GlobalState->HVBCurrentuA |= (int64_t) msg.data[2] << 16;
			AMS_GlobalState->HVBCurrentuA |= (int64_t) msg.data[3] << 8;
			AMS_GlobalState->HVBCurrentuA |= (int64_t) msg.data[4] << 0;

			AMS_GlobalState->HVBCurrent =
					(float) (AMS_GlobalState->HVBCurrentuA / 1000000.f);

		}
	}
}

