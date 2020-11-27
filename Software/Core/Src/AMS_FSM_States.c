/**
 ******************************************************************************
 * @file AMS_FSM_States.c
 * @brief AMS FSM States
 ******************************************************************************
 */

#include <AMS_FSM_States.h>

int globalTimer = 0;

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

state_t initState = {&state_init_enter, &state_init_iterate, &state_init_exit, "Init_s"};

void state_init_enter(fsm_t *fsm)
{
	if(AMS_GlobalState == NULL)
	{
		AMS_GlobalState = malloc(sizeof(AMS_GlobalState_t));
		memset(AMS_GlobalState, 0, sizeof(AMS_GlobalState_t));

		// As AMS_GlobalState is accessible across threads, we need to use a semaphore to access it
		AMS_GlobalState->sem = osSemaphoreNew(1U, 1U, NULL);
		if(osSemaphoreAcquire(AMS_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
		{
			AMS_GlobalState->heartbeatTimer = osTimerNew(&heartbeatTimer_cb, osTimerPeriodic, fsm, NULL);
			if(osTimerStart(AMS_GlobalState->heartbeatTimer, AMS_HEARTBEAT_PERIOD) != osOK)
			{
				char msg[] = "Failed to create Heartbeat Timer";
				AMS_LogErr(msg, strlen(msg));
			}

			AMS_GlobalState->heartbeatTimerAMS = osTimerNew(&heartbeatTimerBMS_cb, osTimerPeriodic, fsm, NULL);
			if(osTimerStart(AMS_GlobalState->heartbeatTimerAMS, AMS_HEARTBEATBMS_PERIOD) != osOK)
			{
				char msg[] = "Failed to create BMS Heartbeat Timer";
				AMS_LogErr(msg, strlen(msg));
			}

			AMS_GlobalState->IDC_AlarmTimer = osTimerNew(&IDC_Alarm_cb, osTimerPeriodic, fsm, NULL);
			if(osTimerStart(AMS_GlobalState->IDC_AlarmTimer, AMS_IDC_PERIOD) != osOK)
			{
				char msg[] = "Failed to create IDC_Alarm Timer";
				AMS_LogErr(msg, strlen(msg));
			}
			AMS_GlobalState->ccTimer = osTimerNew(&ccTimer_cb, osTimerPeriodic, fsm, NULL);
			if(osTimerStart(AMS_GlobalState->ccTimer, AMS_CS_PERIOD) != osOK)
			{
				char msg[] = "Failed to create Coulomb Counting Timer";
				AMS_LogErr(msg, strlen(msg));
			}
			AMS_GlobalState->cTimer = osTimerNew(&cTimer_cb, osTimerPeriodic, fsm, NULL);
			if(osTimerStart(AMS_GlobalState->cTimer, AMS_CS_PERIOD/2) != osOK)
			{
				char msg[] = "Failed to create Current Sensing Timer";
				AMS_LogErr(msg, strlen(msg));
			}
#ifdef DEBUG_CB
			AMS_GlobalState->debugTimer = osTimerNew(&debugTimer_cb, osTimerPeriodic, fsm, NULL);
			if(osTimerStart(AMS_GlobalState->debugTimer, DEBUG_PERIOD) != osOK)
			{
				char msg[] = "Failed to create debug timer";
				AMS_LogErr(msg, strlen(msg));
			}
#endif

			AMS_GlobalState->CANQueue = osMessageQueueNew(AMS_CAN_QUEUESIZE, sizeof(AMS_CAN_Generic_t), NULL);
			if(AMS_GlobalState->CANQueue == NULL)
			{
				char msg[] = "Failed to make CANQueue";
				AMS_LogErr(msg, strlen(msg));
			}

			AMS_GlobalState->startupTicks = HAL_GetTick();
			osSemaphoreRelease(AMS_GlobalState->sem);

		}
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

void state_init_iterate(fsm_t *fsm)
{
	fsm_changeState(fsm, &SoCState, "We are initialised, time to SoC");
}

void state_init_exit(fsm_t *fsm)
{
	return;
}

state_t idleState = {&state_idle_enter, &state_idle_iterate, &state_idle_exit, "Idle_s"};

void state_idle_enter(fsm_t *fsm)
{
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

void state_idle_iterate(fsm_t *fsm)
{
	while(osMessageQueueGetCount(AMS_GlobalState->CANQueue) >= 1)
	{
		AMS_CAN_Generic_t msg;
		if(osMessageQueueGet(AMS_GlobalState->CANQueue, &msg, 0U, 0U) == osOK)
		{
			/** Handle the packet */
			/**
			 * @brief Packets idle is looking for
			 * CHASSIS_RTD, [BMS_BadCellVoltage], [BMS_BadCellTemperature], AMS2_ChargEnabled,
			 * [BMS_TransmitVoltages], [BMS_TransmitTemperatures]
			 */

			if(msg.header.IDE == CAN_ID_EXT)
			{
				/** BMS_TransmitVoltages With BMSID masked off */
				if((msg.header.ExtId & BMS_ID_MASK) == Compose_CANId(CAN_PRIORITY_NORMAL, CAN_SRC_ID_BMS, 0x0, CAN_TYPE_TRANSMIT, 0x02, 0x0))
				{
					uint8_t BMSId; uint8_t vMsgId; uint16_t voltages[4];
					Parse_BMS_TransmitVoltage(msg.header.ExtId, msg.data, &BMSId, &vMsgId, voltages);
					if(BMSId > BMS_COUNT)
					{
						char msg[] = "BMS ID Outside of acceptable range!";
						AMS_LogErr(msg, strlen(msg));
						return;
					}
					uint8_t voltageIndexStart = vMsgId * 4; // vMsgId : start | 0:0->3, 1:4->7, 2:8->9
					if(osSemaphoreAcquire(AMS_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
					{
						for(int i = 0; i < 4; i++)
						{
							AMS_GlobalState->BMSVoltages[BMSId][voltageIndexStart + i] = voltages[i];
						}
						/** If last message, log all voltages to SD*/
						if(vMsgId == 2)
						{
#if BMS_LOG_V
							printf("[%li] BMS-%i: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f (V)\r\n", getRuntime(), BMSId,
									AMS_GlobalState->BMSVoltages[BMSId][0]/1000.f,
									AMS_GlobalState->BMSVoltages[BMSId][1]/1000.f,
									AMS_GlobalState->BMSVoltages[BMSId][2]/1000.f,
									AMS_GlobalState->BMSVoltages[BMSId][3]/1000.f,
									AMS_GlobalState->BMSVoltages[BMSId][4]/1000.f,
									AMS_GlobalState->BMSVoltages[BMSId][5]/1000.f,
									AMS_GlobalState->BMSVoltages[BMSId][6]/1000.f,
									AMS_GlobalState->BMSVoltages[BMSId][7]/1000.f,
									AMS_GlobalState->BMSVoltages[BMSId][8]/1000.f,
									AMS_GlobalState->BMSVoltages[BMSId][9]/1000.f);
#endif
						}
						osSemaphoreRelease(AMS_GlobalState->sem);
					}
				}

				/** BMS_TransmitTemperatures With BMSID masked off */
				if((msg.header.ExtId & BMS_ID_MASK) == Compose_CANId(CAN_PRIORITY_NORMAL, CAN_SRC_ID_BMS, 0x0, CAN_TYPE_TRANSMIT, 0x03, 0x0))
				{
					uint8_t BMSId; uint8_t tMsgId; uint8_t temperatures[6];
					Parse_BMS_TransmitTemperature(msg.header.ExtId, msg.data, &BMSId, &tMsgId, temperatures);
					if(BMSId > BMS_COUNT)
					{
						char msg[] = "BMS ID Outside of acceptable range!";
						AMS_LogErr(msg, strlen(msg));
						return;
					}
					uint8_t temperatureIndexStart = tMsgId * 6; // tMsgId : start | 0:0->5, 1:11
					if(osSemaphoreAcquire(AMS_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
					{
						for(int i = 0; i < 6; i++)
						{
							AMS_GlobalState->BMSTemperatures[BMSId][temperatureIndexStart + i] = temperatures[i];
						}
						/** If last message, log all temperatures to SD*/
						if(tMsgId == 1)
						{
#if BMS_LOG_T
							printf("[%li] BMS-%i: %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i (Degrees)\r\n", getRuntime(), BMSId,
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
									AMS_GlobalState->BMSTemperatures[BMSId][11]);
#endif
						}
						osSemaphoreRelease(AMS_GlobalState->sem);
					}
				}

				BMS_handleBadCellVoltage(fsm, msg);

				BMS_handleBadCellTemperature(fsm, msg);

				Sendyne_handleColoumbCount(fsm, msg);

				Sendyne_handleCurrent(fsm, msg);

				/** CC_RTD with no care about BMSID */
				if((msg.header.ExtId) == Compose_CANId(0x2, 0x16, 0x0, 0x0, 0x0, 0x0))
				{
					fsm_changeState(fsm, &prechargeState, "RTD from CC, moving to Precharge");
				}
			}
		}
	}
}

void state_idle_exit(fsm_t *fsm)
{
	return;
}

state_t prechargeState = {&state_precharge_enter, &state_precharge_iterate, &state_precharge_exit, "Precharge_s"};

void state_precharge_enter(fsm_t *fsm)
{
	// Set Contractors to precharge mode
	HAL_GPIO_WritePin(HVA_N_GPIO_Port, HVA_N_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HVB_N_GPIO_Port, HVB_N_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HVB_P_GPIO_Port, HVA_P_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVB_P_GPIO_Port, HVB_P_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(PRECHG_GPIO_Port, PRECHG_Pin, GPIO_PIN_SET);

	prechargeTimer = osTimerNew(&prechargeTimer_cb, osTimerPeriodic, fsm, NULL);
	if(osTimerStart(prechargeTimer, PRECHARGE_DELAY) != osOK)
	{
		char msg[] = "Failed to start Precharge Voltage Request Timer";
		AMS_LogErr(msg, strlen(msg));
	}
	globalTimer = HAL_GetTick();
}

void state_precharge_iterate(fsm_t *fsm)
{
	while(osMessageQueueGetCount(AMS_GlobalState->CANQueue) >= 1)
	{
		AMS_CAN_Generic_t msg;
		if(osMessageQueueGet(AMS_GlobalState->CANQueue, &msg, 0U, 0U) == osOK)
		{
			/** Handle the packet */
			/**
			 * @brief Packets precharge is looking for
			 * [BMS_BadCellVoltage], [BMS_BadCellTemperature],
			 * [BMS_TransmitVoltages], [BMS_TransmitTemperatures]
			 */
			if(msg.header.IDE == CAN_ID_EXT)
			{
				/** BMS_TransmitVoltages With BMSID masked off */
				if((msg.header.ExtId & BMS_ID_MASK) == Compose_CANId(CAN_PRIORITY_NORMAL, CAN_SRC_ID_BMS, 0x0, CAN_TYPE_TRANSMIT, 0x02, 0x0))
				{
					uint8_t BMSId; uint8_t vMsgId; uint16_t voltages[4];
					Parse_BMS_TransmitVoltage(msg.header.ExtId, msg.data, &BMSId, &vMsgId, voltages);
					uint8_t voltageIndexStart = vMsgId * 4; // vMsgId : start | 0:0->3, 1:4->7, 2:8->9
					if(osSemaphoreAcquire(AMS_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
					{
						for(int i = 0; i < 4; i++)
						{
							AMS_GlobalState->BMSVoltages[BMSId][voltageIndexStart + i] = voltages[i];
						}
						/** If last message, log all voltages to SD*/
						if(vMsgId == 2)
						{
						}
						osSemaphoreRelease(AMS_GlobalState->sem);
					}
				}

				/** BMS_TransmitTemperatures With BMSID masked off */
				if((msg.header.ExtId & BMS_ID_MASK) == Compose_CANId(CAN_PRIORITY_NORMAL, CAN_SRC_ID_BMS, 0x0, CAN_TYPE_TRANSMIT, 0x03, 0x0))
				{
					uint8_t BMSId; uint8_t tMsgId; uint8_t temperatures[6];
					Parse_BMS_TransmitTemperature(msg.header.ExtId, msg.data, &BMSId, &tMsgId, temperatures);

					uint8_t temperatureIndexStart = tMsgId * 6; // tMsgId : start | 0:0->5, 1:11
					if(osSemaphoreAcquire(AMS_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
					{
						for(int i = 0; i < 6; i++)
						{
							AMS_GlobalState->BMSTemperatures[BMSId][temperatureIndexStart + i] = temperatures[i];
						}
						/** If last message, log all temperatures to SD*/
						if(tMsgId == 1)
						{
						}
						osSemaphoreRelease(AMS_GlobalState->sem);
					}
				}

				Sendyne_handleVoltage(fsm, msg);

				Sendyne_handleColoumbCount(fsm, msg);

				Sendyne_handleCurrent(fsm, msg);

				BMS_handleBadCellVoltage(fsm, msg);

				BMS_handleBadCellTemperature(fsm, msg);
			}
		}
	}

	if(osSemaphoreAcquire(AMS_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
	{
		float accumulatorVoltage = 0;
		for(int i = 0; i < BMS_COUNT; i++)
		{
			for(int j = 0; j < BMS_VOLTAGE_COUNT; j++)
			{
				accumulatorVoltage += (float)(AMS_GlobalState->BMSVoltages[i][j] / 1000.0f);
			}
			accumulatorVoltage = ACCUMULATOR_VOLTAGE;

			if(fabs(fabs(AMS_GlobalState->Voltage) - ACCUMULATOR_VOLTAGE) < PRECHARGE_VDIFF)
			{
				/** Our voltage is close enough to the battery voltage, precharge done */
				char x[80];
				snprintf(x, 80, "HV Voltage Drop: %f, RTD.", fabs(fabs(AMS_GlobalState->Voltage) - ACCUMULATOR_VOLTAGE));
				fsm_changeState(fsm, &drivingState, x);
			}
		}
		osSemaphoreRelease(AMS_GlobalState->sem);
	} else {
		char msg[] = "Failed to acquire semaphore in precharge check";
		AMS_LogErr(msg, strlen(msg));
	}
	//	if(HAL_GetTick() - globalTimer > 2000)
	//	{
	//		fsm_changeState(fsm, &drivingState, "Forced into driving state on timeout");
	//	}
}

void state_precharge_exit(fsm_t *fsm)
{
	if(osTimerDelete(prechargeTimer) != osOK)
	{
		char msg[] = "Failed to Delete Precharge Request Voltage Timer";
		AMS_LogErr(msg, strlen(msg));
	}
}

void prechargeTimer_cb(void *fsm)
{
	Sendyne_requestVoltage(CS_V1);
}

state_t drivingState = {&state_driving_enter, &state_driving_iterate, &state_driving_exit, "Driving_s"};

void state_driving_enter(fsm_t *fsm)
{
	// Close Connectors

	// LOW - PRECHG
	HAL_GPIO_WritePin(PRECHG_GPIO_Port, PRECHG_Pin, GPIO_PIN_SET);

	// PROFET Positions AFTER Precharge
	// HIGH - HVA+, HVA-, HVB+, HVB-
	HAL_GPIO_WritePin(HVA_N_GPIO_Port, HVA_N_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HVB_N_GPIO_Port, HVB_N_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HVA_P_GPIO_Port, HVA_P_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(HVB_P_GPIO_Port, HVB_P_Pin, GPIO_PIN_SET);

}

void state_driving_iterate(fsm_t *fsm)
{
	while(osMessageQueueGetCount(AMS_GlobalState->CANQueue) >= 1)
	{
		AMS_CAN_Generic_t msg;
		if(osMessageQueueGet(AMS_GlobalState->CANQueue, &msg, 0U, 0U) == osOK)
		{
			/** Handle the packet */
			/**
			 * @brief Packets driving state is looking for
			 * BMS_BadCellVoltage, BMS_BadCellTemperature,
			 * BMS_TransmitVoltages, BMS_TransmitTemperatures
			 */

			if(msg.header.IDE == CAN_ID_EXT)
			{
				/** BMS_TransmitVoltages With BMSID masked off */
				if((msg.header.ExtId & BMS_ID_MASK) == Compose_CANId(CAN_PRIORITY_NORMAL, CAN_SRC_ID_BMS, 0x0, CAN_TYPE_TRANSMIT, 0x02, 0x0))
				{
					uint8_t BMSId; uint8_t vMsgId; uint16_t voltages[4];
					Parse_BMS_TransmitVoltage(msg.header.ExtId, msg.data, &BMSId, &vMsgId, voltages);
					if(BMSId > BMS_COUNT)
					{
						char msg[] = "BMS ID Outside of acceptable range!";
						AMS_LogErr(msg, strlen(msg));
						return;
					}
					uint8_t voltageIndexStart = vMsgId * 4; // vMsgId : start | 0:0->3, 1:4->7, 2:8->9
					if(osSemaphoreAcquire(AMS_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
					{
						for(int i = 0; i < 4; i++)
						{
							AMS_GlobalState->BMSVoltages[BMSId][voltageIndexStart + i] = voltages[i];
						}
						/** If last message, log all voltages to SD*/
						if(vMsgId == 2)
						{
#if BMS_LOG_V
							printf("[%li] BMS-%i: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f (V)\r\n", getRuntime(), BMSId,
									AMS_GlobalState->BMSVoltages[BMSId][0]/1000.f,
									AMS_GlobalState->BMSVoltages[BMSId][1]/1000.f,
									AMS_GlobalState->BMSVoltages[BMSId][2]/1000.f,
									AMS_GlobalState->BMSVoltages[BMSId][3]/1000.f,
									AMS_GlobalState->BMSVoltages[BMSId][4]/1000.f,
									AMS_GlobalState->BMSVoltages[BMSId][5]/1000.f,
									AMS_GlobalState->BMSVoltages[BMSId][6]/1000.f,
									AMS_GlobalState->BMSVoltages[BMSId][7]/1000.f,
									AMS_GlobalState->BMSVoltages[BMSId][8]/1000.f,
									AMS_GlobalState->BMSVoltages[BMSId][9]/1000.f);
#endif
						}
						osSemaphoreRelease(AMS_GlobalState->sem);
					}
				}

				/** BMS_TransmitTemperatures With BMSID masked off */
				if((msg.header.ExtId & BMS_ID_MASK) == Compose_CANId(CAN_PRIORITY_NORMAL, CAN_SRC_ID_BMS, 0x0, CAN_TYPE_TRANSMIT, 0x03, 0x0))
				{
					uint8_t BMSId; uint8_t tMsgId; uint8_t temperatures[6];
					Parse_BMS_TransmitTemperature(msg.header.ExtId, msg.data, &BMSId, &tMsgId, temperatures);
					if(BMSId > BMS_COUNT)
					{
						char msg[] = "BMS ID Outside of acceptable range!";
						AMS_LogErr(msg, strlen(msg));
						return;
					}
					uint8_t temperatureIndexStart = tMsgId * 6; // tMsgId : start | 0:0->5, 1:11
					if(osSemaphoreAcquire(AMS_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
					{
						for(int i = 0; i < 6; i++)
						{
							AMS_GlobalState->BMSTemperatures[BMSId][temperatureIndexStart + i] = temperatures[i];
						}
						/** If last message, log all temperatures to SD*/
						if(tMsgId == 1)
						{
#if BMS_LOG_T
							printf("[%li] BMS-%i: %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i (Degrees)\r\n", getRuntime(), BMSId,
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
									AMS_GlobalState->BMSTemperatures[BMSId][11]);
#endif
						}
						osSemaphoreRelease(AMS_GlobalState->sem);
					}
				}
			}

			BMS_handleBadCellVoltage(fsm, msg);

			BMS_handleBadCellTemperature(fsm, msg);

			Sendyne_handleColoumbCount(fsm, msg);

			Sendyne_handleCurrent(fsm, msg);

			/** CC Soft Shutdown */
			if(msg.header.ExtId == Compose_CANId(0x02, 0x16, 0x0, 0x1, 0x1, 0))
			{
				// Return to IdleState
				fsm_changeState(fsm, &idleState, "Soft Shutdown from CC");
			}
		}
	}
}

void state_driving_exit(fsm_t *fsm)
{
	// LOW - PRECHG
	HAL_GPIO_WritePin(PRECHG_GPIO_Port, PRECHG_Pin, GPIO_PIN_RESET);

	// LOW - HVA+, HVA-, HVB+, HVB-
	HAL_GPIO_WritePin(HVA_N_GPIO_Port, HVA_N_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVB_N_GPIO_Port, HVB_N_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVA_P_GPIO_Port, HVA_P_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVB_P_GPIO_Port, HVB_P_Pin, GPIO_PIN_RESET);
}

state_t errorState = {&state_error_enter, &state_error_iterate, &state_error_exit, "Error_s"};

void state_error_enter(fsm_t *fsm)
{

	// LOW - PRECHG
	HAL_GPIO_WritePin(PRECHG_GPIO_Port, PRECHG_Pin, GPIO_PIN_RESET);

	// LOW - HVA+, HVA-, HVB+, HVB-
	HAL_GPIO_WritePin(HVA_N_GPIO_Port, HVA_N_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVB_N_GPIO_Port, HVB_N_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVA_P_GPIO_Port, HVA_P_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVB_P_GPIO_Port, HVB_P_Pin, GPIO_PIN_RESET);

	// Trip Shutdown Alarm Line
	HAL_GPIO_WritePin(ALARM_CTRL_GPIO_Port, ALARM_CTRL_Pin, GPIO_PIN_SET);

	if(osSemaphoreAcquire(AMS_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
	{
		if(osTimerDelete(AMS_GlobalState->IDC_AlarmTimer) != osOK)
		{
			Error_Handler();
		}
		if(osTimerDelete(AMS_GlobalState->heartbeatTimer) != osOK)
		{
			Error_Handler();
		}
		if(osTimerDelete(AMS_GlobalState->heartbeatTimerAMS) != osOK)
		{
			Error_Handler();
		}
		if(osTimerDelete(AMS_GlobalState->ccTimer) != osOK)
		{
			Error_Handler();
		}
		if(osTimerDelete(AMS_GlobalState->cTimer) != osOK)
		{
			Error_Handler();
		}
		if(osMessageQueueDelete(AMS_GlobalState->CANQueue) != osOK)
		{
			Error_Handler();
		}

		osSemaphoreRelease(AMS_GlobalState->sem);
	}

	// Disable CAN Interrupts
	HAL_CAN_DeactivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_DeactivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void state_error_iterate(fsm_t *fsm)
{
	do{
		// We cannot escape from here. We are forever hitting UART.
		// Trip Shutdown Alarm Line
		HAL_GPIO_WritePin(ALARM_CTRL_GPIO_Port, ALARM_CTRL_Pin, GPIO_PIN_SET);
		AMS_LogErr("Stuck in error state", strlen("Stuck in error state"));
		HAL_Delay(100);
	} while(1);
}

void state_error_exit(fsm_t *fsm)
{
	Error_Handler();
	return; // We should never get here.
}

state_t resetState = {&state_reset_enter, &state_reset_iterate, &state_reset_exit, "Reset_s"};

void state_reset_enter(fsm_t *fsm)
{
	// LOW - PRECHG
	HAL_GPIO_WritePin(PRECHG_GPIO_Port, PRECHG_Pin, GPIO_PIN_RESET);

	// LOW - HVA+, HVA-, HVB+, HVB-
	HAL_GPIO_WritePin(HVA_N_GPIO_Port, HVA_N_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVB_N_GPIO_Port, HVB_N_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVA_P_GPIO_Port, HVA_P_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVB_P_GPIO_Port, HVB_P_Pin, GPIO_PIN_RESET);
}

void state_reset_iterate(fsm_t *fsm)
{
	fsm_changeState(fsm, &idleState, "Resetting to Idle");
}

void state_reset_exit(fsm_t *fsm)
{
	return;
}

state_t SoCState = {&state_SoC_enter, &state_SoC_iterate, &state_SoC_exit, "SoC_s"};

void state_SoC_enter(fsm_t *fsm)
{
	/** We need 1 voltage packet from each BMS */
}

void state_SoC_iterate(fsm_t *fsm)
{
	while(osMessageQueueGetCount(AMS_GlobalState->CANQueue) >= 1)
	{
		AMS_CAN_Generic_t msg;
		if(osMessageQueueGet(AMS_GlobalState->CANQueue, &msg, 0U, 0U) == osOK)
		{
			/** Handle the packet */
			/**
			 * @brief Packets driving state is looking for
			 * BMS_BadCellVoltage, BMS_BadCellTemperature,
			 * BMS_TransmitVoltages, BMS_TransmitTemperatures CC_Voltage
			 */
			if(msg.header.IDE == CAN_ID_EXT)
			{
				/** BMS_TransmitVoltages With BMSID masked off */
				if((msg.header.ExtId & BMS_ID_MASK) == Compose_CANId(CAN_PRIORITY_NORMAL, CAN_SRC_ID_BMS, 0x0, CAN_TYPE_TRANSMIT, 0x02, 0x0))
				{
					uint8_t BMSId; uint8_t vMsgId; uint16_t voltages[4];
					Parse_BMS_TransmitVoltage(msg.header.ExtId, msg.data, &BMSId, &vMsgId, voltages);
					if(BMSId > BMS_COUNT)
					{
						char msg[] = "BMS ID Outside of acceptable range!";
						AMS_LogErr(msg, strlen(msg));
						return;
					}
					uint8_t voltageIndexStart = vMsgId * 4; // vMsgId : start | 0:0->3, 1:4->7, 2:8->9
					if(osSemaphoreAcquire(AMS_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
					{
						for(int i = 0; i < 4; i++)
						{
							AMS_GlobalState->BMSVoltages[BMSId][voltageIndexStart + i] = voltages[i];
						}
						/** If last message, log all voltages to SD*/
						if(vMsgId == 2)
						{
#if BMS_LOG_V
							printf("[%li] BMS-%i: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f (V)\r\n", getRuntime(), BMSId,
									AMS_GlobalState->BMSVoltages[BMSId][0]/1000.f,
									AMS_GlobalState->BMSVoltages[BMSId][1]/1000.f,
									AMS_GlobalState->BMSVoltages[BMSId][2]/1000.f,
									AMS_GlobalState->BMSVoltages[BMSId][3]/1000.f,
									AMS_GlobalState->BMSVoltages[BMSId][4]/1000.f,
									AMS_GlobalState->BMSVoltages[BMSId][5]/1000.f,
									AMS_GlobalState->BMSVoltages[BMSId][6]/1000.f,
									AMS_GlobalState->BMSVoltages[BMSId][7]/1000.f,
									AMS_GlobalState->BMSVoltages[BMSId][8]/1000.f,
									AMS_GlobalState->BMSVoltages[BMSId][9]/1000.f);
#endif
						}
						osSemaphoreRelease(AMS_GlobalState->sem);
					}
				}

				/** BMS_TransmitTemperatures With BMSID masked off */
				if((msg.header.ExtId & BMS_ID_MASK) == Compose_CANId(CAN_PRIORITY_NORMAL, CAN_SRC_ID_BMS, 0x0, CAN_TYPE_TRANSMIT, 0x03, 0x0))
				{
					uint8_t BMSId; uint8_t tMsgId; uint8_t temperatures[6];
					Parse_BMS_TransmitTemperature(msg.header.ExtId, msg.data, &BMSId, &tMsgId, temperatures);
					if(BMSId > BMS_COUNT)
					{
						char msg[] = "BMS ID Outside of acceptable range!";
						AMS_LogErr(msg, strlen(msg));
						return;
					}
					uint8_t temperatureIndexStart = tMsgId * 6; // tMsgId : start | 0:0->5, 1:11
					if(osSemaphoreAcquire(AMS_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
					{
						for(int i = 0; i < 6; i++)
						{
							AMS_GlobalState->BMSTemperatures[BMSId][temperatureIndexStart + i] = temperatures[i];
						}
						/** If last message, log all temperatures to SD*/
						if(tMsgId == 1)
						{
#if BMS_LOG_T
							printf("[%li] BMS-%i: %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i (Degrees)\r\n", getRuntime(), BMSId,
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
									AMS_GlobalState->BMSTemperatures[BMSId][11]);
#endif
						}
						osSemaphoreRelease(AMS_GlobalState->sem);
					}
				}

				BMS_handleBadCellVoltage(fsm, msg);

				BMS_handleBadCellTemperature(fsm, msg);
			}
		}
	}

	if(osSemaphoreAcquire(AMS_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
	{
		int i = 0;
		while(i < BMS_COUNT)
		{
			if(AMS_GlobalState->BMSVoltages[i][0]/1000.0f < BMS_CELL_VMIN)
			{
				break;
			}
			i++;
		}
		if(i == BMS_COUNT)
		{
			fsm_changeState(fsm, &idleState, "All BMSs awake, move to idle");
		}
		osSemaphoreRelease(AMS_GlobalState->sem);
	}

	if(HAL_GetTick() - AMS_GlobalState->startupTicks > 2000)
	{
		fsm_changeState(fsm, &idleState, "Timeout of SoC, moving to idle");
	}
}

void state_SoC_exit(fsm_t *fsm)
{
	return;
}

void BMS_handleBadCellVoltage(fsm_t *fsm, AMS_CAN_Generic_t msg)
{
	/** BMS_BadCellVoltage With BMSID masked off */
	if((msg.header.ExtId & BMS_ID_MASK) == Compose_CANId(0x0, 0x12, 0x0, 0x0, 0x00, 0x0))
	{
		uint8_t BMSId; uint8_t cellNum; uint8_t voltage;
		Parse_BMS_BadCellVoltage(msg.data, &BMSId, &cellNum, &voltage);

		AMS_CellVoltageShutdown_t cVS = Compose_AMS_CellVoltageShutdown(cellNum, BMSId, voltage);
		CAN_TxHeaderTypeDef header =
		{
				.ExtId = cVS.id,
				.IDE = CAN_ID_EXT,
				.RTR = CAN_RTR_DATA,
				.DLC = sizeof(cVS.data),
				.TransmitGlobalTime = DISABLE,
		};

		// Notify Chassis we have a bad cell voltage.
		HAL_CAN_AddTxMessage(&hcan1, &header, cVS.data, &AMS_GlobalState->CAN2_TxMailbox);

		char x[80];
		int len = snprintf(x, 80, "Found Bad Cell Voltage: Cell:%i, Voltage: %i", cellNum, voltage);

		// Bad BMS cell voltage found, we need to change to errorState.
		AMS_LogErr(x, len);
		fsm_changeState(fsm, &errorState, "Found Bad BMS Cell Voltage");
	}
}

void BMS_handleBadCellTemperature(fsm_t *fsm, AMS_CAN_Generic_t msg)
{
	/** BMS_BadCellTemperature With BMSID masked off */
	if((msg.header.ExtId & BMS_ID_MASK) == Compose_CANId(0x0, 0x12, 0x0, 0x0, 0x01, 0x0))
	{
		uint8_t BMSId; uint8_t cellNum; uint8_t temperature;
		Parse_BMS_BadCellTemperature(msg.data, &BMSId, &cellNum, &temperature);

		AMS_CellTemperatureShutdown_t cTS = Compose_AMS_CellTemperatureShutdown(cellNum, BMSId, temperature);
		CAN_TxHeaderTypeDef header =
		{
				.ExtId = cTS.id,
				.IDE = CAN_ID_EXT,
				.RTR = CAN_RTR_DATA,
				.DLC = sizeof(cTS.data),
				.TransmitGlobalTime = DISABLE,
		};

		// Notify Chassis we have a bad cell temperature.
		HAL_CAN_AddTxMessage(&hcan1, &header, cTS.data, &AMS_GlobalState->CAN2_TxMailbox);

		char x[80];
		int len = snprintf(x, 80, "Found Bad Cell Temperature: Cell:%i, Temp: %i", cellNum, temperature);

		// Bad BMS cell temperature found, we need to change to errorState
		AMS_LogErr(x, len);
		//		fsm_changeState(fsm, &errorState, "Found Bad BMS Cell Temperature");
	}
}

void Sendyne_handleVoltage(fsm_t *fsm, AMS_CAN_Generic_t msg)
{
	/** CS */
	if((msg.header.ExtId) == CS_1_RESPONSE_EXTID)
	{
		/** Voltage */
		if(msg.data[0] == CS_V1)
		{
			if(osSemaphoreAcquire(AMS_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
			{
				AMS_GlobalState->VoltageuV = 0;
				AMS_GlobalState->VoltageuV |= (int32_t)msg.data[1] << 24;
				AMS_GlobalState->VoltageuV |= (int32_t)msg.data[2] << 16;
				AMS_GlobalState->VoltageuV |= (int32_t)msg.data[3] << 8;
				AMS_GlobalState->VoltageuV |= (int32_t)msg.data[4] << 0;
				AMS_GlobalState->Voltage = AMS_GlobalState->VoltageuV / 1000000.0f;

				AMS_GlobalState->Voltage = (float)(AMS_GlobalState->VoltageuV / 1000000.f);
				printf("[%li] Voltage: %f\r\n", getRuntime(), AMS_GlobalState->Voltage);

				osSemaphoreRelease(AMS_GlobalState->sem);
			}
		}
	}
}

void Sendyne_handleColoumbCount(fsm_t *fsm, AMS_CAN_Generic_t msg)
{
	/** Current Sensor Coulomb Counting */
	if(msg.header.ExtId == CS_1_RESPONSE_EXTID)
	{
		if(msg.data[0] == CURRENT_SENSOR_CC_LOW)
		{
			if(osSemaphoreAcquire(AMS_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
			{
				AMS_GlobalState->CoulombCountuA = 0;
				AMS_GlobalState->CoulombCountuA |= (int64_t)msg.data[1] << 24;
				AMS_GlobalState->CoulombCountuA |= (int64_t)msg.data[2] << 16;
				AMS_GlobalState->CoulombCountuA |= (int64_t)msg.data[3] << 8;
				AMS_GlobalState->CoulombCountuA |= (int64_t)msg.data[4] << 0;

				osSemaphoreRelease(AMS_GlobalState->sem);

			}
		} else if(msg.data[0] == CURRENT_SENSOR_CC_HIGH)
		{
			if(osSemaphoreAcquire(AMS_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
			{
				AMS_GlobalState->CoulombCountuA |= (int64_t)msg.data[1] << 56;
				AMS_GlobalState->CoulombCountuA |= (int64_t)msg.data[2] << 48;
				AMS_GlobalState->CoulombCountuA |= (int64_t)msg.data[3] << 40;
				AMS_GlobalState->CoulombCountuA |= (int64_t)msg.data[4] << 32;

				AMS_GlobalState->CoulombCount = (float)(AMS_GlobalState->CoulombCountuA / 1000000.f);

#if CS_LOG_CC
				printf(x, "[%li] Coloumb Count: %f\r\n", getRuntime(), AMS_GlobalState->CoulombCount);
#endif
				osSemaphoreRelease(AMS_GlobalState->sem);
			}
		}
	}
}

void Sendyne_handleCurrent(fsm_t *fsm, AMS_CAN_Generic_t msg)
{
	if(msg.header.ExtId == CS_1_RESPONSE_EXTID)
	{
		if(msg.data[0] == CURRENT_SENSOR_CURRENT)
		{
			if(osSemaphoreAcquire(AMS_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
			{
				AMS_GlobalState->HVACurrentuA = 0;
				AMS_GlobalState->HVACurrentuA |= (int64_t)msg.data[1] << 24;
				AMS_GlobalState->HVACurrentuA |= (int64_t)msg.data[2] << 16;
				AMS_GlobalState->HVACurrentuA |= (int64_t)msg.data[3] << 8;
				AMS_GlobalState->HVACurrentuA |= (int64_t)msg.data[4] << 0;

				AMS_GlobalState->HVACurrent = (float)(AMS_GlobalState->HVACurrentuA / 1000000.f);

				osSemaphoreRelease(AMS_GlobalState->sem);
			}
		}
	} else if(msg.header.ExtId == CS_2_RESPONSE_EXTID)
	{
		if(msg.data[0] == CURRENT_SENSOR_CURRENT)
		{
			if(osSemaphoreAcquire(AMS_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
			{
				AMS_GlobalState->HVBCurrentuA = 0;
				AMS_GlobalState->HVBCurrentuA |= (int64_t)msg.data[1] << 24;
				AMS_GlobalState->HVBCurrentuA |= (int64_t)msg.data[2] << 16;
				AMS_GlobalState->HVBCurrentuA |= (int64_t)msg.data[3] << 8;
				AMS_GlobalState->HVBCurrentuA |= (int64_t)msg.data[4] << 0;

				AMS_GlobalState->HVBCurrent = (float)(AMS_GlobalState->HVBCurrentuA / 1000000.f);

				osSemaphoreRelease(AMS_GlobalState->sem);
			}
		}
	}
}



