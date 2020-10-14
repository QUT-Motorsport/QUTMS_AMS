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
	if(AMS_GlobalState == NULL)
	{
		AMS_GlobalState = malloc(sizeof(AMS_GlobalState_t));
		memset(AMS_GlobalState, 0, sizeof(AMS_GlobalState_t));

		// As AMS_GlobalState is accessible across threads, we need to use a semaphore to access it
		AMS_GlobalState->sem = osSemaphoreNew(3U, 3U, NULL);
		if(osSemaphoreAcquire(AMS_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
		{
			AMS_GlobalState->heartbeatTimer = osTimerNew(&heartbeatTimer_cb, osTimerPeriodic, fsm, NULL);
			if(osTimerStart(AMS_GlobalState->heartbeatTimer, AMS_HEARTBEAT_PERIOD) != osOK)
			{
				Error_Handler();
			}
			AMS_GlobalState->IDC_AlarmTimer = osTimerNew(&IDC_Alarm_cb, osTimerPeriodic, fsm, NULL);
			if(osTimerStart(AMS_GlobalState->IDC_AlarmTimer, AMS_IDC_PERIOD) != osOK)
			{
				Error_Handler();
			}

			AMS_GlobalState->CANQueue = osMessageQueueNew(AMS_CAN_QUEUESIZE, sizeof(AMS_CAN_Generic_t), NULL);
			if(AMS_GlobalState->CANQueue == NULL)
			{
				Error_Handler();
			}
			AMS_GlobalState->startupTicks = HAL_GetTick();
			osSemaphoreRelease(AMS_GlobalState->sem);
		}
	}

	/* Set initial pin states */
	// ALARM Line - HIGH
	HAL_GPIO_WritePin(ALARM_CTRL_GPIO_Port, ALARM_CTRL_Pin, GPIO_PIN_SET);

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

			/** BMS_TransmitVoltages With BMSID masked off */
			if((msg.header.ExtId & BMS_ID_MASK) == Compose_CANId(0x2, 0x12, 0x0, 0x3, 0x02, 0x0))
			{
				uint8_t BMSId; uint8_t vMsgId; uint16_t voltages[4];
				Parse_BMS_TransmitVoltage(*((BMS_TransmitVoltage_t*)&(msg.data)), &BMSId, &vMsgId, voltages);

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
						AMS_LogToSD((char*)&(AMS_GlobalState->BMSVoltages[BMSId][0]), BMS_VOLTAGE_COUNT * (sizeof(uint16_t)/sizeof(char)));
					}
					osSemaphoreRelease(AMS_GlobalState->sem);
				}
			}

			/** BMS_TransmitTemperatures With BMSID masked off */
			if((msg.header.ExtId & BMS_ID_MASK) == Compose_CANId(0x2, 0x12, 0x0, 0x3,0x03, 0x0))
			{
				uint8_t BMSId; uint8_t tMsgId; uint8_t temperatures[6];
				Parse_BMS_TransmitTemperature(*((BMS_TransmitTemperature_t*)&(msg.data)), &BMSId, &tMsgId, temperatures);

				uint8_t temperatureIndexStart = tMsgId * 6; // tMsgId : start | 0:0->5, 1:11
				if(osSemaphoreAcquire(AMS_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
				{
					for(int i = 0; i < 6; i++)
					{
						AMS_GlobalState->BMSTemperatues[BMSId][temperatureIndexStart + i] = temperatures[i];
					}
					/** If last message, log all temperatures to SD*/
					if(tMsgId == 1)
					{
						AMS_LogToSD((char*)&(AMS_GlobalState->BMSTemperatues[BMSId][0]), BMS_TEMPERATURE_COUNT);
					}
					osSemaphoreRelease(AMS_GlobalState->sem);
				}
			}

			/** BMS_BadCellVoltage With BMSID masked off */
			if((msg.header.ExtId & BMS_ID_MASK) == Compose_CANId(0x0, 0x12, 0x0, 0x0, 0x00, 0x0))
			{
				uint8_t BMSId; uint8_t cellNum; uint8_t voltage;
				Parse_BMS_BadCellVoltage(*((BMS_BadCellVoltage_t *)(&msg.data)), &BMSId, &cellNum, &voltage);

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

				// Bad BMS cell voltage found, we need to change to errorState.
				fsm_changeState(fsm, &errorState, "Found Bad BMS Cell Voltage");
			}

			/** BMS_BadCellTemperature With BMSID masked off */
			if((msg.header.ExtId & BMS_ID_MASK) == Compose_CANId(0x0, 0x12, 0x0, 0x0, 0x01, 0x0))
			{
				uint8_t BMSId; uint8_t cellNum; uint8_t temperature;
				Parse_BMS_BadCellTemperature(*((BMS_BadCellTemperature_t *)(&msg.data)), &BMSId, &cellNum, &temperature);

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

				// Bad BMS cell temperature found, we need to change to errorState
				fsm_changeState(fsm, &errorState, "Found Bad BMS Cell Temperature");
			}

			/**
			 * TODO: ChargeEnabled & RTD
			 */
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

	prechargeTimer = osTimerNew(&prechargeTimer_cb, osTimerOnce, fsm, NULL);
	if(osTimerStart(prechargeTimer, PRECHARGE_DELAY) != osOK)
	{
		Error_Handler();
	}
}

void state_precharge_iterate(fsm_t *fsm)
{
	return; // In precharge state, wait for timer cb.

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

			/** BMS_TransmitVoltages With BMSID masked off */
			if((msg.header.ExtId & BMS_ID_MASK) == Compose_CANId(0x2, 0x12, 0x0, 0x3, 0x02, 0x0))
			{
				uint8_t BMSId; uint8_t vMsgId; uint16_t voltages[4];
				Parse_BMS_TransmitVoltage(*((BMS_TransmitVoltage_t*)&(msg.data)), &BMSId, &vMsgId, voltages);

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
						AMS_LogToSD((char*)&(AMS_GlobalState->BMSVoltages[BMSId][0]), BMS_VOLTAGE_COUNT * (sizeof(uint16_t)/sizeof(char)));
					}
					osSemaphoreRelease(AMS_GlobalState->sem);
				}
			}

			/** BMS_TransmitTemperatures With BMSID masked off */
			if((msg.header.ExtId & BMS_ID_MASK) == Compose_CANId(0x2, 0x12, 0x0, 0x3,0x03, 0x0))
			{
				uint8_t BMSId; uint8_t tMsgId; uint8_t temperatures[6];
				Parse_BMS_TransmitTemperature(*((BMS_TransmitTemperature_t*)&(msg.data)), &BMSId, &tMsgId, temperatures);

				uint8_t temperatureIndexStart = tMsgId * 6; // tMsgId : start | 0:0->5, 1:11
				if(osSemaphoreAcquire(AMS_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
				{
					for(int i = 0; i < 6; i++)
					{
						AMS_GlobalState->BMSTemperatues[BMSId][temperatureIndexStart + i] = temperatures[i];
					}
					/** If last message, log all temperatures to SD*/
					if(tMsgId == 1)
					{
						AMS_LogToSD((char*)&(AMS_GlobalState->BMSTemperatues[BMSId][0]), BMS_TEMPERATURE_COUNT);
					}
					osSemaphoreRelease(AMS_GlobalState->sem);
				}
			}

			/** BMS_BadCellVoltage With BMSID masked off */
			if((msg.header.ExtId & BMS_ID_MASK) == Compose_CANId(0x0, 0x12, 0x0, 0x0, 0x00, 0x0))
			{
				uint8_t BMSId; uint8_t cellNum; uint8_t voltage;
				Parse_BMS_BadCellVoltage(*((BMS_BadCellVoltage_t *)(&msg.data)), &BMSId, &cellNum, &voltage);

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

				// Bad BMS cell voltage found, we need to change to errorState.
				fsm_changeState(fsm, &errorState, "Found Bad BMS Cell Voltage");
			}

			/** BMS_BadCellTemperature With BMSID masked off */
			if((msg.header.ExtId & BMS_ID_MASK) == Compose_CANId(0x0, 0x12, 0x0, 0x0, 0x01, 0x0))
			{
				uint8_t BMSId; uint8_t cellNum; uint8_t temperature;
				Parse_BMS_BadCellTemperature(*((BMS_BadCellTemperature_t *)(&msg.data)), &BMSId, &cellNum, &temperature);

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

				// Bad BMS cell temperature found, we need to change to errorState
				fsm_changeState(fsm, &errorState, "Found Bad BMS Cell Temperature");
			}
		}
	}
}

void state_precharge_exit(fsm_t *fsm)
{
	if(osTimerDelete(prechargeTimer) != osOK)
	{
		Error_Handler();
	}
}

void prechargeTimer_cb(void *fsm)
{
	fsm_changeState((fsm_t *)fsm, &drivingState, "Precharge Done, Now RTD");
}

state_t drivingState = {&state_driving_enter, &state_driving_iterate, &state_driving_exit, "Driving_s"};

void state_driving_enter(fsm_t *fsm)
{
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

void state_driving_iterate(fsm_t *fsm)
{
	while(osMessageQueueGetCount(AMS_GlobalState->CANQueue) >= 1)
	{
		AMS_CAN_Generic_t msg;
		if(osMessageQueueGet(AMS_GlobalState->CANQueue, &msg, 0U, 0U))
		{
			/** Handle the packet */
			/**
			 * @brief Packets driving state is looking for
			 * BMS_BadCellVoltage, BMS_BadCellTemperature,
			 * BMS_TransmitVoltages, BMS_TransmitTemperatures
			 */

			/** BMS_TransmitVoltages With BMSID masked off */
			if((msg.header.ExtId & BMS_ID_MASK) == Compose_CANId(0x2, 0x12, 0x0, 0x3, 0x02, 0x0))
			{
				uint8_t BMSId; uint8_t vMsgId; uint16_t voltages[4];
				Parse_BMS_TransmitVoltage(*((BMS_TransmitVoltage_t*)&(msg.data)), &BMSId, &vMsgId, voltages);

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
						AMS_LogToSD((char*)&(AMS_GlobalState->BMSVoltages[BMSId][0]), BMS_VOLTAGE_COUNT * (sizeof(uint16_t)/sizeof(char)));
					}
					osSemaphoreRelease(AMS_GlobalState->sem);
				}
			}

			/** BMS_TransmitTemperatures With BMSID masked off */
			if((msg.header.ExtId & BMS_ID_MASK) == Compose_CANId(0x2, 0x12, 0x0, 0x3,0x03, 0x0))
			{
				uint8_t BMSId; uint8_t tMsgId; uint8_t temperatures[6];
				Parse_BMS_TransmitTemperature(*((BMS_TransmitTemperature_t*)&(msg.data)), &BMSId, &tMsgId, temperatures);

				uint8_t temperatureIndexStart = tMsgId * 6; // tMsgId : start | 0:0->5, 1:11
				if(osSemaphoreAcquire(AMS_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
				{
					for(int i = 0; i < 6; i++)
					{
						AMS_GlobalState->BMSTemperatues[BMSId][temperatureIndexStart + i] = temperatures[i];
					}
					/** If last message, log all temperatures to SD*/
					if(tMsgId == 1)
					{
						AMS_LogToSD((char*)&(AMS_GlobalState->BMSTemperatues[BMSId][0]), BMS_TEMPERATURE_COUNT);
					}
					osSemaphoreRelease(AMS_GlobalState->sem);
				}
			}

			/** BMS_BadCellVoltage With BMSID masked off */
			if((msg.header.ExtId & BMS_ID_MASK) == Compose_CANId(0x0, 0x12, 0x0, 0x0, 0x00, 0x0))
			{
				uint8_t BMSId; uint8_t cellNum; uint8_t voltage;
				Parse_BMS_BadCellVoltage(*((BMS_BadCellVoltage_t *)(&msg.data)), &BMSId, &cellNum, &voltage);

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

				// Bad BMS cell voltage found, we need to change to errorState.
				fsm_changeState(fsm, &errorState, "Found Bad BMS Cell Voltage");
			}

			/** BMS_BadCellTemperature With BMSID masked off */
			if((msg.header.ExtId & BMS_ID_MASK) == Compose_CANId(0x0, 0x12, 0x0, 0x0, 0x01, 0x0))
			{
				uint8_t BMSId; uint8_t cellNum; uint8_t temperature;
				Parse_BMS_BadCellTemperature(*((BMS_BadCellTemperature_t *)(&msg.data)), &BMSId, &cellNum, &temperature);

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

				// Bad BMS cell temperature found, we need to change to errorState
				fsm_changeState(fsm, &errorState, "Found Bad BMS Cell Voltage");
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
	AMS_LogToSD("Entered Error State. Oh No", strlen("Entered Error State. Oh No"));

	// LOW - PRECHG
	HAL_GPIO_WritePin(PRECHG_GPIO_Port, PRECHG_Pin, GPIO_PIN_RESET);

	// LOW - HVA+, HVA-, HVB+, HVB-
	HAL_GPIO_WritePin(HVA_N_GPIO_Port, HVA_N_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVB_N_GPIO_Port, HVB_N_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVA_P_GPIO_Port, HVA_P_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVB_P_GPIO_Port, HVB_P_Pin, GPIO_PIN_RESET);

	// Trip Shutdown Alarm Line
	HAL_GPIO_WritePin(ALARM_CTRL_GPIO_Port, ALARM_CTRL_Pin, GPIO_PIN_RESET);

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
		HAL_GPIO_WritePin(ALARM_CTRL_GPIO_Port, ALARM_CTRL_Pin, GPIO_PIN_RESET);
		AMS_LogErr("Stuck in error state", strlen("Stuck in error state"));
		HAL_Delay(50);
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
