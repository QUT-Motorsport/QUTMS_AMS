/**
 ******************************************************************************
 * @file AMS_FSM_States.c
 * @brief AMS FSM States
 ******************************************************************************
 */

#include <AMS_FSM_States.h>

extern bool charge;

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
	if (vMsgId == 2 && charge) {
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
	if (tMsgId == 2 && charge) {
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
				"Found Bad Cell Voltage: id: %i, Cell:%i, Voltage: %i", BMSId, cellNum,
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
			AMS_GlobalState->Voltage = AMS_GlobalState->VoltageuV / 1000000.0f;

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

			CAN_TxHeaderTypeDef h = { .ExtId = Compose_CANId(CAN_PRIORITY_DEBUG,
					sourceId, autonomous, type, extra, BMSId),
					.IDE = CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC =
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

			AMS_GlobalState->HVACurrent = (float) (AMS_GlobalState->HVACurrentuA
					/ 1000000.f);

			/** Send Voltage Log Msg to CC */

			uint8_t priority;
			uint16_t sourceId;
			uint8_t autonomous;
			uint8_t type;
			uint16_t extra;
			uint8_t BMSId;
			Parse_CANId(msg.header.ExtId, &priority, &sourceId, &autonomous,
					&type, &extra, &BMSId);

			CAN_TxHeaderTypeDef h = { .ExtId = Compose_CANId(CAN_PRIORITY_DEBUG,
					sourceId, autonomous, type, extra, BMSId),
					.IDE = CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC =
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

			AMS_GlobalState->HVBCurrent = (float) (AMS_GlobalState->HVBCurrentuA
					/ 1000000.f);

		}
	}
}

