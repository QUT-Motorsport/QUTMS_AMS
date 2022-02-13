/*
 * bms.c
 *
 *  Created on: 1 Jan. 2022
 *      Author: Calvin
 */

#include "bms.h"
#include "main.h"
#include "can.h"
#include "heartbeat.h"
#include "states.h"
#include <FSM.h>

ms_timer_t bms_timer;
bms_status_t bms;

void bms_ctrl_off() {
	HAL_GPIO_WritePin(BMS_CTRL_GPIO_Port, BMS_CTRL_Pin, GPIO_PIN_RESET);
}

void bms_ctrl_on() {
	HAL_GPIO_WritePin(BMS_CTRL_GPIO_Port, BMS_CTRL_Pin, GPIO_PIN_SET);
}

void bms_setup() {
	bms_timer = timer_init(5, true, bms_CAN_timer_cb);

	for (int i = 0; i < BMS_COUNT; i++) {
		for (int j = 0; j < BMS_VOLT_COUNT || j < BMS_TEMP_COUNT; j++) {
			if (j < BMS_VOLT_COUNT) {
				bms.voltages[i][j] = 3300;
				bmu_window_filter_initialize(&bms.voltage_filters[i][j], bms.voltages[i][j], FILTER_SIZE_VOLT);
			}

			if (j < BMS_TEMP_COUNT) {
				bms.temperatures[i][j] = 25;
				bmu_window_filter_initialize(&bms.temperature_filters[i][j], bms.temperatures[i][j], FILTER_SIZE_TEMP);
			}
		}
	}

	timer_start(&bms_timer);
}

void bms_CAN_timer_cb(void *args) {
	CAN_MSG_Generic_t msg;
	while (queue_next(&queue_CAN_BMS, &msg)) {
		uint32_t masked_id = (msg.ID & ~0xF);
		uint8_t idx = (msg.ID & 0xF);

		if (idx < 1 || idx > BMS_COUNT) {
			printf("Invalid BMS ID: %d, MSG: 0x%08X\r\n", idx);
		} else {
			// update heartbeat state
			heartbeats.BMS[idx-1] = true;
			heartbeats.hb_BMS_start[idx-1] = HAL_GetTick();
			AMS_hbState.bmsStatus |= 1 << (idx-1);

			if (masked_id == BMS_TransmitVoltage_ID) {
				bms_handleVoltageMsg(&msg);
			} else if (masked_id == BMS_TransmitTemperature_ID) {
				bms_handleTemperatureMsg(&msg);
			} else if (masked_id == BMS_BadCellVoltage_ID) {

			} else if (masked_id == BMS_BadCellTemperature_ID) {

			}
		}
	}

	bool bad_volt_or_temp = false;
	for (int i = 0; i < BMS_COUNT; i++) {
		for (int j = 0; j < BMS_VOLT_COUNT || j < BMS_TEMP_COUNT; j++) {
			if (j < BMS_VOLT_COUNT) {
				uint16_t volt_filtered = bms.voltage_filters[i][j].current_filtered;
				if (volt_filtered > BMS_VOLT_OVER || volt_filtered < BMS_VOLT_UNDER) {
					bad_volt_or_temp = true;
					AMS_hbState.flags.BMS_OVER_VOLT = (volt_filtered > BMS_VOLT_OVER) ? 1 : 0;
					AMS_hbState.flags.BMS_UNDER_VOLT = (volt_filtered < BMS_VOLT_UNDER) ? 1 : 0;
				}
			}

			if (j < BMS_TEMP_COUNT) {
				uint8_t temp_filtered = bms.temperature_filters[i][j].current_filtered;
				if (temp_filtered > BMS_TEMP_OVER && temp_filtered < BMS_TEMP_BAD_CUTOFF) {
					bad_volt_or_temp = true;
					AMS_hbState.flags.BMS_BAD_TEMP = 1;
				}
			}
		}
	}

	if (bad_volt_or_temp)
		fsm_changeState(&fsm, &state_error, "bad voltage or temperature detected");
}

void bms_handleVoltageMsg(CAN_MSG_Generic_t *msg) {
	uint8_t idx = (msg->ID & 0xF);
	uint8_t msgId;
	uint16_t voltages[BMS_VOLT_PACK_COUNT];

	Parse_BMS_TransmitVoltage(msg->data, &msgId, voltages);

	for (int i = 0; ((i < BMS_VOLT_PACK_COUNT) && ( ((msgId * BMS_VOLT_PACK_COUNT) + i) < BMS_VOLT_COUNT)); i++) {
		bms.voltages[idx-1][(msgId * BMS_VOLT_PACK_COUNT) + i] = voltages[i];
		bmu_window_filter_update(
			&bms.voltage_filters[idx-1][(msgId * BMS_VOLT_PACK_COUNT) + i],
			bms.voltages[idx-1][(msgId * BMS_VOLT_PACK_COUNT) + i]
		);
	}
}

void bms_handleTemperatureMsg(CAN_MSG_Generic_t *msg) {
	uint8_t idx = (msg->ID & 0xF);
	uint8_t msgId;
	uint8_t temperatures[BMS_TEMP_PACK_COUNT];

	Parse_BMS_TransmitTemperature(msg->data, &msgId, temperatures);

	for (int i = 0; i < BMS_TEMP_PACK_COUNT; i++) {
		bms.temperatures[idx-1][(msgId * BMS_TEMP_PACK_COUNT) + i] = temperatures[i];
		bmu_window_filter_update(
			&bms.temperature_filters[idx-1][(msgId * BMS_TEMP_PACK_COUNT) + i],
			bms.temperatures[idx-1][(msgId * BMS_TEMP_PACK_COUNT) + i]
		);
	}
}
