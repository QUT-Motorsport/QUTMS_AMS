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
ms_timer_t bms_reboot_timer;
bms_status_t bms;

void bms_ctrl_off() {
	HAL_GPIO_WritePin(BMS_CTRL_GPIO_Port, BMS_CTRL_Pin, GPIO_PIN_RESET);
}

void bms_ctrl_on() {
	HAL_GPIO_WritePin(BMS_CTRL_GPIO_Port, BMS_CTRL_Pin, GPIO_PIN_SET);
}

void bms_setup() {
	bms_timer = timer_init(5, true, bms_CAN_timer_cb);
	bms_reboot_timer = timer_init(BMS_BOOT_TIME, false, bms_reboot_timer_cb);

	for (int i = 0; i < BMS_COUNT; i++) {
		for (int j = 0; j < BMS_VOLT_COUNT; j++) {
			bms.voltages[i][j] = 3300;
			bmu_window_filter_initialize(&bms.voltage_filters[i][j], bms.voltages[i][j], FILTER_SIZE_VOLT);
		}

		for (int j = 0; j < BMS_TEMP_COUNT; j++) {
			bms.temperatures[i][j] = 25;
			bms.temperature_times[i][j] = HAL_GetTick(); // NOTE: or 0?
			bmu_window_filter_initialize(&bms.temperature_filters[i][j], bms.temperatures[i][j], FILTER_SIZE_TEMP);
		}
	}

	timer_start(&bms_timer);
}

void bms_reboot_timer_cb(void *args) {
	// its been 75ms, so turn ctrl line off
	bms_ctrl_off();
}

void bms_CAN_timer_cb(void *args) {
	CAN_MSG_Generic_t msg;
	while (queue_next(&queue_CAN_BMS, &msg)) {
		uint32_t masked_id = (msg.ID & ~0xF);
		uint8_t idx = (msg.ID & 0xF);

		if (idx < 1 || idx > BMS_COUNT) {
			printf("Invalid BMS ID: %d, MSG: 0x%08luX\r\n", idx, masked_id);
		}
		else {
			// update heartbeat state
			heartbeats.BMS[idx - 1] = true;
			heartbeats.hb_BMS_start[idx - 1] = HAL_GetTick();
			AMS_hbState.bmsStatus |= 1 << (idx - 1);

			if (masked_id == BMS_TransmitVoltage_ID) {
				bms_handleVoltageMsg(&msg);
			}
			else if (masked_id == BMS_TransmitTemperature_ID) {
				bms_handleTemperatureMsg(&msg);
			}
			else if (masked_id == BMS_BadCellVoltage_ID) {

			}
			else if (masked_id == BMS_BadCellTemperature_ID) {

			}
		}
	}

	if (!check_bms_heartbeat()) {
		if (!bms_reboot_timer.isRunning) {
			// bms has dropped out, so turn the ctrl line on to reboot any dead ones
			bms_ctrl_on();
			timer_start(&bms_reboot_timer);
		}
	}

	bool bad_volt = false;
	bool bad_temp = false;
	int bad_i = 0;
	int bad_j = 0;

	for (int i = 0; i < BMS_COUNT; i++) {
		for (int j = 0; j < BMS_VOLT_COUNT; j++) {
			uint16_t volt_filtered = bms.voltage_filters[i][j].current_filtered;
			if (volt_filtered > BMS_VOLT_OVER || volt_filtered < BMS_VOLT_UNDER) {
				bad_volt = true;
				bad_i = i;
				bad_j = j;

				if (volt_filtered > BMS_VOLT_OVER) {
					AMS_hbState.flags.BMS_OVER_VOLT = 1;
					//printf("Over Volt\r\n");
				}
				else if (volt_filtered < BMS_VOLT_UNDER) {
					AMS_hbState.flags.BMS_UNDER_VOLT = 1;
					//printf("Under Volt\r\n");
				}
			}
		}

		for (int j = 0; j < BMS_TEMP_COUNT; j++) {
			if (((i == (6 - 1)) && (j >= 7)) || ((i == 7) && ((j == 4) || (j == 5) || (j == 8)))) {
				continue;
			}
			uint8_t temp_filtered = bms.temperature_filters[i][j].current_filtered;
			if (temp_filtered > BMS_TEMP_OVER) {
				bad_temp = true;
				bad_i = i;
				bad_j = j;
				AMS_hbState.flags.BMS_BAD_TEMP = 1;
				//printf("Bad Temp\r\n");
			}

			uint32_t last_temp_time = bms.temperature_times[i][j];
			if (HAL_GetTick() - last_temp_time > TEMP_CUTOFF_TIMEOUT) {
				AMS_hbState.flags.BMS_BAD_TEMP = 1;

				bad_i = i;
				bad_j = j;

				printf("bad temp %i %i: %i\r\n", bad_i, bad_j, bms.temperature_filters[bad_i][bad_j].current_filtered);
				if (AMS_hbState.stateID != AMS_STATE_SHUTDOWN && AMS_hbState.stateID != AMS_STATE_TRIG_SHUTDOWN) {
					fsm_changeState(&fsm, &state_trig_shutdown, "bad BMS temperature persistent");
					return;
				}
			}

		}
	}

	if (bad_volt) {
		printf("bad volt %i %i: %i\r\n", bad_i, bad_j, bms.voltage_filters[bad_i][bad_j].current_filtered);
		fsm_changeState(&fsm, &state_trig_shutdown, "bad voltage detected");
		return;
	}

	if (bad_temp) {
		printf("bad temp %i %i: %i\r\n", bad_i, bad_j, bms.temperature_filters[bad_i][bad_j].current_filtered);

		fsm_changeState(&fsm, &state_trig_shutdown, "bad temperature detected");
		return;
	}
}

void bms_handleVoltageMsg(CAN_MSG_Generic_t *msg) {
	uint8_t idx = (msg->ID & 0xF);
	uint8_t msgId;
	uint16_t voltages[BMS_VOLT_PACK_COUNT];

	Parse_BMS_TransmitVoltage(msg->data, &msgId, voltages);

	for (int i = 0; ((i < BMS_VOLT_PACK_COUNT) && (((msgId * BMS_VOLT_PACK_COUNT) + i) < BMS_VOLT_COUNT)); i++) {
		bms.voltages[idx - 1][(msgId * BMS_VOLT_PACK_COUNT) + i] = voltages[i];
		bmu_window_filter_update(&bms.voltage_filters[idx - 1][(msgId * BMS_VOLT_PACK_COUNT) + i],
				bms.voltages[idx - 1][(msgId * BMS_VOLT_PACK_COUNT) + i]);
	}
}

void bms_handleTemperatureMsg(CAN_MSG_Generic_t *msg) {
	uint8_t idx = (msg->ID & 0xF);
	uint8_t msgId;
	uint8_t temperatures[BMS_TEMP_PACK_COUNT];

	Parse_BMS_TransmitTemperature(msg->data, &msgId, temperatures);

	for (int i = 0; ((i < BMS_TEMP_PACK_COUNT) && (((msgId * BMS_TEMP_PACK_COUNT) + i) < BMS_TEMP_COUNT)); i++) {
		bms.temperatures[idx - 1][(msgId * BMS_TEMP_PACK_COUNT) + i] = temperatures[i];

		if (temperatures[i] < BMS_TEMP_BAD_CUTOFF) {
			bms.temperature_times[idx - 1][(msgId * BMS_TEMP_PACK_COUNT) + i] = HAL_GetTick();
			bmu_window_filter_update(&bms.temperature_filters[idx - 1][(msgId * BMS_TEMP_PACK_COUNT) + i],
					bms.temperatures[idx - 1][(msgId * BMS_TEMP_PACK_COUNT) + i]);
		}
		else {
			/*
			 printf("invalid temp: %i %i: %i\r\n", idx,
			 (msgId * BMS_TEMP_PACK_COUNT) + i, temperatures[i]);
			 */
		}
	}
}
