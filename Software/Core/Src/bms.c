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
			AMS_heartbeatState.bmsStatus |= 1 << (idx-1);

			if (masked_id == BMS_TransmitVoltage_ID) {
				bms_handleVoltageMsg(&msg);
			} else if (masked_id == BMS_TransmitTemperature_ID) {
				bms_handleTemperatureMsg(&msg);
			} else if (masked_id == BMS_BadCellVoltage_ID) {

			} else if (masked_id == BMS_BadCellTemperature_ID) {

			}
		}

	}
}

void bms_handleVoltageMsg(CAN_MSG_Generic_t *msg) {
	uint8_t idx = (msg->ID & 0xF);
	uint8_t msgId;
	uint16_t voltages[BMS_VOLT_PACK_COUNT];

	Parse_BMS_TransmitVoltage(msg->data, &msgId, voltages);

	for (int i = 0; ((i < BMS_VOLT_PACK_COUNT) && ( ((msgId * BMS_VOLT_PACK_COUNT) + i) < BMS_VOLT_COUNT)); i++) {
		bms.voltages[idx-1][(msgId * BMS_VOLT_PACK_COUNT) + i] = voltages[i];
	}
}

void bms_handleTemperatureMsg(CAN_MSG_Generic_t *msg) {
	uint8_t idx = (msg->ID & 0xF);
	uint8_t msgId;
	uint8_t temperatures[BMS_TEMP_PACK_COUNT];

	Parse_BMS_TransmitTemperature(msg->data, &msgId, temperatures);

	for (int i = 0; i < BMS_TEMP_PACK_COUNT; i++) {
		bms.temperatures[idx-1][(msgId * BMS_TEMP_PACK_COUNT) + i] = temperatures[i];
	}
}
