/*
 * bms.h
 *
 *  Created on: 1 Jan. 2022
 *      Author: Calvin
 */

#ifndef INC_BMS_H_
#define INC_BMS_H_

#include <Timer.h>
#include <QUTMS_CAN.h>
#include <CAN_BMS.h>
#include "bmu_window_filter.h"

#define BMS_VOLT_OVER 3700
#define BMS_VOLT_UNDER 2000

#define BMS_TEMP_OVER 50
#define BMS_TEMP_BAD_CUTOFF 100

#define FILTER_SIZE_VOLT 8
#define FILTER_SIZE_TEMP 4

typedef struct bms_status {
	uint16_t voltages[BMS_COUNT][BMS_VOLT_COUNT];
	uint8_t temperatures[BMS_COUNT][BMS_TEMP_COUNT];
	bmu_window_filter_t voltage_filters[BMS_COUNT][BMS_VOLT_COUNT];
	bmu_window_filter_t temperature_filters[BMS_COUNT][BMS_TEMP_COUNT];
	uint8_t lastVoltMsgIdx[BMS_COUNT];
} bms_status_t;

extern ms_timer_t bms_timer;
extern bms_status_t bms;

void bms_ctrl_off();
void bms_ctrl_on();

void bms_setup();

void bms_CAN_timer_cb(void *args);

void bms_handleVoltageMsg(CAN_MSG_Generic_t *msg);
void bms_handleTemperatureMsg(CAN_MSG_Generic_t *msg);


#endif /* INC_BMS_H_ */
