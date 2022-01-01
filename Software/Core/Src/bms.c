/*
 * bms.c
 *
 *  Created on: 1 Jan. 2022
 *      Author: Calvin
 */

#include "bms.h"
#include "main.h"

ms_timer_t bms_timer;

void bms_ctrl_off() {
	HAL_GPIO_WritePin(BMS_CTRL_GPIO_Port, BMS_CTRL_Pin, GPIO_PIN_RESET);
}

void bms_ctrl_on() {
	HAL_GPIO_WritePin(BMS_CTRL_GPIO_Port, BMS_CTRL_Pin, GPIO_PIN_SET);
}
