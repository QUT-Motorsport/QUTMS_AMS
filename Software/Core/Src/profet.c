/*
 * profet.c
 *
 *  Created on: 1 Jan. 2022
 *      Author: Calvin
 */

#include "profet.h"
#include "main.h"
#include "heartbeat.h"

void profet_open_all() {
	profet_HVA_N_open();
	profet_HVB_N_open();

	profet_HVA_P_open();
	profet_HVB_P_open();

	profet_precharge_open();
}

void profet_precharge() {
	profet_HVA_N_close();
	profet_HVB_N_close();

	profet_HVA_P_open();
	profet_HVB_P_open();

	profet_precharge_close();
}

void profet_TS_active() {
	profet_HVA_N_close();
	profet_HVB_N_close();

	profet_HVA_P_close();
	profet_HVB_P_close();

	profet_precharge_open();
}

void profet_HVA_N_open() {
	HAL_GPIO_WritePin(HVA_N_GPIO_Port, HVA_N_Pin, GPIO_PIN_RESET);
	AMS_hbState.flags.C_HVAn = 0;
}

void profet_HVA_N_close() {
	HAL_GPIO_WritePin(HVA_N_GPIO_Port, HVA_N_Pin, GPIO_PIN_SET);
	AMS_hbState.flags.C_HVAn = 1;
}

void profet_HVB_N_open() {
	HAL_GPIO_WritePin(HVB_N_GPIO_Port, HVB_N_Pin, GPIO_PIN_RESET);
	AMS_hbState.flags.C_HVBn = 0;
}

void profet_HVB_N_close() {
	HAL_GPIO_WritePin(HVB_N_GPIO_Port, HVB_N_Pin, GPIO_PIN_SET);
	AMS_hbState.flags.C_HVBn = 1;
}

void profet_HVA_P_open() {
	HAL_GPIO_WritePin(HVA_P_GPIO_Port, HVA_P_Pin, GPIO_PIN_RESET);
	AMS_hbState.flags.C_HVAp = 0;
}

void profet_HVA_P_close() {
	HAL_GPIO_WritePin(HVA_P_GPIO_Port, HVA_P_Pin, GPIO_PIN_SET);
	AMS_hbState.flags.C_HVAp = 1;
}

void profet_HVB_P_open() {
	HAL_GPIO_WritePin(HVB_P_GPIO_Port, HVB_P_Pin, GPIO_PIN_RESET);
	AMS_hbState.flags.C_HVBp = 0;
}

void profet_HVB_P_close() {
	HAL_GPIO_WritePin(HVB_P_GPIO_Port, HVB_P_Pin, GPIO_PIN_SET);
	AMS_hbState.flags.C_HVBp = 1;
}

void profet_precharge_open() {
	HAL_GPIO_WritePin(PRECHG_GPIO_Port, PRECHG_Pin, GPIO_PIN_RESET);
	AMS_hbState.flags.C_Precharge = 0;
}

void profet_precharge_close() {
	HAL_GPIO_WritePin(PRECHG_GPIO_Port, PRECHG_Pin, GPIO_PIN_SET);
	AMS_hbState.flags.C_Precharge = 1;
}
