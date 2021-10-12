/*
 * AMS_FSM_SoC.c
 *
 *  Created on: 6 Jul 2021
 *      Author: Thomas Fraser
 */

#include <AMS_FSM_States.h>

state_t SoCState = { &state_SoC_enter, &state_SoC_iterate, &state_SoC_exit,
		"SoC_s" };

void state_SoC_enter(fsm_t *fsm) {
	debugCAN_enterState(AMS_STATE_ID_SoC);

	// start asking BMS to send stuff
	AMS_GlobalState->heartbeatTimerAMS = timer_init(AMS_HEARTBEATBMS_PERIOD,
			true, &heartbeatTimerBMS_cb);
	timer_start(&AMS_GlobalState->heartbeatTimerAMS);

	/** We need 1 voltage packet from each BMS */
	AMS_GlobalState->bmsWakeupTimer = timer_init(BMS_WAKEUP_TIMEOUT, false,
			&wakeupTimerBMS_cb);
	timer_start(&AMS_GlobalState->bmsWakeupTimer);
}

void state_SoC_iterate(fsm_t *fsm) {
	while (!queue_empty(&AMS_GlobalState->CANQueue)) {
		AMS_CAN_Generic_t msg;
		if (queue_next(&AMS_GlobalState->CANQueue, &msg)) {
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
			bms |= (1 << i);
		}
	}

	if (bms_count == BMS_COUNT) {
		if (charge) {
			fsm_changeState(fsm, &chargingState, "Charging, all BMSs awake");
		} else {
			fsm_changeState(fsm, &idleState, "All BMSs awake, moving to idle");
		}
	} else {
		if (bms_count > 0) {
			printf("bms: %d %x\r\n", bms_count, bms);
		}
	}
}

void state_SoC_exit(fsm_t *fsm) {
	debugCAN_exitState(AMS_STATE_ID_SoC);

	/** Now we have all BMSs, disable boot pin */
	HAL_GPIO_WritePin(BMS_CTRL_GPIO_Port, BMS_CTRL_Pin, GPIO_PIN_RESET);

	/** Stop and delete the BMS Wakeup Timer */
	timer_stop(&AMS_GlobalState->bmsWakeupTimer);
	return;
}
