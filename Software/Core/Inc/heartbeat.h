/*
 * heartbeat.h
 *
 *  Created on: Dec 30, 2021
 *      Author: Calvin
 */

#ifndef INC_HEARTBEAT_H_
#define INC_HEARTBEAT_H_

#include <stdbool.h>
#include <CAN_AMS.h>
#include <CAN_BMS.h>
#include <CAN_ChrgCtrl.h>
#include <CAN_VCU.h>
#include <Timer.h>

#include "main.h"

typedef struct heartbeat_states {
	uint32_t heartbeat_timeout;

	bool BMS[BMS_COUNT];
	bool VCU_CTRL;
	bool SENDYNE1;
	bool SENDYNE2;
	bool CHRG_CTRL;

	uint32_t hb_BMS_start[BMS_COUNT];
	uint32_t hb_VCU_CTRL_start;
	uint32_t hb_SENDYNE1_start;
	uint32_t hb_SENDYNE2_start;
	uint32_t hb_CHRG_CTRL_start;

} heartbeat_states_t;

extern heartbeat_states_t heartbeats;
extern ms_timer_t timer_heartbeat;

extern CHRGCTRL_HeartbeatState_t CHRG_CTRL_hbState;
extern AMS_HeartbeatState_t AMS_hbState;
extern VCU_HeartbeatState_t VCU_CTRL_hbState;

void setup_heartbeat();
void heartbeat_timer_cb(void *args);

void heartbeat_timeout_reset();

// call every time checking CAN message queue to update heartbeat status of boards
bool check_heartbeat_msg(CAN_MSG_Generic_t *msg);

// call to update status of heartbeat timeouts and detect potential board loss
bool check_CAN2_heartbeat();
bool check_CHRG_heartbeat();
bool check_bms_heartbeat();
bool check_sendyne_heartbeat();


#endif /* INC_HEARTBEAT_H_ */
