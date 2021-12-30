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
#include <CAN_CC.h>
#include <Timer.h>

#include "main.h"

typedef struct heartbeat_states {
	uint32_t heartbeat_timeout;

	bool BMS[BMS_COUNT];
	bool CC;

	uint32_t hb_BMS_start[BMS_COUNT];
	uint32_t hb_CC_start;
} heartbeat_states_t;

extern heartbeat_states_t heartbeats;
extern ms_timer_t timer_heartbeat;

extern AMS_HeartbeatState_t AMS_heartbeatState;
extern CC_HeartbeatState_t CC_heartbeatState;

void setup_heartbeat();
void heartbeat_timer_cb(void *args);

void heartbeat_timeout_reset();


#endif /* INC_HEARTBEAT_H_ */
