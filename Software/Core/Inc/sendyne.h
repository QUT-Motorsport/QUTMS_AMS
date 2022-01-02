/*
 * sendyne.h
 *
 *  Created on: Jan 2, 2022
 *      Author: Calvin
 */

#ifndef INC_SENDYNE_H_
#define INC_SENDYNE_H_

#include <Timer.h>
#include <QUTMS_CAN.h>

typedef struct sendyne_state {
	int32_t voltage_uV;
	float voltage;

	int32_t HVACurrent_uA;
	float HVACurrent;

	int32_t HVBCurrent_uA;
	float HVBCurrent;

} sendyne_state_t;

extern ms_timer_t timer_sendyne;
extern sendyne_state_t sendyne;

void sendyne_handleCurrent(CAN_MSG_Generic_t *msg);
void sendyne_handleVoltage(CAN_MSG_Generic_t *msg);

void setup_sendyne();

void sendyne_timer_update();

void sendyne_CAN_timer_cb(void *args);
void sendyne_voltage_timer_cb(void *args);
void sendyne_current_timer_cb(void *args);


#endif /* INC_SENDYNE_H_ */
