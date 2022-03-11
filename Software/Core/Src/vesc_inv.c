/*
 * vesc_inv.c
 *
 *  Created on: Mar 10, 2022
 *      Author: Calvin
 */

#include "vesc_inc.h"

vesc_data_t vesc_inv;

bool check_vesc_msg(CAN_MSG_Generic_t *msg) {
	uint8_t vesc_ID = (msg->ID & 0xFF);
	uint8_t vesc_type_ID = msg->ID >> 8;

	if ((vesc_type_ID == VESC_CAN_PACKET_STATUS_5) && (vesc_ID < NUM_VESC) ) {
		int32_t tachoRPM;
		float inputVoltage;
		float reserved;
		Parse_VESC_CANPacketStatus5(msg->data, &tachoRPM, &inputVoltage, &reserved);

		vesc_inv.voltage[vesc_ID] = inputVoltage;

		return true;
	}

	return false;
}
