/*
 * can_dict.c
 *
 *  Created on: 1 Jan. 2022
 *      Author: Calvin
 */

#include "can_dict.h"
#include "can.h"

#include <CAN_AMS.h>

obj_dict_t AMS_obj_dict;
ms_timer_t timer_OD;

void AMS_OD_init() {
	OD_init(&AMS_obj_dict);

	// set values
	// OD_setValue(&CC_obj_dict, CC_OD_IDX_DEADZONE, 0);

	// object dictionary messages are fairly quiet, so only need to check every 50ms
	timer_OD = timer_init(50, true, OD_timer_cb);

	// start timer
	timer_start(&timer_OD);
}

void AMS_OD_handleCAN(CAN_MSG_Generic_t *msg) {
	uint8_t outputData[8];

	// interprets CAN message as either get value
	bool sendMsg = OD_handleCAN(&AMS_obj_dict, msg->data, outputData);

	if (sendMsg) {
		CAN_TxHeaderTypeDef header = { .IDE = CAN_ID_EXT, .RTR = CAN_RTR_DATA, .TransmitGlobalTime = DISABLE };

		AMS_OBJ_DICT_t new_msg = Compose_AMS_OBJ_DICT(outputData);
		header.ExtId = new_msg.id;
		header.DLC = sizeof(new_msg.data);


		AMS_send_can_msg((CAN_HandleTypeDef *)msg->hcan, &header, new_msg.data);
	}
}

void OD_timer_cb(void *args) {
	CAN_MSG_Generic_t msg;

	while (queue_next(&queue_CAN_OD, &msg)) {
		AMS_OD_handleCAN(&msg);
	}
}
