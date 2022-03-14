/*
 * vesc_inc.h
 *
 *  Created on: Mar 10, 2022
 *      Author: Calvin
 */

#ifndef INC_VESC_INC_H_
#define INC_VESC_INC_H_

#include <CAN_VESC.h>
#include <QUTMS_CAN.h>
#include <stdbool.h>

typedef struct vesc_data {
	float voltage[NUM_VESC];
} vesc_data_t;

extern vesc_data_t vesc_inv;

bool check_vesc_msg(CAN_MSG_Generic_t *msg);

#endif /* INC_VESC_INC_H_ */
