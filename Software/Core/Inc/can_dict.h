/*
 * can_dict.h
 *
 *  Created on: 1 Jan. 2022
 *      Author: Calvin
 */

#ifndef INC_CAN_DICT_H_
#define INC_CAN_DICT_H_

#include <obj_dic.h>
#include <QUTMS_CAN.h>
#include <Timer.h>

#include "can.h"

extern obj_dict_t AMS_obj_dict;
extern ms_timer_t timer_OD;

void AMS_OD_init();
void AMS_OD_handleCAN(CAN_MSG_Generic_t *msg);
void OD_timer_cb(void *args);

#endif /* INC_CAN_DICT_H_ */
