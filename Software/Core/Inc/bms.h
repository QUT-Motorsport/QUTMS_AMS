/*
 * bms.h
 *
 *  Created on: 1 Jan. 2022
 *      Author: Calvin
 */

#ifndef INC_BMS_H_
#define INC_BMS_H_

#include <Timer.h>

extern ms_timer_t bms_timer;

void bms_ctrl_off();
void bms_ctrl_on();


#endif /* INC_BMS_H_ */
