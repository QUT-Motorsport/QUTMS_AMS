/*
 * shutdown.h
 *
 *  Created on: 12 Jan. 2022
 *      Author: Calvin J
 */

#ifndef INC_SHUTDOWN_H_
#define INC_SHUTDOWN_H_

#include <QUTMS_CAN.h>
#include <stdbool.h>

bool check_shutdown_msg(CAN_MSG_Generic_t *msg, bool *shdn_status);

extern bool shutdown_status;


#endif /* INC_SHUTDOWN_H_ */
