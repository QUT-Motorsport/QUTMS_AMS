/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <Timer.h>
#include <queue.h>

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN Private defines */
#define CAN_QUEUE_SIZE 50

extern message_queue_t queue_CAN2;
extern message_queue_t queue_CAN4;
extern message_queue_t queue_CAN_OD;

extern uint32_t txMailbox_CAN2;
extern uint32_t txMailbox_CAN4;

/**
 * For when you forget
 * @brief CAN2 = Chassis = hcan2, CAN4 = BMS = hcan1
 */
#define CANBUS4 hcan1
#define CANBUS2 hcan2
/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);

/* USER CODE BEGIN Prototypes */

bool setup_CAN();
bool init_CAN2();
bool init_CAN4();

HAL_StatusTypeDef AMS_send_can_msg(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[]);

// called from CAN interrupts, just adds any messages to the CAN queues
void handle_CAN_interrupt(CAN_HandleTypeDef *hcan, int fifo);



/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

