/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    can.c
 * @brief   This file provides code for the configuration
 *          of the CAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include <QUTMS_CAN.h>
#include <stdio.h>

message_queue_t queue_CAN2;
message_queue_t queue_CAN4;
message_queue_t queue_CAN_OD;

uint32_t txMailbox_CAN2;
uint32_t txMailbox_CAN4;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void) {

	/* USER CODE BEGIN CAN1_Init 0 */

	/* USER CODE END CAN1_Init 0 */

	/* USER CODE BEGIN CAN1_Init 1 */

	/* USER CODE END CAN1_Init 1 */
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 4;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = DISABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN1_Init 2 */

	/* USER CODE END CAN1_Init 2 */

}
/* CAN2 init function */
void MX_CAN2_Init(void) {

	/* USER CODE BEGIN CAN2_Init 0 */

	/* USER CODE END CAN2_Init 0 */

	/* USER CODE BEGIN CAN2_Init 1 */

	/* USER CODE END CAN2_Init 1 */
	hcan2.Instance = CAN2;
	hcan2.Init.Prescaler = 4;
	hcan2.Init.Mode = CAN_MODE_NORMAL;
	hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan2.Init.TimeSeg1 = CAN_BS1_15TQ;
	hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan2.Init.TimeTriggeredMode = DISABLE;
	hcan2.Init.AutoBusOff = DISABLE;
	hcan2.Init.AutoWakeUp = DISABLE;
	hcan2.Init.AutoRetransmission = DISABLE;
	hcan2.Init.ReceiveFifoLocked = DISABLE;
	hcan2.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN2_Init 2 */

	/* USER CODE END CAN2_Init 2 */

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED = 0;

void HAL_CAN_MspInit(CAN_HandleTypeDef *canHandle) {

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (canHandle->Instance == CAN1) {
		/* USER CODE BEGIN CAN1_MspInit 0 */

		/* USER CODE END CAN1_MspInit 0 */
		/* CAN1 clock enable */
		HAL_RCC_CAN1_CLK_ENABLED++;
		if (HAL_RCC_CAN1_CLK_ENABLED == 1) {
			__HAL_RCC_CAN1_CLK_ENABLE();
		}

		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**CAN1 GPIO Configuration
		 PA11     ------> CAN1_RX
		 PA12     ------> CAN1_TX
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_11;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_12;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* CAN1 interrupt Init */
		HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
		/* USER CODE BEGIN CAN1_MspInit 1 */

		/* USER CODE END CAN1_MspInit 1 */
	} else if (canHandle->Instance == CAN2) {
		/* USER CODE BEGIN CAN2_MspInit 0 */

		/* USER CODE END CAN2_MspInit 0 */
		/* CAN2 clock enable */
		__HAL_RCC_CAN2_CLK_ENABLE();
		HAL_RCC_CAN1_CLK_ENABLED++;
		if (HAL_RCC_CAN1_CLK_ENABLED == 1) {
			__HAL_RCC_CAN1_CLK_ENABLE();
		}

		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**CAN2 GPIO Configuration
		 PB12     ------> CAN2_RX
		 PB13     ------> CAN2_TX
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_12;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_13;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* CAN2 interrupt Init */
		HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
		/* USER CODE BEGIN CAN2_MspInit 1 */

		/* USER CODE END CAN2_MspInit 1 */
	}
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef *canHandle) {

	if (canHandle->Instance == CAN1) {
		/* USER CODE BEGIN CAN1_MspDeInit 0 */

		/* USER CODE END CAN1_MspDeInit 0 */
		/* Peripheral clock disable */
		HAL_RCC_CAN1_CLK_ENABLED--;
		if (HAL_RCC_CAN1_CLK_ENABLED == 0) {
			__HAL_RCC_CAN1_CLK_DISABLE();
		}

		/**CAN1 GPIO Configuration
		 PA11     ------> CAN1_RX
		 PA12     ------> CAN1_TX
		 */
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);

		/* CAN1 interrupt Deinit */
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
		/* USER CODE BEGIN CAN1_MspDeInit 1 */

		/* USER CODE END CAN1_MspDeInit 1 */
	} else if (canHandle->Instance == CAN2) {
		/* USER CODE BEGIN CAN2_MspDeInit 0 */

		/* USER CODE END CAN2_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_CAN2_CLK_DISABLE();
		HAL_RCC_CAN1_CLK_ENABLED--;
		if (HAL_RCC_CAN1_CLK_ENABLED == 0) {
			__HAL_RCC_CAN1_CLK_DISABLE();
		}

		/**CAN2 GPIO Configuration
		 PB12     ------> CAN2_RX
		 PB13     ------> CAN2_TX
		 */
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12 | GPIO_PIN_13);

		/* CAN2 interrupt Deinit */
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX1_IRQn);
		/* USER CODE BEGIN CAN2_MspDeInit 1 */

		/* USER CODE END CAN2_MspDeInit 1 */
	}
}

/* USER CODE BEGIN 1 */
bool setup_CAN() {
	// setup CAN queues
	queue_init(&queue_CAN2, sizeof(CAN_MSG_Generic_t));
	queue_init(&queue_CAN4, sizeof(CAN_MSG_Generic_t));
	queue_init(&queue_CAN_OD, sizeof(CAN_MSG_Generic_t));

	return true;
}

bool init_CAN2() {
	if (HAL_CAN_Start(&CANBUS2) != HAL_OK) {
		printf("ERROR: FAILED TO START CANBUS2\r\n");
		return false;
	}

	CAN_FilterTypeDef sFilterConfig;

	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0001;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&CANBUS2, &sFilterConfig) != HAL_OK) {
		printf("failed to config filter on CANBUS2\r\n");
		return false;
	}

	if (HAL_CAN_ActivateNotification(&CANBUS2, CAN_IT_RX_FIFO0_MSG_PENDING)
			!= HAL_OK) {
		printf("Failed to activate CANBUS2 notification on RX0");
		return false;
	}

	if (HAL_CAN_ActivateNotification(&CANBUS2, CAN_IT_RX_FIFO1_MSG_PENDING)
			!= HAL_OK) {
		printf("Failed to activate CANBUS2 notification on RX1");
		return false;
	}

	return true;
}

bool init_CAN4() {
	MX_CAN1_Init();

	if (HAL_CAN_Start(&CANBUS4) != HAL_OK) {
		printf("ERROR: FAILED TO START CANBUS4\r\n");
		return false;
	}

	CAN_FilterTypeDef sFilterConfig;

	sFilterConfig.FilterBank = 14;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0001;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&CANBUS4, &sFilterConfig) != HAL_OK) {
		printf("failed to config filter on CANBUS2\r\n");
		return false;
	}

	if (HAL_CAN_ActivateNotification(&CANBUS4, CAN_IT_RX_FIFO0_MSG_PENDING)
			!= HAL_OK) {
		printf("Failed to activate CANBUS4 notification on RX0");
		return false;
	}

	if (HAL_CAN_ActivateNotification(&CANBUS4, CAN_IT_RX_FIFO1_MSG_PENDING)
			!= HAL_OK) {
		printf("Failed to activate CANBUS4 notification on RX1");
		return false;
	}

	return true;
}

HAL_StatusTypeDef AMS_send_can_msg(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[]) {
	int can_idx = 0;

	uint32_t *pTxMailbox;
	if (hcan == &CANBUS2) {
		pTxMailbox = &txMailbox_CAN2;
		can_idx = 2;
	}
	else if (hcan == &CANBUS4) {
		pTxMailbox = &txMailbox_CAN4;
		can_idx = 4;
	}

	// finally send CAN msg
	HAL_StatusTypeDef result = HAL_CAN_AddTxMessage(hcan, pHeader, aData, pTxMailbox);
	if (result != HAL_OK) {
		printf("FAILED TO SEND CANBUS%i - e: %lu\r\n", can_idx + 1, hcan->ErrorCode);
	}

	return result;
}

void handle_CAN_interrupt(CAN_HandleTypeDef *hcan, int fifo) {
	__disable_irq();
	//int fill = HAL_CAN_GetRxFifoFillLevel(hcan, fifo);

	CAN_MSG_Generic_t msg;
	CAN_RxHeaderTypeDef header;

	while (HAL_CAN_GetRxFifoFillLevel(hcan, fifo) > 0) {
		//for (int i = 0; i < fill; i++) {
		if (HAL_CAN_GetRxMessage(hcan, fifo, &header, msg.data) != HAL_OK) {
			printf("failed to read CAN msg");
		}

		msg.hcan = hcan;
		msg.ID = header.IDE == CAN_ID_EXT ? header.ExtId : header.StdId;
		msg.ID_TYPE = header.IDE == CAN_ID_EXT ? 1 : 0;
		msg.DLC = header.DLC;
		msg.timestamp = HAL_GetTick();

		if (msg.ID == CC_OBJ_DICT_ID) {
			// object dictionary messages get sent to their own queue for processing
			queue_add(&queue_CAN_OD, &msg);
		}
		else {
			// add to CAN recieve queue
			if (hcan == &CANBUS2) {
				queue_add(&queue_CAN2, &msg);
			}
			else if (hcan == &CANBUS4) {
				queue_add(&queue_CAN4, &msg);
			}
		}
	}
	__enable_irq();
}




/* USER CODE END 1 */
