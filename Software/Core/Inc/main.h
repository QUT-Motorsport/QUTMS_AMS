/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define PRECHARGE_DELAY 300 // Milliseconds
#define SEM_ACQUIRE_TIMEOUT 32 //Milliseconds
#define AMS_HEARTBEAT_PERIOD 75 //Milliseconds
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define MStoTICKS(MILLISECONDS) (MILLISECONDS * osKernelGetTickFreq()/1000.0)
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void ASM_LogErr(char* TAG, char* subsystem, char* error);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
__NO_RETURN void fsm_thread_mainLoop(void* arg);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HVB_P_Pin GPIO_PIN_1
#define HVB_P_GPIO_Port GPIOC
#define HVA_P_Pin GPIO_PIN_1
#define HVA_P_GPIO_Port GPIOA
#define HVB_N_Pin GPIO_PIN_2
#define HVB_N_GPIO_Port GPIOA
#define HVA_N_Pin GPIO_PIN_6
#define HVA_N_GPIO_Port GPIOA
#define FAN_Pin GPIO_PIN_7
#define FAN_GPIO_Port GPIOA
#define PRECHG_Pin GPIO_PIN_1
#define PRECHG_GPIO_Port GPIOB
#define ALARM_CTRL_Pin GPIO_PIN_2
#define ALARM_CTRL_GPIO_Port GPIOB
#define BMS_CTRL_Pin GPIO_PIN_6
#define BMS_CTRL_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_7
#define BUZZER_GPIO_Port GPIOB
#define LED0_Pin GPIO_PIN_8
#define LED0_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
