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
// Periods
#define PRECHARGE_DELAY 500U // Milliseconds
#define SEM_ACQUIRE_TIMEOUT 32U // Milliseconds
#define SEM_ACQUIRE_GLOBALSTATE_TIMEOUT 64U // Milliseconds, might need a longer timeout for global states.
#define AMS_HEARTBEAT_PERIOD 75U // Milliseconds
#define AMS_IDC_PERIOD 250U // Milliseconds
#define AMS_CS_PERIOD 1000U // Milliseconds
#define AMS_CAN_QUEUESIZE 25

// BMS
#define BMS_COUNT 12
#define BMS_VOLTAGE_COUNT 10
#define BMS_TEMPERATURE_COUNT 12

// Debug
//#define DEBUG_CB
#define ENABLE_CS
#define DEBUG_PERIOD 2500U // Milliseconds

// Bit Masks
#define BMS_ID_MASK 0x1FFFFFF0

// Current Sensor CAN Information
#define CURRENT_SENSOR_CAN_EXTID 0xA100201
#define CURRENT_SENSOR_CAN_RESPONSE_EXTID 0xA100200
#define CURRENT_SENSOR_REQ_SIZE 1
#define CURRENT_SENSOR_CC_LOW 0x41
#define CURRENT_SENSOR_CC_HIGH 0x42

#define CURRENT_SENSOR_MAX_AS CURRENT_SENSOR_MAX_AH * 3600
#define CURRENT_SENSOR_MAX_AH 54.495

// LOGGING DEFS
#define CS_LOG_CC 0
//#define CAN2_LOG_ON_MSG
//#define CAN4_LOG_ON_MSG
#define BMS_LOG_IDLE_V 0
#define BMS_LOG_IDLE_T 1

// General
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/**
 * @brief Callback for the IDC ALARM timer, which will be called every 250 milliseconds.
 * @param fsm A pointer to the FSM object
 */
void IDC_Alarm_cb(void* fsm);
/**
 * @brief Callback for the heartbeat timer, which will be called every 75 milliseconds to send a heartbeat.
 * @param fsm A pointer to the FSM object
 */
void heartbeatTimer_cb(void *fsm);
void osTimer_cb(void *fsm);
void debugTimer_cb(void *fsm);
void AMS_LogInfo(char* msg, size_t length);
void AMS_LogErr(char* error, size_t length);
void AMS_LogToSD(char* msg, size_t length);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
uint32_t getRuntime();
__NO_RETURN void fsm_thread_mainLoop(void* fsm);
__NO_RETURN void uart_thread_mainLoop(void* fsm);

/** Temp Function */
float vToSoC(float voltage);
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
#define SD_DI_Pin GPIO_PIN_9
#define SD_DI_GPIO_Port GPIOC
#define SD_CLK_Pin GPIO_PIN_10
#define SD_CLK_GPIO_Port GPIOC
#define SD_DO_Pin GPIO_PIN_11
#define SD_DO_GPIO_Port GPIOC
#define SD_CS_Pin GPIO_PIN_12
#define SD_CS_GPIO_Port GPIOC
#define IDC_ALARM_Pin GPIO_PIN_5
#define IDC_ALARM_GPIO_Port GPIOB
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
