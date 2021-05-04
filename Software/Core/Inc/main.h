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
#include <stdbool.h>
#include "fsm.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
bool charge;
// Printf to UART
#define PRINTF_TO_UART

// Periods
#define PRECHARGE_DELAY 500U // Milliseconds
#define SEM_ACQUIRE_TIMEOUT 32U // Milliseconds
#define AMS_HEARTBEATBMS_PERIOD 1000U
#define AMS_HEARTBEAT_PERIOD 75U // Milliseconds
#define AMS_IDC_PERIOD 250U // Milliseconds
#define AMS_CS_PERIOD 1000U // Milliseconds
#define AMS_CAN_QUEUESIZE 25 //Units
#define BMS_WAKEUP_TIMEOUT 10000U //Milliseconds

// BMS
#define BMS_COUNT 6
#define BMS_VOLTAGE_COUNT 10 /**< Voltages Read Per BMS */
#define BMS_TEMPERATURE_COUNT 12 /**< Temperatures Read Per BMS */

//Voltage
//#define ACCUMULATOR_VOLTAGE 36.0f * BMS_COUNT
#define ACCUMULATOR_VOLTAGE 59.0f

// Debug
#define DEBUG_CB
#define ENABLE_CS
#define DEBUG_PERIOD 1000U // Milliseconds
#define PRECHARGE_VDIFF 5.0f
#define BMS_CELL_VMIN 2.0f

// Bit Masks
#define BMS_ID_MASK 0x1FFFFFF0

// Current Sensor CAN Information
//#define CS_2_EXTID 0xA100201
//#define CS_1_EXTID 0xA100211
//#define CS_2_RESPONSE_EXTID 0xA100200
//#define CS_1_RESPONSE_EXTID 0xA100210

// OG, before Sam Broke the wire
#define CS_1_EXTID 0xA100201
#define CS_2_EXTID 0xA100211
#define CS_1_RESPONSE_EXTID 0xA100200
#define CS_2_RESPONSE_EXTID 0xA100210

#define CURRENT_SENSOR_REQ_SIZE 1
#define CURRENT_SENSOR_CC_LOW 0x40
#define CURRENT_SENSOR_CC_HIGH 0x41
#define CURRENT_SENSOR_CURRENT 0x20



#define CURRENT_SENSOR_MAX_AS CURRENT_SENSOR_MAX_AH * 3600
#define CURRENT_SENSOR_MAX_AH 54.495

#define CS_V1 0x60
#define CS_V2 0x61
#define CS_V3 0x62

// AMS_LogInfo
#define AMS_LOGINFO_ENABLED

// LOGGING DEFS
#define CS_LOG_CC 0
//#define CAN2_LOG_ON_MSG
//#define CAN4_LOG_ON_MSG
#define BMS_LOG_V 0
#define BMS_LOG_T 0

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
void Sendyne_requestVoltage(int index);
void heartbeatTimer_cb(void *fsm);
void heartbeatTimerBMS_cb(void *fsm);
void wakeupTimerBMS_cb(void *fsm);
void ccTimer_cb(void *fsm);
void cTimer_cb(void *fsm);
void debugTimer_cb(void *fsm);
void AMS_LogErr(char* error, size_t length);
void AMS_LogToSD(char* msg, size_t length);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
float getRuntime();
int _write(int file, char *data, int len);
void handleCAN(CAN_HandleTypeDef *hcan, int fifo);
__NO_RETURN void fsm_mainLoop(void* fsm);
__NO_RETURN void uart_thread_mainLoop(void* fsm);

fsm_t fsm;

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
