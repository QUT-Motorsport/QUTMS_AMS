/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 * @author         : Thomas Fraser
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "AMS_CAN_Messages.h"
#include "BMS_CAN_Messages.h"
#include "FSM.h"
#include "AMS_FSM_States.h"
#include "BitBangedSD.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
osThreadId_t fsmThread;
const osThreadAttr_t fsmThreadAttr = {
		.stack_size = 2048
};
bbspi_t *SPI;
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
	GPIO_TypeDef ports[4] = {*SD_CLK_GPIO_Port, *SD_DO_GPIO_Port, *SD_DI_GPIO_Port, *SD_CS_GPIO_Port};
	uint16_t pins[4] = {SD_CLK_Pin, SD_DO_Pin, SD_DI_Pin, SD_CS_Pin};
	SPI = new_bbspi(ports, pins);
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_CAN1_Init();
	MX_USART3_UART_Init();
	MX_TIM4_Init();
	MX_CAN2_Init();
	/* USER CODE BEGIN 2 */
	// Activate CAN Interrupt
	char *msg = "------------------------------------\r\n";
	AMS_LogInfo(msg, strlen(msg));
	AMS_LogInfo(msg, strlen(msg));
	AMS_LogInfo("Setup Complete\r\n", strlen("Setup Complete\r\n"));

	HAL_Delay(500U);

	if (HAL_CAN_Start(&hcan1) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_CAN_Start(&hcan2) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		Error_Handler();
	}
	if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
	{
		Error_Handler();
	}
	if(HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		Error_Handler();
	}

	/** Create CAN Filter & Apply it to &hcan1, &hcan2 */
	CAN_FilterTypeDef  sFilterConfig;

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

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
	{
		/* Filter configuration Error */
		Error_Handler();
	}

	if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
	{
		/* Filter configuration Error */
		Error_Handler();
	}

	//Create FSM instance
	fsm_t *fsm = fsm_new(&idleState);

	// Create a new thread, where our FSM will run.
	osThreadNew(fsm_thread_mainLoop, fsm, &fsmThreadAttr);
	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
	MX_FREERTOS_Init();
	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure the Systick interrupt time
	 */
	__HAL_RCC_PLLI2S_ENABLE();
}

/* USER CODE BEGIN 4 */

void IDC_Alarm_cb(void* fsm)
{
	if(HAL_GPIO_ReadPin(IDC_ALARM_GPIO_Port, IDC_ALARM_Pin) == GPIO_PIN_RESET)
	{
		//TODO
		fsm_changeState(fsm, &errorState, "BMS Alarm Triggered");
	}
}

void heartbeatTimer_cb(void *fsm)
{
	//	// Take the GlobalState sem, find our values then fire off the packet
	if(osSemaphoreAcquire(AMS_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
	{
		// Get GPIO States
		bool HVAn_state = HAL_GPIO_ReadPin(HVA_N_GPIO_Port, HVA_N_Pin);
		bool HVBn_state = HAL_GPIO_ReadPin(HVB_N_GPIO_Port, HVB_N_Pin);
		bool precharge_state = HAL_GPIO_ReadPin(PRECHG_GPIO_Port, PRECHG_Pin);
		bool HVAp_state = HAL_GPIO_ReadPin(HVA_P_GPIO_Port, HVA_P_Pin);
		bool HVBp_state = HAL_GPIO_ReadPin(HVB_P_GPIO_Port, HVB_P_Pin);

		uint16_t averageVoltage = 0;
		for(int i = 0; i < BMS_COUNT; i++)
		{
			for(int j = 0; j < BMS_VOLTAGE_COUNT; j++)
			{
				averageVoltage += AMS_GlobalState->BMSVoltages[i][j]; // Our voltages are already stored here as a 12 bit integer.
			}
		}

		averageVoltage /= (BMS_COUNT * BMS_VOLTAGE_COUNT); /**< Good to send as already multiplied by 1000 */
		averageVoltage = 4090;

		uint16_t runtime = (HAL_GetTick() - AMS_GlobalState->startupTicks) / 1000;

		AMS_HeartbeatResponse_t canPacket = Compose_AMS_HeartbeatResponse(HVAn_state, HVBn_state, precharge_state, HVAp_state, HVBp_state, averageVoltage, runtime);
		CAN_TxHeaderTypeDef header =
		{
				.ExtId = canPacket.id,
				.IDE = CAN_ID_EXT,
				.RTR = CAN_RTR_DATA,
				.DLC = sizeof(canPacket.data),
				.TransmitGlobalTime = DISABLE,
		};

		HAL_CAN_AddTxMessage(&hcan2, &header, canPacket.data, &AMS_GlobalState->CAN2_TxMailbox);
		osSemaphoreRelease(AMS_GlobalState->sem);
	} else
	{
		Error_Handler();
	}
}

void osTimer_cb(void *fsm)
{
	CAN_TxHeaderTypeDef header =
	{
			.ExtId = 0xA100201,
			.IDE = CAN_ID_EXT,
			.RTR = CAN_RTR_DATA,
			.DLC = 1,
			.TransmitGlobalTime = DISABLE,
	};

	char *msg = "Logging Message 1\r\n";
	AMS_LogInfo(msg, strlen(msg));
	uint8_t data = 0x41;
	if(HAL_CAN_AddTxMessage(&hcan1, &header, &data, &AMS_GlobalState->CAN2_TxMailbox) != HAL_OK)
	{
		Error_Handler();
	}

	msg = "Logging Message 2\r\n";
	AMS_LogInfo(msg, strlen(msg));
	uint8_t data2 = 0x42;
	if(HAL_CAN_AddTxMessage(&hcan1, &header, &data2, &AMS_GlobalState->CAN2_TxMailbox) != HAL_OK)
	{
		Error_Handler();
	}

	free(msg);
}

/**
 * @brief CAN Callback function
 * @param hcan the can instance responsible for the callback
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	AMS_CAN_Generic_t *msg = calloc(1, sizeof(AMS_CAN_Generic_t));
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &(msg->header), msg->data);

	/**< Should we lock the GlobalState Semaphore? */
	char x[80];
	int len = sprintf(x, "Got Msg: [ID, Byte 0], [%02lX, %02X]\r\n", msg->header.ExtId, msg->data[0]);
	AMS_LogInfo(x, len);
	osMessageQueuePut(AMS_GlobalState->CANQueue, msg, 0U, 0U);
	AMS_LogInfo("Message Passed to Queue\r\n", strlen("Message Passed to Queue\r\n"));
}

/**
 * @brief FSM thread main loop task for RTOS
 * @param fsm the FSM object passed to the loop
 */
__NO_RETURN void fsm_thread_mainLoop(void *fsm)
{
	AMS_LogInfo("Entering FSM Thread\r\n", strlen("Entering FSM Thread\r\n"));
	// Reset our FSM in idleState, as we are just starting
	fsm_setLogFunction(fsm, &AMS_LogInfo);
	fsm_reset(fsm, &idleState);

	for(;;)
	{
		//TODO
		// T`s is our main loop now.
		fsm_iterate(fsm);
		if(fsm_getState_t(fsm) == &idleState)
		{
			fsm_changeState(fsm, &prechargeState, "Main loop changing precharge");
		}
	}
}

/**
 * @brief Log Info to UART
 * @param msg pointer to msg
 * @param length lenth of msg
 */
void AMS_LogInfo(char* msg, size_t length)
{
	HAL_UART_Transmit(&huart3, (uint8_t *)msg, length, HAL_MAX_DELAY);
}

void AMS_VerboseLog(char *msg)
{
#ifdef VERBOSE
	char *msgIter = msg;
	int length = 0;

	while(*msgIter != '\0')
	{
		length++;
		msgIter++;
	}

	AMS_LogInfo(msg, length);
#endif
}

void AMS_LogErr(char* error, size_t length)
{
	char* errorMsg = malloc(length + 9);
	int len = sprintf(errorMsg, "ERROR: %s\r\n", error);
	if(len == length + 9)
	{
		HAL_UART_Transmit(&huart3, (uint8_t *)errorMsg, len, HAL_MAX_DELAY);
	} else
	{
		Error_Handler();
	}
	free(errorMsg);
}

void AMS_LogToSD(char* msg, size_t length)
{
	for(int i = 0; i < length; i++)
	{
		// Transfer each byte of our message
		bbspi_transferByte(SPI, *(msg + i));
	}
}

float vToSoC(float voltage)
{
	if(voltage > 3.9 || voltage < 2.6)
	{
		char msg[80];
		int len = sprintf(msg, "Found voltage outside of expected range of [3.9 -> 2.6]\r\n");
		AMS_LogInfo(msg, len);
		return 0.0f;
	}
	return 0.55; // For now, return 55% SoC
}
/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	AMS_LogErr("Error Handler Tripped\r\n", strlen("Error Handler Tripped\r\n"));
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
