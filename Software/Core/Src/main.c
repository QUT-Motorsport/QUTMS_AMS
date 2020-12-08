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
#include <math.h>
#include <sys/unistd.h>
#include "AMS_CAN_Messages.h"
#include "BMS_CAN_Messages.h"
#include "FSM.h"
#include "AMS_FSM_States.h"
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
bool charge;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/** Create global object required for RTOS */
osThreadId_t fsmThread;
const osThreadAttr_t fsmThreadAttr = {
		.name = "fsmMainThread",
		.stack_size = 2048,
		.priority = osPriorityHigh
};
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */
	charge = false;
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	/** Turn one LED 1 on, LED0 off */
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_CAN1_Init();
	MX_TIM4_Init();
	MX_CAN2_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	/** Log Boot & HAL Initionalisation */
	char magicMsg[] = "PitterPatterLetsGetChargin'\r\n";
	char dbuf[strlen(magicMsg)];

	HAL_UART_Transmit(&huart1, (uint8_t *)&magicMsg, strlen(magicMsg), HAL_MAX_DELAY);
	if(HAL_UART_Receive(&huart1, (uint8_t*)&dbuf, strlen(magicMsg), 100) == HAL_OK)
	{
		if(strcmp(dbuf,magicMsg)) {
			charge = true;
		}
	}
	printf("PWR_ENABLED\r\n");
	printf("HAL Initialisation Complete\r\n");
	if(charge) printf("Entering Charging Mode as indicated by UART loopback\r\n");


	/** ALARM Line - Safe is actually logic low */
	HAL_GPIO_WritePin(ALARM_CTRL_GPIO_Port, ALARM_CTRL_Pin, GPIO_PIN_RESET);

	/** BMS Control - HIGH (Turn on all BMS) */
	HAL_GPIO_WritePin(BMS_CTRL_GPIO_Port, BMS_CTRL_Pin, GPIO_PIN_SET);

	/** Set Initial PROFET Pin Positions (All Off) */
	/** Contactors */
	HAL_GPIO_WritePin(HVA_N_GPIO_Port, HVA_N_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVB_N_GPIO_Port, HVB_N_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVB_P_GPIO_Port, HVA_P_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HVB_P_GPIO_Port, HVB_P_Pin, GPIO_PIN_RESET);
	/** Precharge */
	HAL_GPIO_WritePin(PRECHG_GPIO_Port, PRECHG_Pin, GPIO_PIN_RESET);

	/** FAN On (Not used as of 2020 QLD Comp */
	HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_SET);

	/** Start CANBUS2 only */
	if (HAL_CAN_Start(&CANBUS2) != HAL_OK)
	{
		char msg[] = "Failed to CAN_Start CAN2";
		AMS_LogErr(msg, strlen(msg));
		char msg2[80];
		int len = snprintf(msg2, 80, "CAN2 Error Code: %liU", CANBUS2.ErrorCode);
		AMS_LogErr(msg2, len);
	}

	/** Create CAN Filter & Apply it to CANBUS2 */

	CAN_FilterTypeDef  CAN2FilterConfig;

	CAN2FilterConfig.FilterBank = 14;
	CAN2FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN2FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN2FilterConfig.FilterIdHigh = 0x0000;
	CAN2FilterConfig.FilterIdLow = 0x0001;
	CAN2FilterConfig.FilterMaskIdHigh = 0x0000;
	CAN2FilterConfig.FilterMaskIdLow = 0x0000;
	CAN2FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	CAN2FilterConfig.FilterActivation = ENABLE;
	CAN2FilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&CANBUS2, &CAN2FilterConfig) != HAL_OK)
	{
		/* Filter configuration Error */
		char msg[] = "Failed to set CAN2 Filter";
		AMS_LogErr(msg, strlen(msg));
	}

	/** Active CAN Interrupts for handling incoming messages */
	if(HAL_CAN_ActivateNotification(&CANBUS2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		char msg[] = "Failed to activate CAN2 notification on RX0";
		AMS_LogErr(msg, strlen(msg));
	}

	if(HAL_CAN_ActivateNotification(&CANBUS2, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
	{
		char msg[] = "Failed to activate CAN2 notification on RX1";
		AMS_LogErr(msg, strlen(msg));
	}

	/** Create FSM instance */
	fsm_t *fsm = fsm_new(&deadState);

	/** Create a new thread, where our FSM will run. */
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
	RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure the Systick interrupt time
	 */
	__HAL_RCC_PLLI2S_ENABLE();
}

/* USER CODE BEGIN 4 */
/** Request a voltage from the Sendyne SFP200MOD3 */
void Sendyne_requestVoltage(int index)
{
	CAN_TxHeaderTypeDef header =
	{
			.ExtId = CS_1_EXTID,
			.IDE = CAN_ID_EXT,
			.RTR = CAN_RTR_DATA,
			.DLC = 1,
			.TransmitGlobalTime = DISABLE,
	};
	uint8_t data = index;

	if(HAL_CAN_AddTxMessage(&CANBUS4, &header, &data, &AMS_GlobalState->CAN4_TxMailbox) != HAL_OK)
	{
		char msg[] = "Failed to send current sensor voltage packet";
		AMS_LogErr(msg, strlen(msg));
	}
}
/** BMS Alarm Line Timer Callback */
void IDC_Alarm_cb(void* fsm)
{
	if(HAL_GPIO_ReadPin(IDC_ALARM_GPIO_Port, IDC_ALARM_Pin) == 0)
	{
		fsm_changeState(fsm, &errorState, "BMS Alarm Triggered");
	}
}

/** Heartbeat Callback */
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

		// Are we RTD?
		bool initialised = false;
		if(fsm_getState_t(fsm) == &idleState || fsm_getState_t(fsm) == &prechargeState) {
			initialised = true;
		}

		uint16_t averageVoltage = 0;
		for(int i = 0; i < BMS_COUNT; i++)
		{
			for(int j = 0; j < BMS_VOLTAGE_COUNT; j++)
			{
				averageVoltage += AMS_GlobalState->BMSVoltages[i][j]; // Our voltages are already stored here as a 12 bit integer.
			}
		}

		averageVoltage /= (BMS_COUNT * BMS_VOLTAGE_COUNT); /**< Good to send as already multiplied by 1000 */

		uint16_t runtime = getRuntime();

		AMS_HeartbeatResponse_t canPacket = Compose_AMS_HeartbeatResponse(initialised, HVAn_state, HVBn_state, precharge_state, HVAp_state, HVBp_state, averageVoltage, runtime);
		CAN_TxHeaderTypeDef header =
		{
				.ExtId = canPacket.id,
				.IDE = CAN_ID_EXT,
				.RTR = CAN_RTR_DATA,
				.DLC = sizeof(canPacket.data),
				.TransmitGlobalTime = DISABLE,
		};

		HAL_CAN_AddTxMessage(&CANBUS2, &header, canPacket.data, &AMS_GlobalState->CAN2_TxMailbox);

		osSemaphoreRelease(AMS_GlobalState->sem);
	} else
	{
		char msg[] = "Failed to send AMS Heartbeat";
		AMS_LogErr(msg, strlen(msg));
	}
}

/** BMS Heartbeat Callback (lower 1Hz compared to 13.3Hz as above */
void heartbeatTimerBMS_cb(void *fsm)
{
	//	// Take the GlobalState sem, find our values then fire off the packet
	if(osSemaphoreAcquire(AMS_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
	{
		if(charge)
		{
			/** We are charging, so send BMSs charge enabled based heart beat */
			BMS_ChargeEnabled_t canPacket = Compose_BMS_ChargeEnabled(0x00);

			CAN_TxHeaderTypeDef header =
			{
					.ExtId = canPacket.id,
					.IDE = CAN_ID_EXT,
					.RTR = CAN_RTR_DATA,
					.DLC = 0,
					.TransmitGlobalTime = DISABLE,
			};

			HAL_CAN_AddTxMessage(&CANBUS4, &header, NULL, &AMS_GlobalState->CAN4_TxMailbox);
		} else {
			/** We are driving, so send BMSs normal heart beat */
			AMS_HeartbeatResponse_t canPacket = Compose_AMS_HeartbeatResponse(0, 0, 0, 0, 0, 0, 0, 0);

			CAN_TxHeaderTypeDef header =
			{
					.ExtId = canPacket.id,
					.IDE = CAN_ID_EXT,
					.RTR = CAN_RTR_DATA,
					.DLC = sizeof(canPacket.data),
					.TransmitGlobalTime = DISABLE,
			};

			HAL_CAN_AddTxMessage(&CANBUS4, &header, canPacket.data, &AMS_GlobalState->CAN4_TxMailbox);
		}
		osSemaphoreRelease(AMS_GlobalState->sem);
	} else
	{
		char msg[] = "Failed to send AMS Heartbeat";
		AMS_LogErr(msg, strlen(msg));
	}
}

/** Coulomb Counting Timer Callback */
void ccTimer_cb(void *fsm)
{
	//	 Request Coloumb Count From CS1
	CAN_TxHeaderTypeDef header =
	{
			.ExtId = CS_1_EXTID,
			.IDE = CAN_ID_EXT,
			.RTR = CAN_RTR_DATA,
			.DLC = 1,
			.TransmitGlobalTime = DISABLE,
	};

	uint8_t data = CURRENT_SENSOR_CC_LOW;
	if(HAL_CAN_AddTxMessage(&CANBUS4, &header, &data, &AMS_GlobalState->CAN4_TxMailbox) != HAL_OK)
	{
		char msg[] = "Failed to send current sensor packet 1";
		AMS_LogErr(msg, strlen(msg));
	}

	osDelay(1);

	uint8_t data2 = CURRENT_SENSOR_CC_HIGH;
	if(HAL_CAN_AddTxMessage(&CANBUS4, &header, &data2, &AMS_GlobalState->CAN4_TxMailbox) != HAL_OK)
	{
		char msg[] = "Failed to send current sensor packet 2";
		AMS_LogErr(msg, strlen(msg));
	}
	return;
}

/** Current Timer Callback */
void cTimer_cb(void *fsm)
{
	// Request Current From Both
	CAN_TxHeaderTypeDef header =
	{
			.ExtId = CS_1_EXTID,
			.IDE = CAN_ID_EXT,
			.RTR = CAN_RTR_DATA,
			.DLC = 1,
			.TransmitGlobalTime = DISABLE,
	};

	uint8_t data = CURRENT_SENSOR_CURRENT;
	if(HAL_CAN_AddTxMessage(&CANBUS4, &header, &data, &AMS_GlobalState->CAN4_TxMailbox) != HAL_OK)
	{
		char msg[] = "Failed to send current sensor current packet 1";
		AMS_LogErr(msg, strlen(msg));
	}

	osDelay(1);

	header.ExtId = CS_2_EXTID;

	if(HAL_CAN_AddTxMessage(&CANBUS4, &header, &data, &AMS_GlobalState->CAN4_TxMailbox) != HAL_OK)
	{
		char msg[] = "Failed to send current sensor current packet 1";
		AMS_LogErr(msg, strlen(msg));
	}
	return;
}

/** Debug Timer Callback (Forces a heartbeat to ensure logging doesnt lead to shutdown) */
void debugTimer_cb(void *fsm)
{
	heartbeatTimer_cb(fsm);
	printf("[%li] V: %f, ", getRuntime(), AMS_GlobalState->Voltage);

	printf("IC: %f, ", AMS_GlobalState->HVACurrent + AMS_GlobalState->HVBCurrent);

	printf("CC: %f\r\n", AMS_GlobalState->CoulombCount);
	return;
}

/**
 * @brief FSM thread main loop task for RTOS
 * @param fsm the FSM object passed to the loop
 */
__NO_RETURN void fsm_thread_mainLoop(void *fsm)
{
	// Reset our FSM in idleState, as we are just starting
	fsm_setLogFunction(fsm, &printf);
	fsm_reset(fsm, &initState);

	/** Wait for BMSs to boot */
//	HAL_StatusTypeDef err;
//	do
//	{
//		err = HAL_CAN_Start(&CANBUS4);
//		HAL_Delay(100);
//	} while(err != HAL_OK);
	osDelay(2750);

	/** Log above */
	printf("BMS Wake-up Timeout Complete\r\n");

	/** Now BMSs have booted, start CAN4 */
	if (HAL_CAN_Start(&CANBUS4) != HAL_OK)
	{
		char msg[] = "Failed to CAN_Start CAN4";
		AMS_LogErr(msg, strlen(msg));
		char msg2[80];
		int len = snprintf(msg2, 80, "CAN4 Error Code: %liU", CANBUS4.ErrorCode);
		AMS_LogErr(msg2, len);
		char msg3[] = "Error likely caused by BMS not powered with isolated side of BMS powered.";
		AMS_LogErr(msg3, strlen(msg3));
	}

	CAN_FilterTypeDef  CAN4FilterConfig;

	CAN4FilterConfig.FilterBank = 0;
	CAN4FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN4FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN4FilterConfig.FilterIdHigh = 0x0000;
	CAN4FilterConfig.FilterIdLow = 0x0001;
	CAN4FilterConfig.FilterMaskIdHigh = 0x0000;
	CAN4FilterConfig.FilterMaskIdLow = 0x0000;
	CAN4FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	CAN4FilterConfig.FilterActivation = ENABLE;
	CAN4FilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&CANBUS4, &CAN4FilterConfig) != HAL_OK)
	{
		/* Filter configuration Error */
		char msg[] = "Failed to set CAN4 Filter";
		AMS_LogErr(msg, strlen(msg));
	}

	if(HAL_CAN_ActivateNotification(&CANBUS4, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		char msg[] = "Failed to activate CAN4 notification on RX0";
		AMS_LogErr(msg, strlen(msg));
	}

	if(HAL_CAN_ActivateNotification(&CANBUS4, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
	{
		char msg[] = "Failed to activate CAN4 notification on RX1";
		AMS_LogErr(msg, strlen(msg));
	}

	if(fsm_getState_t(fsm) != &errorState)
	{
		/** Manually move into SoC now the BMSs have booted */
		if(charge)
		{
			fsm_changeState(fsm, &chargingState, "Entering Charging as indicated by UART Loop back");
		} else
		{
			fsm_changeState(fsm, &SoCState, "BMS timeout complete, move to SoC");
		}
	}

	for(;;)
	{
		fsm_iterate(fsm);
	}
}

/**
 * @brief Log Error to UART
 * @param msg pointer to msg
 * @param length lenth of msg
 */
void AMS_LogErr(char* error, size_t length)
{
	char* errorMsg = malloc(length + 9);
	int len = sprintf(errorMsg, "ERROR: %s\r\n", error);
	if(len == length + 9)
	{
		HAL_UART_Transmit(&huart1, (uint8_t *)errorMsg, len, HAL_MAX_DELAY);
	} else
	{
		printf("Failed to log error in AMS_LogErr\r\n");
	}
	free(errorMsg);
}
/** Unused as SD is wired incorrectly */
void AMS_LogToSD(char* msg, size_t length)
{
	for(int i = 0; i < length; i++)
	{
		// Transfer each byte of our message
		//		bbspi_transferByte(SPI, *(msg + i));
	}
}

/** Get the runtime of the FSM in seconds */
uint32_t getRuntime()
{
	return floor((HAL_GetTick() - AMS_GlobalState->startupTicks) / 1000.f);
}

#ifdef PRINTF_TO_UART
/** Override _write to log to UART */
int _write(int file, char *data, int len)
{
	if((file != STDOUT_FILENO) && (file != STDERR_FILENO))
	{
		return -1;
	}
	HAL_StatusTypeDef s = HAL_UART_Transmit(&huart1, (uint8_t*)data, len, HAL_MAX_DELAY);

	return (s == HAL_OK ? len : 0);
}
#endif

/** CAN Message Management */
void handleCAN(CAN_HandleTypeDef *hcan, int fifo)
{
	// Iterate over the CAN FIFO buffer, adding all CAN messages to the CAN Queue.
	//	printf("Handling Can \r\n");
	while(HAL_CAN_GetRxFifoFillLevel(hcan, fifo) > 0)
	{
		AMS_CAN_Generic_t msg;
		if(HAL_CAN_GetRxMessage(hcan, fifo,  &(msg.header), msg.data) != HAL_OK)
		{
			char msg[] = "Failed top read in CAN message";
			AMS_LogErr(msg, strlen(msg));
		}
		osMessageQueuePut(AMS_GlobalState->CANQueue, &msg, 0U, 0U);

		/** Send any CAN4 messages out on CAN2 */
		if(hcan == &CANBUS4)
		{
			CAN_TxHeaderTypeDef header =
			{
					.ExtId = msg.header.ExtId,
					.IDE = CAN_ID_EXT,
					.RTR = CAN_RTR_DATA,
					.DLC = sizeof(msg.header.DLC),
					.TransmitGlobalTime = DISABLE,
			};
			HAL_CAN_AddTxMessage(&CANBUS2, &header, msg.data, &AMS_GlobalState->CAN2_TxMailbox);
		}
	}
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
	char msg[] = "Error Handler Triggered";
	AMS_LogErr(msg, strlen(msg));
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
	printf("%s: Failed to assert @ [%i, %li]\r\n", "ERROR", *file, line);
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
