/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h> 

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

uint16_t motor1_pulse=0;
uint16_t motor2_pulse=0;
uint16_t motor3_pulse=0;
uint16_t motor4_pulse=0;

uint16_t rx_count=0;
uint8_t dbus_rx_buffer[18]={0}; // receiver data buffer
RC_Ctl_t RC_CtrlData; // dbus channel typeder
int16_t sp1 = 10000;


uint8_t rxbuf[8]={0,0,0,0,0,0,0,0};
CAN_RxHeaderTypeDef RxHeader;
CAN_TxHeaderTypeDef TxHeader;
uint8_t tx_data[8];
uint32_t txmailbox;
static volatile uint32_t  can_status;

static volatile int32_t cnt =0;




	//uint32_t pTxMailbox = 0;

static volatile HAL_StatusTypeDef status = HAL_OK;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void set_motor_voltage(int16_t v1, int16_t v2, int16_t v3, int16_t v4);
//extern void CAN2_Filter_Init(void);// CAN FILTER INIT
//void CAN1_Filter_Init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor1_pulse);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, motor1_pulse);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, motor1_pulse);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, motor1_pulse);
  HAL_UART_Receive_DMA(&huart3, dbus_rx_buffer, 18);

  //CAN2_Filter_Init();// CAN FILTER INIT

	 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//set_motor_voltage(20000, 20000, 20000, 20000);
		
		switch (RC_CtrlData.rc.s2)
			{
				case 1:
				{
					if(RC_CtrlData.rc.ch0 > 1024 && abs(RC_CtrlData.rc.ch1 - 1024) < 20 ){
						motor4_pulse += (RC_CtrlData.rc.ch0-1024)*0.66;	

					}
					else if (RC_CtrlData.rc.ch0 < 1024 && abs(RC_CtrlData.rc.ch1 - 1024) < 20){
						motor4_pulse -= (1024-RC_CtrlData.rc.ch0)*0.66 ;
					}
					if(RC_CtrlData.rc.ch1 > 1024 && abs(RC_CtrlData.rc.ch0 - 1024) < 20 ){
						motor3_pulse += (RC_CtrlData.rc.ch1-1024)*0.66;						
					}
					else if (RC_CtrlData.rc.ch1 < 1024 && abs(RC_CtrlData.rc.ch0 - 1024) < 20){
						motor3_pulse -= (1024-RC_CtrlData.rc.ch1)*0.88 ;
					}					
					break;				
				}
				case 3:
				{
					if(RC_CtrlData.rc.ch0 > 1024 && abs(RC_CtrlData.rc.ch1 - 1024) < 20 ){
						motor2_pulse += (RC_CtrlData.rc.ch0-1024)*0.005;						
					}
					else if (RC_CtrlData.rc.ch0 < 1024 && abs(RC_CtrlData.rc.ch1 - 1024) < 20){
						motor2_pulse -= (1024-RC_CtrlData.rc.ch0)*0.005 ;
					}
					if(RC_CtrlData.rc.ch1 > 1024 && abs(RC_CtrlData.rc.ch0 - 1024) < 20 ){
						motor1_pulse += (RC_CtrlData.rc.ch1-1024)*0.66;						
					}
					else if (RC_CtrlData.rc.ch1 < 1024 && abs(RC_CtrlData.rc.ch0 - 1024) < 20){
						motor1_pulse -= (1024-RC_CtrlData.rc.ch1)*0.66 ;
					}					
					break;				
				}
				default:
					break;
					
			}
		

		if(motor4_pulse >= 250){
			motor4_pulse = 250;
		}
		if(motor4_pulse <= 50){
			motor4_pulse = 50;
		}
		if(motor3_pulse >= 250){
			motor3_pulse = 250;
		}
		if(motor3_pulse <= 50){
			motor3_pulse = 50;
		}
		if(motor2_pulse >= 250){
			motor2_pulse = 250;
		}
		if(motor2_pulse <= 150){
			motor2_pulse = 150;
		}
		if(motor1_pulse >= 250){
			motor1_pulse = 250;
		}
		if(motor1_pulse <= 50){
			motor1_pulse = 50;
		}
		
			
			

	
		
		
		
	 
	   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor1_pulse);
	   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, motor2_pulse);
	   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, motor3_pulse);
	   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, motor4_pulse);
		cnt += 1;
		HAL_Delay(100);
		 

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

/* USER CODE BEGIN 4 */


void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
	

			if(dbus_rx_buffer == NULL)
			 {
			 return;
			 }

			 RC_CtrlData.rc.ch0 = ((int16_t)dbus_rx_buffer[0] | ((int16_t)dbus_rx_buffer[1] << 8)) & 0x07FF;
			 RC_CtrlData.rc.ch1 = (((int16_t)dbus_rx_buffer[1] >> 3) | ((int16_t)dbus_rx_buffer[2] << 5))& 0x07FF;
			 RC_CtrlData.rc.ch2 = (((int16_t)dbus_rx_buffer[2] >> 6) | ((int16_t)dbus_rx_buffer[3] << 2) |((int16_t)dbus_rx_buffer[4] << 10)) & 0x07FF;
			 RC_CtrlData.rc.ch3 = (((int16_t)dbus_rx_buffer[4] >> 1) | ((int16_t)dbus_rx_buffer[5]<<7)) &0x07FF;

			 RC_CtrlData.rc.s1 = ((dbus_rx_buffer[5] >> 4) & 0x000C) >> 2;
			 RC_CtrlData.rc.s2 = ((dbus_rx_buffer[5] >> 4) & 0x0003);
			 RC_CtrlData.mouse.x = ((int16_t)dbus_rx_buffer[6]) | ((int16_t)dbus_rx_buffer[7] << 8);
			 RC_CtrlData.mouse.y = ((int16_t)dbus_rx_buffer[8]) | ((int16_t)dbus_rx_buffer[9] << 8);
			 RC_CtrlData.mouse.z = ((int16_t)dbus_rx_buffer[10]) | ((int16_t)dbus_rx_buffer[11] << 8);
			 RC_CtrlData.mouse.press_l = dbus_rx_buffer[12];
			 RC_CtrlData.mouse.press_r = dbus_rx_buffer[13];
			 RC_CtrlData.key.v = ((int16_t)dbus_rx_buffer[14]);// | ((int16_t)pData[15] << 8);
			//HAL_UART_Receive_IT(&huart3, dbus_rx_buffer, 18);
			//HAL_UART_Receive_DMA(&huart3, dbus_rx_buffer, 18);

	
}

void set_motor_voltage(int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
		
  TxHeader.StdId = 0x1ff;
  TxHeader.ExtId = 0x00;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 8;
  TxHeader.TransmitGlobalTime = DISABLE;
	tx_data[0] = (v1>>8)&0xff;
	tx_data[1] =	(v1)&0xff;
	tx_data[2] = (v2>>8)&0xff;
	tx_data[3] =	(v2)&0xff;
	tx_data[4] = (v3>>8)&0xff;
	tx_data[5] =	(v3)&0xff;
	tx_data[6] = (v4>>8)&0xff;
	tx_data[7] =	(v4)&0xff;
	//while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
	if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, tx_data, &txmailbox) != HAL_OK)
  {
    /* Transmission request Error */
	can_status = HAL_CAN_GetError(&hcan1);
    Error_Handler();
  }	
//   if((hcan1.Instance->TSR & CAN_TSR_TME0) != RESET)
//   	{
//   		txmailbox = CAN_TX_MAILBOX0;
//   	}
//   if((hcan1.Instance->TSR & CAN_TSR_TME1) != RESET)
//   	{
//   		txmailbox = CAN_TX_MAILBOX1;
//   	}

//		if((hcan1.Instance->TSR & CAN_TSR_TME2) != RESET)
//   	{
//   		txmailbox = CAN_TX_MAILBOX2;
//   	}


	
}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0 , &RxHeader, rxbuf) != HAL_OK )
		{
			Error_Handler();
		}
	 HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //OPEN CAN RX IT
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
