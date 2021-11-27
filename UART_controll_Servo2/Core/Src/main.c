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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
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
uint16_t current_pos1 = 400;
uint16_t current_pos2 = 500;
uint16_t current_pos3 = 600;
uint16_t current_pos4 = 950;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void servo_start(void);
void servo_grap(void);
void servo_right(void);
void servo_middle(void);
void servo_left(void);
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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, current_pos1); // START POSITION
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, current_pos2); // START POSITION
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, current_pos3); // START POSITION
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, current_pos4); // START POSITION

  char buffer[1];
  char msg[30];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // START UART
    uint8_t arr[9] = {0};
    uint8_t i = 0, myInt = 0;
    uint16_t goal;
    if (HAL_UART_Receive(&huart1, buffer, 1, 1000) == HAL_OK && buffer[0] == '!') // only read if serial is OK and there is a start char "!"
    {
      while (buffer[0] != '#') // loop till encounter the end char "#"
      {
        HAL_UART_Receive(&huart1, buffer, 1, 1000);
        // if (buffer[0] == ':') //save all char before ":" in myInt
        // {
        //   i = 0;
        //   myInt = atoi(arr);
        //   memset(&arr[0], 0, sizeof(arr)); //clear arr array
        //   continue;
        // }
        if (buffer[0] == '#') // add all char before "#" to myInt
        {
          goal = atoi(arr);
          memset(&arr[0], 0, sizeof(arr)); // clear arr array
        }
        arr[i] = buffer[0]; // add buffer char to array
        i++;
      }
      sprintf(msg, "The goal is: %d \n\r", goal);
      HAL_UART_Transmit(&huart1, msg, strlen(msg), 1000);

      // SERVO MOTOR test
      //  if (current_pos2 < goal2){
      //    for(current_pos2; current_pos2 < goal2; current_pos2++)
      //    {
      //    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2, current_pos2);
      //    HAL_Delay(1);
      //    }
      //  }
      //  else if (current_pos2 > goal2){
      //    for(current_pos2; current_pos2 > goal2; current_pos2--)
      //    {
      //    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2, current_pos2);
      //    HAL_Delay(1);
      //    }
      //  }
      // END SERVO MOTOR TEST

      if (goal == 5)
      {
        sprintf(msg, "Go to Start\r\n");
        HAL_UART_Transmit(&huart1, msg, strlen(msg), 1000);
        servo_start();
      }

      else if (goal == 4)
      {
        sprintf(msg, "Grapp the cube\r\n");
        HAL_UART_Transmit(&huart1, msg, strlen(msg), 1000);
        servo_grap();
      }

      else if (goal == 0)
      {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        HAL_Delay(500);
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        sprintf(msg, "right	\r\n");
        HAL_UART_Transmit(&huart1, msg, strlen(msg), 1000);
        servo_grap();
        servo_right();
        servo_start();
      }

      else if (goal == 1)
      {
        sprintf(msg, "middle\r\n");
        HAL_UART_Transmit(&huart1, msg, strlen(msg), 1000);
        servo_grap();
        servo_middle();
        servo_start();
      }

      else if (goal == 2)
      {
        sprintf(msg, "left\r\n");
        HAL_UART_Transmit(&huart1, msg, strlen(msg), 1000);
        servo_grap();
        servo_left();
        servo_start();
      }
    }
    // END UART
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
}

/* USER CODE BEGIN 4 */
void servo_start(void)
{
  uint16_t joint1 = 400;
  uint16_t joint2 = 600;
  uint16_t joint3 = 600;
  uint16_t joint4 = 950;

  // JOINT2
  if (current_pos2 < joint2)
  {
    for (current_pos2; current_pos2 < joint2; current_pos2++)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, current_pos2);
      HAL_Delay(10);
    }
  }
  else if (current_pos2 > joint2)
  {
    for (current_pos2; current_pos2 > joint2; current_pos2--)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, current_pos2);
      HAL_Delay(10);
    }
  }

  // JOINT3
  if (current_pos3 < joint3)
  {
    for (current_pos3; current_pos3 < joint3; current_pos3++)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, current_pos3);
      HAL_Delay(10);
    }
  }
  else if (current_pos3 > joint3)
  {
    for (current_pos3; current_pos3 > joint3; current_pos3--)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, current_pos3);
      HAL_Delay(10);
    }
  }
  // JOINT1
  if (current_pos1 < joint1)
  {
    for (current_pos1; current_pos1 < joint1; current_pos1++)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, current_pos1);
      HAL_Delay(10);
    }
  }
  else if (current_pos1 > joint1)
  {
    for (current_pos1; current_pos1 > joint1; current_pos1--)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, current_pos1);
      HAL_Delay(10);
    }
  }
  // JOINT4
  if (current_pos4 < joint4)
  {
    for (current_pos4; current_pos4 < joint4; current_pos4++)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, current_pos4);
      HAL_Delay(10);
    }
  }
  else if (current_pos4 > joint4)
  {
    for (current_pos4; current_pos4 > joint4; current_pos4--)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, current_pos4);
      HAL_Delay(10);
    }
  }
}

void servo_grap(void)
{
  uint16_t joint1 = 400;
  uint16_t joint2 = 680;
  uint16_t joint3 = 780;
  uint16_t joint4 = 650;

  // JOINT1
  if (current_pos1 < joint1)
  {
    for (current_pos1; current_pos1 < joint1; current_pos1++)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, current_pos1);
      HAL_Delay(10);
    }
  }
  else if (current_pos1 > joint1)
  {
    for (current_pos1; current_pos1 > joint1; current_pos1--)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, current_pos1);
      HAL_Delay(10);
    }
  }

  // JOINT3
  if (current_pos3 < joint3)
  {
    for (current_pos3; current_pos3 < joint3; current_pos3++)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, current_pos3);
      HAL_Delay(10);
    }
  }
  else if (current_pos3 > joint3)
  {
    for (current_pos3; current_pos3 > joint3; current_pos3--)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, current_pos3);
      HAL_Delay(10);
    }
  }
  // JOINT2
  if (current_pos2 < joint2)
  {
    for (current_pos2; current_pos2 < joint2; current_pos2++)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, current_pos2);
      HAL_Delay(10);
    }
  }
  else if (current_pos2 > joint2)
  {
    for (current_pos2; current_pos2 > joint2; current_pos2--)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, current_pos2);
      HAL_Delay(10);
    }
  }
  // JOINT4
  if (current_pos4 < joint4)
  {
    for (current_pos4; current_pos4 < joint4; current_pos4++)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, current_pos4);
      HAL_Delay(10);
    }
  }
  else if (current_pos4 > joint4)
  {
    for (current_pos4; current_pos4 > joint4; current_pos4--)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, current_pos4);
      HAL_Delay(10);
    }
  }
}

void servo_right(void)
{
  uint16_t joint1 = 650;
  uint16_t joint2 = 650;
  uint16_t joint3 = 600;
  uint16_t joint4 = 950;

  // JOINT2
  if (current_pos2 < joint2)
  {
    for (current_pos2; current_pos2 < joint2; current_pos2++)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, current_pos2);
      HAL_Delay(10);
    }
  }
  else if (current_pos2 > joint2)
  {
    for (current_pos2; current_pos2 > joint2; current_pos2--)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, current_pos2);
      HAL_Delay(10);
    }
  }

  
  // JOINT3
  if (current_pos3 < joint3)
  {
    for (current_pos3; current_pos3 < joint3; current_pos3++)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, current_pos3);
      HAL_Delay(10);
    }
  }
  else if (current_pos3 > joint3)
  {
    for (current_pos3; current_pos3 > joint3; current_pos3--)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, current_pos3);
      HAL_Delay(10);
    }
  }

  // JOINT1
  joint2 = 800;
  if (current_pos1 < joint1)
  {
    for (current_pos1; current_pos1 < joint1; current_pos1++)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, current_pos1);
      HAL_Delay(10);
    }
  }
  else if (current_pos1 > joint1)
  {
    for (current_pos1; current_pos1 > joint1; current_pos1--)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, current_pos1);
      HAL_Delay(10);
    }
  }

// JOINT2
  if (current_pos2 < joint2)
  {
    for (current_pos2; current_pos2 < joint2; current_pos2++)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, current_pos2);
      HAL_Delay(10);
    }
  }
  else if (current_pos2 > joint2)
  {
    for (current_pos2; current_pos2 > joint2; current_pos2--)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, current_pos2);
      HAL_Delay(10);
    }
  }

  // JOINT4
  if (current_pos4 < joint4)
  {
    for (current_pos4; current_pos4 < joint4; current_pos4++)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, current_pos4);
      HAL_Delay(10);
    }
  }
  else if (current_pos4 > joint4)
  {
    for (current_pos4; current_pos4 > joint4; current_pos4--)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, current_pos4);
      HAL_Delay(10);
    }
  }
}

void servo_middle(void)
{
  uint16_t joint1 = 850;
  uint16_t joint2 = 650;
  uint16_t joint3 = 600;
  uint16_t joint4 = 950;

  // JOINT2
  if (current_pos2 < joint2)
  {
    for (current_pos2; current_pos2 < joint2; current_pos2++)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, current_pos2);
      HAL_Delay(10);
    }
  }
  else if (current_pos2 > joint2)
  {
    for (current_pos2; current_pos2 > joint2; current_pos2--)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, current_pos2);
      HAL_Delay(10);
    }
  }

  
  // JOINT3
  if (current_pos3 < joint3)
  {
    for (current_pos3; current_pos3 < joint3; current_pos3++)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, current_pos3);
      HAL_Delay(10);
    }
  }
  else if (current_pos3 > joint3)
  {
    for (current_pos3; current_pos3 > joint3; current_pos3--)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, current_pos3);
      HAL_Delay(10);
    }
  }

  // JOINT1
  joint2 = 800;
  if (current_pos1 < joint1)
  {
    for (current_pos1; current_pos1 < joint1; current_pos1++)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, current_pos1);
      HAL_Delay(10);
    }
  }
  else if (current_pos1 > joint1)
  {
    for (current_pos1; current_pos1 > joint1; current_pos1--)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, current_pos1);
      HAL_Delay(10);
    }
  }

// JOINT2
  if (current_pos2 < joint2)
  {
    for (current_pos2; current_pos2 < joint2; current_pos2++)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, current_pos2);
      HAL_Delay(10);
    }
  }
  else if (current_pos2 > joint2)
  {
    for (current_pos2; current_pos2 > joint2; current_pos2--)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, current_pos2);
      HAL_Delay(10);
    }
  }

  // JOINT4
  if (current_pos4 < joint4)
  {
    for (current_pos4; current_pos4 < joint4; current_pos4++)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, current_pos4);
      HAL_Delay(10);
    }
  }
  else if (current_pos4 > joint4)
  {
    for (current_pos4; current_pos4 > joint4; current_pos4--)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, current_pos4);
      HAL_Delay(10);
    }
  }
}

void servo_left(void)
{
  uint16_t joint1 = 1050;
  uint16_t joint2 = 650;
  uint16_t joint3 = 600;
  uint16_t joint4 = 950;

  // JOINT2
  if (current_pos2 < joint2)
  {
    for (current_pos2; current_pos2 < joint2; current_pos2++)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, current_pos2);
      HAL_Delay(10);
    }
  }
  else if (current_pos2 > joint2)
  {
    for (current_pos2; current_pos2 > joint2; current_pos2--)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, current_pos2);
      HAL_Delay(10);
    }
  }

  
  // JOINT3
  if (current_pos3 < joint3)
  {
    for (current_pos3; current_pos3 < joint3; current_pos3++)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, current_pos3);
      HAL_Delay(10);
    }
  }
  else if (current_pos3 > joint3)
  {
    for (current_pos3; current_pos3 > joint3; current_pos3--)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, current_pos3);
      HAL_Delay(10);
    }
  }

  // JOINT1
  joint2 = 800;
  if (current_pos1 < joint1)
  {
    for (current_pos1; current_pos1 < joint1; current_pos1++)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, current_pos1);
      HAL_Delay(10);
    }
  }
  else if (current_pos1 > joint1)
  {
    for (current_pos1; current_pos1 > joint1; current_pos1--)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, current_pos1);
      HAL_Delay(10);
    }
  }

// JOINT2
  if (current_pos2 < joint2)
  {
    for (current_pos2; current_pos2 < joint2; current_pos2++)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, current_pos2);
      HAL_Delay(10);
    }
  }
  else if (current_pos2 > joint2)
  {
    for (current_pos2; current_pos2 > joint2; current_pos2--)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, current_pos2);
      HAL_Delay(10);
    }
  }

  // JOINT4
  if (current_pos4 < joint4)
  {
    for (current_pos4; current_pos4 < joint4; current_pos4++)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, current_pos4);
      HAL_Delay(10);
    }
  }
  else if (current_pos4 > joint4)
  {
    for (current_pos4; current_pos4 > joint4; current_pos4--)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, current_pos4);
      HAL_Delay(10);
    }
  }
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
