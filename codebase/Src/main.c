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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// #define kp 5
// #define ki 0.1
// #define kd 2
#define i_max 6000
#define out_max 18000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
uint8_t msg[8] = {4, 0, 4, 0, 4, 0, 4, 0};                   // CAN Bus Send Buffer
static volatile uint8_t canRX[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // CAN Bus Receive Buffer
static CAN_TxHeaderTypeDef txHeader;                                // CAN Bus Receive Header
static CAN_RxHeaderTypeDef rxHeader;                         // CAN Bus Transmit Header
uint8_t dbus_rx_buffer[18]={0}; // receiver data buffer
RC_Ctl_t RC_CtrlData; // dbus channel typeder
static volatile HAL_StatusTypeDef canTxStatus = HAL_OK;
static volatile HAL_StatusTypeDef canRxStatus = HAL_OK;
uint32_t canMailbox;
uint16_t target = 600;
static volatile int16_t target_A=1200, target_B=-900, target_C=-600, target_D=850;
int vf,vr,vz,vg;
// static volatile int16_t target_A=0, target_B=0, target_C=0, target_D=0;
//A=-2, B=1.5, C=1. D=-17/12
static volatile int16_t current_rpm_A,current_rpm_B,current_rpm_C,current_rpm_D;
pid_struct_t motorA, motorB, motorC, motorD;
arm_struct_t arm;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void CAN_Set_Motor_Voltage(int16_t v1_, int16_t v2_, int16_t v3_, int16_t v4_, uint8_t target_buffer[]);
void Integrate_Contorl();
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
  MX_USART2_UART_Init();
  MX_DMA_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  CAN_FilterTypeDef canfil; // CAN Bus Filter

  canfil.FilterBank = 0;
  canfil.FilterMode = CAN_FILTERMODE_IDMASK;
  canfil.FilterFIFOAssignment = CAN_RX_FIFO1;
  canfil.FilterIdHigh = 0;
  canfil.FilterIdLow = 0;
  canfil.FilterMaskIdHigh = 0;
  canfil.FilterMaskIdLow = 0;
  canfil.FilterScale = CAN_FILTERSCALE_32BIT;
  canfil.FilterActivation = ENABLE;
  canfil.SlaveStartFilterBank = 14;

  txHeader.DLC = 8; // Number of bites to be transmitted max- 8
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.StdId = 0x200;
  txHeader.ExtId = 0x0;
  txHeader.TransmitGlobalTime = DISABLE;

  HAL_CAN_ConfigFilter(&hcan, &canfil);
  HAL_CAN_Start(&hcan);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  // HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
  // HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxHeader, (uint8_t *)canRX);
  pid_init(&motorA, 5, 0.05, 0, 8000, 16000);
  pid_init(&motorB, 8, 1.5, 0, 8000, 16000);
  pid_init(&motorC, 8, 1.5, 0, 8000, 16000);
  pid_init(&motorD, 8, 1, 0, 8000, 16000);
  arm_init(&arm, 150, 150, 150, 150);
  HAL_UART_Receive_DMA(&huart1, dbus_rx_buffer, 18);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    canRxStatus = HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO1, &rxHeader, (uint8_t *)canRX);
    switch(rxHeader.StdId){
      case 0x201:{
        current_rpm_A = -(1+~((canRX[2]<<8)+canRX[3]));
        break;
      }
      case 0x202:{
        current_rpm_B = -(1+~((canRX[2]<<8)+canRX[3]));
        break;
      }
      case 0x203:{
        current_rpm_C = -(1+~((canRX[2]<<8)+canRX[3]));
        break;
      }
      case 0x204:{
        current_rpm_D = -(1+~((canRX[2]<<8)+canRX[3]));
        break;
      }
    }
    move_arm(&arm,vf,vr,vz,vg);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, arm.angle[0]);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, arm.angle[1]>>2);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, arm.angle[2]>>2);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, arm.angle[3]);
    // HAL_Delay(1);
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

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
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
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 719;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 100000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 100000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

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
			// RC_CtrlData.mouse.x = ((int16_t)dbus_rx_buffer[6]) | ((int16_t)dbus_rx_buffer[7] << 8);
			// RC_CtrlData.mouse.y = ((int16_t)dbus_rx_buffer[8]) | ((int16_t)dbus_rx_buffer[9] << 8);
			// RC_CtrlData.mouse.z = ((int16_t)dbus_rx_buffer[10]) | ((int16_t)dbus_rx_buffer[11] << 8);
			// RC_CtrlData.mouse.press_l = dbus_rx_buffer[12];
			// RC_CtrlData.mouse.press_r = dbus_rx_buffer[13];
			// RC_CtrlData.key.v = ((int16_t)dbus_rx_buffer[14]);// | ((int16_t)pData[15] << 8);
			//HAL_UART_Receive_IT(&huart3, dbus_rx_buffer, 18);
			//HAL_UART_Receive_DMA(&huart3, dbus_rx_buffer, 18);
      Integrate_Contorl();
}
// void HAL_CAN_RxFIFO1MsgPendingCallback(CAN_HandleTypeDef *hcan){
//   for(int i=0;i<6;i++){
//     HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//     HAL_Delay(200);
//   }
//   if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rxHeader, (uint8_t *)canRX) != HAL_OK){
//     Error_Handler();
//     HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
//     canRxStatus = HAL_ERROR;
//   }
//   else{
//     // HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
//     switch(rxHeader.StdId){
//       case 0x201:{
//         current_rpm_A = -(1+~((canRX[2]<<8)+canRX[3]));
//         break;
//       }
//       case 0x202:{
//         current_rpm_B = -(1+~((canRX[2]<<8)+canRX[3]));
//         break;
//       }
//       case 0x203:{
//         current_rpm_C = -(1+~((canRX[2]<<8)+canRX[3]));
//         break;
//       }
//       case 0x204:{
//         current_rpm_D = -(1+~((canRX[2]<<8)+canRX[3]));
//         break;
//       }
//     }
//     CAN_Set_Motor_Voltage(pid_calc(&motorA,target_A,current_rpm_A),
//                           pid_calc(&motorB,target_B,current_rpm_B),
//                           pid_calc(&motorC,target_C,current_rpm_C),
//                           pid_calc(&motorD,target_D,current_rpm_D),
//                           msg);
//     canTxStatus = HAL_CAN_AddTxMessage(hcan, &txHeader, msg, &canMailbox);
//     HAL_Delay(5);
//     canRxStatus = HAL_OK;
//   }
// }
/* 
void HAL_CAN_RxFIFO1FullCallback(CAN_HandleTypeDef *hcan){
  for(int i=0;i<6;i++){
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    HAL_Delay(200);
  }
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rxHeader, (uint8_t *)canRX) != HAL_OK){
    Error_Handler();
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
    canRxStatus = HAL_ERROR;
  }
  else{
    switch(rxHeader.StdId){
      case 0x201:{
        current_rpm_A = -(1+~((canRX[2]<<8)+canRX[3]));
        break;
      }
      case 0x202:{
        current_rpm_B = -(1+~((canRX[2]<<8)+canRX[3]));
        break;
      }
      case 0x203:{
        current_rpm_C = -(1+~((canRX[2]<<8)+canRX[3]));
        break;
      }
      case 0x204:{
        current_rpm_D = -(1+~((canRX[2]<<8)+canRX[3]));
        break;
      }
    }
    CAN_Set_Motor_Voltage(pid_calc(&motorA,target_A,current_rpm_A),
                          pid_calc(&motorB,target_B,current_rpm_B),
                          pid_calc(&motorC,target_C,current_rpm_C),
                          pid_calc(&motorD,target_D,current_rpm_D),
                          msg);
    canTxStatus = HAL_CAN_AddTxMessage(hcan, &txHeader, msg, &canMailbox);
    HAL_Delay(5);
    canRxStatus = HAL_OK;
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
  }
} */
void CAN_Set_Motor_Voltage(int16_t v1_, int16_t v2_, int16_t v3_, int16_t v4_, uint8_t target_buffer[]){
  uint16_t v1=v1_,v2=v2_,v3=v3_,v4=v4_;
  target_buffer[0] = v1 >> 8;
  target_buffer[1] = v1;
  target_buffer[2] = v2 >> 8;
  target_buffer[3] = v2;
  target_buffer[4] = v3 >> 8;
  target_buffer[5] = v3;
  target_buffer[6] = v4 >> 8;
  target_buffer[7] = v4;
  return;
}
void Integrate_Contorl(){
  switch(RC_CtrlData.rc.s2){
    case 1:{
      //A=2, B=-1.5, C=-1. D=17/12
      int vv,vp,vr;
      vv=(RC_CtrlData.rc.ch3-1024)*4;
      vp=(RC_CtrlData.rc.ch2-1024)*4;
      vr=(RC_CtrlData.rc.ch0-1024)*4;
      vr+=vp*0.05;
      target_A=(vv+vp+vr)*(1.8);
      target_B=(vv-vp-vr)*(-1.7);
      target_C=(vv+vp-vr)*(-1);
      target_D=(vv-vp+vr)*(1.5);
      CAN_Set_Motor_Voltage(pid_calc(&motorA,target_A,current_rpm_A),
                          pid_calc(&motorB,target_B,current_rpm_B),
                          pid_calc(&motorC,target_C,current_rpm_C),
                          pid_calc(&motorD,target_D,current_rpm_D),
                          msg);
      canTxStatus = HAL_CAN_AddTxMessage(&hcan, &txHeader, msg, &canMailbox);
      break;
    }
    case 2:{
      int vp=(RC_CtrlData.rc.ch2-1024);
      target_A=vp*(1.8);
      target_B=vp*(1.7);
      target_C=vp*(-1);
      target_D=vp*(-1.5);
      CAN_Set_Motor_Voltage(pid_calc(&motorA,target_A,current_rpm_A),
                          pid_calc(&motorB,target_B,current_rpm_B),
                          pid_calc(&motorC,target_C,current_rpm_C),
                          pid_calc(&motorD,target_D,current_rpm_D),
                          msg);
      canTxStatus = HAL_CAN_AddTxMessage(&hcan, &txHeader, msg, &canMailbox);
      vf=(RC_CtrlData.rc.ch3>1024)?1:((RC_CtrlData.rc.ch3<1024)?-1:0);
      vr=(RC_CtrlData.rc.ch2>1024)?148:((RC_CtrlData.rc.ch2<1024)?152:150);
      vz=(RC_CtrlData.rc.ch1>1024)?1:((RC_CtrlData.rc.ch1<1024)?-1:0);
      vg=(RC_CtrlData.rc.s1>1)?vg++:((RC_CtrlData.rc.s1=1)?vg--:0);
      LIMIT_MIN_MAX(vg,50,250);
      break;
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
