# RoboMaster-Internal_Competition
This is the repo for the HKUST RoboMaster Internal Competition.



# software common part

==**We need to finish the following part together accoring to your division of work before this Saturday, 20 November.**==



## 1.1 remote controller

![截屏2021-11-15 上午10.36.00](https://github.com/Dannyrevenger/RoboMaster-Internal_Competition/blob/main/image/remoterflowchart.png)







![image2](https://github.com/Dannyrevenger/RoboMaster-Internal_Competition/blob/main/image/serialportdebuggersoftware.jpg)

==**The received data processed is as follows**==

```c
void RemoteDataProcess(uint8_t *pData) { 

if(pData == NULL) 

{ return; }  

RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; 

RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF; 

RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF; 

RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;  

RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2; 

RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003); 

RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8); 

RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8); 

RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8); 

RC_CtrlData.mouse.press_l = pData[12]; 

RC_CtrlData.mouse.press_r = pData[13]; 

RC_CtrlData.key.v = ((int16_t)pData[14]);// | ((int16_t)pData[15] << 8); //

your control code …. } 
```

## 1.2 servo motor MG995

==Using p w m to drive the servo motor==

<img src="https://github.com/Dannyrevenger/RoboMaster-Internal_Competition/blob/main/image/servomotor.JPG" alt="IMG_0635" style="zoom:10%;" />

```c
 /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  /* USER CODE END 2 */
```



```c
 __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, (uint16_t )temp);
```



## 1.3 brushless motor 3508

==using can protocol to drive the brushless motor 3508==

![IMG_0636](https://github.com/Dannyrevenger/RoboMaster-Internal_Competition/blob/main/image/brushlessmotor.JPG)

```c
/* the stm32 send message to control the chassis -> 4 motors */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) 
{ 
	uint32_t send_mail_box; 
	
	chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID; 
	chassis_tx_message.IDE = CAN_ID_STD; 
	chassis_tx_message.RTR = CAN_RTR_DATA;
	chassis_tx_message.DLC = 0x08; 
	
	chassis_can_send_data[0] = motor1 >> 8; 
	chassis_can_send_data[1] = motor1; 
	chassis_can_send_data[2] = motor2 >> 8; 
	chassis_can_send_data[3] = motor2; 
	chassis_can_send_data[4] = motor3 >> 8; 
	chassis_can_send_data[5] = motor3; 
	chassis_can_send_data[6] = motor4 >> 8; 
	chassis_can_send_data[7] = motor4; 
	
	HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box); 
}

```



