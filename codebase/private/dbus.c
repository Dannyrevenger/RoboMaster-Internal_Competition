#include "dbus.h"

uint8_t rx_buffer[18];
uint8_t RX_STA = 0;

RC rc_reciever;

uint8_t MAX_BYTE = DBUS_LENGTH;

extern UART_HandleTypeDef huart1;

void dbus_init(void){

 HAL_UART_Receive_IT( &huart1,  rx_buffer,  18);

}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

//	//int i;
////  while(huart->Instance ==USART1){
//	
//	
////	  datas[RX_STA] = rx_buffer[0];
////		RX_STA++;
////		
////		if(RX_STA > 20){
////		
////		    RX_STA = 0;
////		}
////		if(RX_STA == 18){
////		
//			//update_dbus(rx_buffer);
//		//	break;
////		   
////			for(i = 0;i<DBUS_LENGTH; i++){
////			  
////				datas[i] = 0;
////			
////			}
////			    RX_STA = 0;
//		
////		}
////	}
//  
//	HAL_UART_Receive_IT( &huart1,  rx_buffer,  1);
//		
//}


void update_dbus(uint8_t *data){

     rc_reciever.ch0 = ((int16_t)data[0] | ((int16_t)data[1] << 8)) & 0x07FF;
     rc_reciever.ch1 = (((int16_t)data[1] >> 3) | ((int16_t)data[2] << 5))& 0x07FF;
     rc_reciever.ch2 = (((int16_t)data[2] >> 6) | ((int16_t)data[3] << 2) |((int16_t)data[4] << 10)) & 0x07FF;
     rc_reciever.ch3 = (((int16_t)data[4] >> 1) | ((int16_t)data[5]<<7)) &0x07FF;

     rc_reciever.s1 = ((data[5] >> 4) & 0x000C) >> 2;
     rc_reciever.s2 = ((data[5] >> 4) & 0x0003);

}
