#ifndef __DBUS_H__
 #define __DBUS_H__


#include "stm32f1xx_hal.h"

typedef struct {
  
	uint16_t ch0;
	uint16_t ch1;
	uint16_t ch2;
	uint16_t ch3;
	uint8_t s1;
	uint8_t s2;

}RC;



typedef enum {
	CHASSIS_CONTROL = 0X01,
  ARM_CONTROL = 0X00
} CONTROLLER_MODE;



#define RC_CH_VALUE_MIN ((uint16_t)364 )
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)

#define DBUS_LENGTH  18

#define THROTTLE_CHANNLE ch3
#define YAW_CHANNEL      ch2
#define ROLL_CHANNEL    ch0
#define PITCH_CHANNEL    ch1

#define MAX_CHANNEL_VALUE 1684
#define MIDDLE_CHANNEL_VALIUE 1024
#define MIN_CHANNEL_VALUE  364

#define  S1_DOWN  0
#define  S1_MIDDLE    1

#define   S2_UP   1
#define   S2_MIDDLE 3
#define   S2_DOWN  2


void dbus_init(void);
void update_dbus(uint8_t *data);

#endif