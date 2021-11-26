#include "M3508.h"
#include "math.h"

#define ABS(x)		((x>0)? (x): (-x)) 


extern CAN_TxHeaderTypeDef CAN1_TX;
extern CAN_RxHeaderTypeDef CAN1_RX;

extern CAN_HandleTypeDef hcan;

extern uint8_t TX_datas[8];

extern uint32_t mailbox;
extern uint8_t RX_datas[8];

float m1_target,m2_target,m3_target,m4_target;

void abs_limit(float *a, float ABS_MAX){
    if(*a > ABS_MAX)
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX;
}

static void pid_param_init(
    pid_t *pid, 
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    float 	kp, 
    float 	ki, 
    float 	kd)
{
    
    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->pid_mode = mode;
    
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
    
}

static void pid_reset(pid_t	*pid, float kp, float ki, float kd)
{
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}


void my_can_filter_init(CAN_HandleTypeDef* _hcan)
{
	
	 CAN_FilterTypeDef  CAN_FILTER1;
	
  CAN_FILTER1.FilterBank= 0;							//过滤器0
	CAN_FILTER1.FilterMode= CAN_FILTERMODE_IDMASK; 
	CAN_FILTER1.FilterScale= CAN_FILTERSCALE_32BIT;		//32位 
	CAN_FILTER1.FilterIdHigh= 0x0000;					//32位ID
	CAN_FILTER1.FilterIdLow= 0x0000;
	CAN_FILTER1.FilterMaskIdHigh= 0x0000;				//32位MASK
	CAN_FILTER1.FilterMaskIdLow= 0x0000;
	CAN_FILTER1.FilterFIFOAssignment= CAN_RX_FIFO0;	//过滤器0关联到FIFO0
	CAN_FILTER1.FilterActivation= ENABLE;				//激活过滤器0
	
	HAL_CAN_ConfigFilter( _hcan,  &CAN_FILTER1);	//滤波器初始化

	
}

float pid_calc(pid_t* pid, float get, float set){
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;	//set - measure
    if (pid->max_err != 0 && ABS(pid->err[NOW]) >  pid->max_err  )
		return 0;
	if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
		return 0;
    
    if(pid->pid_mode == SPEED_PID) //位置式p
    {
        pid->pout = pid->p * pid->err[NOW];
        pid->iout += pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST] );
        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->speed_out = pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->speed_out), pid->MaxOutput);
        pid->last_speed_out = pid->speed_out;	//update last time 
    }
    else if(pid->pid_mode == DELTA_PID)//增量式P
    {
        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);
        
        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->delta_u = pid->pout + pid->iout + pid->dout;
        pid->delta_out = pid->last_delta_out + pid->delta_u;
        abs_limit(&(pid->delta_out), pid->MaxOutput);
        pid->last_delta_out = pid->delta_out;	//update last time
    }
    
    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];
    return pid->pid_mode==SPEED_PID ? pid->speed_out : pid->delta_out;
//	
}

void PID_struct_init(
    pid_t* pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    
    float 	kp, 
    float 	ki, 
    float 	kd)
{
    /*init function pointer*/
    pid->f_param_init = pid_param_init;
    pid->f_pid_reset = pid_reset;
//	pid->f_cal_pid = pid_calc;	
//	pid->f_cal_sp_pid = pid_sp_calc;	//addition
		
    /*init pid param */
    pid->f_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd);
	
}
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4){


   	 CAN1_TX.DLC = 0X08;
	CAN1_TX.ExtId = 0;
	CAN1_TX.IDE = CAN_ID_STD;
	CAN1_TX.RTR = CAN_RTR_DATA;
	CAN1_TX.StdId = 0X200;
	CAN1_TX.TransmitGlobalTime = DISABLE;

	TX_datas[0] = motor1>>8;
	TX_datas[1] = motor1;
	TX_datas[2] = motor2>>8;
	TX_datas[3] = motor2;
	TX_datas[4] = motor3>>8;
	TX_datas[5] = motor3;
	TX_datas[6] = motor4>>8;
	TX_datas[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan, &CAN1_TX, TX_datas, &mailbox);

}


//left up channel for forward/backward motion
//left horizontal channel for yaw motion(self spinning)
//right horizontal channel for left/right
void  get_target(RC rc_reciever){

	if((rc_reciever.ch3-MIDDLE_CHANNEL_VALIUE)>0){
   m1_target = (rc_reciever.ch3-MIDDLE_CHANNEL_VALIUE);
	 m2_target = (rc_reciever.ch3-MIDDLE_CHANNEL_VALIUE);
	 m3_target = (rc_reciever.ch3-MIDDLE_CHANNEL_VALIUE);
	 m4_target = (rc_reciever.ch3-MIDDLE_CHANNEL_VALIUE);
	}
	
	if((rc_reciever.ch3-MIDDLE_CHANNEL_VALIUE)<0){
   m1_target = (rc_reciever.ch3-MIDDLE_CHANNEL_VALIUE);
	 m2_target = (rc_reciever.ch3-MIDDLE_CHANNEL_VALIUE);
	 m3_target = (rc_reciever.ch3-MIDDLE_CHANNEL_VALIUE);
	 m4_target = (rc_reciever.ch3-MIDDLE_CHANNEL_VALIUE);
	
	}
	if((rc_reciever.ch2-MIDDLE_CHANNEL_VALIUE)<-100){
	
	 m1_target = -m1_target;
	 m2_target = m2_target;
	 m3_target = m3_target;
	 m4_target = -m4_target;
	
	
	}
	if((rc_reciever.ch2-MIDDLE_CHANNEL_VALIUE)>100){
	
   m1_target =  m1_target;
	 m2_target = -m2_target;
	 m3_target = -m3_target;
	 m4_target =  m4_target;
	
	}
	
	if((rc_reciever.ch0-MIDDLE_CHANNEL_VALIUE)<-100){
	
	 m1_target = -m1_target;
	 m2_target = m2_target;
	 m3_target = -m3_target;
	 m4_target = m4_target;
	
	
	}
	if((rc_reciever.ch0-MIDDLE_CHANNEL_VALIUE)>100){
	
	 m1_target = m1_target;
	 m2_target = -m2_target;
	 m3_target = -m3_target;
	 m4_target = m4_target;
	
	
	}
}

pid_t m1_pid;
pid_t m2_pid;
pid_t m3_pid;
pid_t m4_pid;


void cal_motor_speed(void){
	
 uint16_t cur_m1_speed,cur_m2_speed,cur_m3_speed,cur_m4_speed;
	if(CAN1_RX.StdId==CAN_frontLeft_EC60_ID)
 cur_m1_speed = 	(RX_datas[2]>>8) + RX_datas[3];
  pid_calc (&m1_pid, cur_m1_speed ,  m1_target);
	
	if(CAN1_RX.StdId==CAN_frontRight_EC60_ID)
 cur_m2_speed = 	(RX_datas[2]>>8) + RX_datas[3];
  pid_calc (&m2_pid, cur_m2_speed ,  m2_target);
	
	if(CAN1_RX.StdId==CAN_backRight_EC60_ID)
 cur_m3_speed = 	(RX_datas[2]>>8) + RX_datas[3];
  pid_calc (&m3_pid, cur_m3_speed ,  m3_target);
	
	if(CAN1_RX.StdId==CAN_backLeft_EC60_ID)
 cur_m4_speed = 	(RX_datas[2]>>8) + RX_datas[3];
  pid_calc (&m4_pid, cur_m4_speed ,  m4_target);

}

