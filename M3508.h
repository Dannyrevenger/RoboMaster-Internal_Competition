#ifndef _M3508_H_
#define _M3508_H_

#include "stm32f1xx_hal.h"
#include "dbus.h"
typedef enum
{

	CAN_TxPY12V_ID 	= 0x200,		//��̨12V����ID
	CAN_TxPY24V_ID	= 0x1FF,		//��̨12V����ID
//	CAN_Pitch_ID 	= 0x201,			//��̨Pitch
//	CAN_Yaw_ID   	= 0x203,			//��̨Yaw
	CAN_YAW_FEEDBACK_ID   = 0x205,		//��̨Yaw24v
	CAN_PIT_FEEDBACK_ID  = 0x206,			//��̨Yaw24v
	CAN_POKE_FEEDBACK_ID  = 0x207,
	CAN_ZGYRO_RST_ID 			= 0x404,
	CAN_ZGYRO_FEEDBACK_MSG_ID = 0x401,
	CAN_MotorLF_ID 	= 0x041,    //��ǰ
	CAN_MotorRF_ID 	= 0x042,		//��ǰ
	CAN_MotorLB_ID 	= 0x043,    //���
	CAN_MotorRB_ID 	= 0x044,		//�Һ�

	CAN_EC60_four_ID	= 0x200,	//EC60���ճ���
	CAN_backLeft_EC60_ID = 0x204, //ec60
	CAN_frontLeft_EC60_ID = 0x201, //ec60
	CAN_backRight_EC60_ID = 0x203, //ec60
	CAN_frontRight_EC60_ID = 0x202, //ec60
	
	//add by langgo
	CAN_3510Moto_ALL_ID = 0x200,
	CAN_3510Moto1_ID = 0x201,
	CAN_3510Moto2_ID = 0x202,
	CAN_3510Moto3_ID = 0x203,
	CAN_3510Moto4_ID = 0x204,
	CAN_DriverPower_ID = 0x80,
	
	
	
	CAN_HeartBeat_ID = 0x156,
	
}CAN_Message_ID;

enum{
    LLAST	= 0,
    LAST 	= 1,
    NOW 	= 2,
    
    SPEED_PID,
    DELTA_PID,
};
typedef struct __pid_t
{
    float p;
    float i;
    float d;
    
    float set[3];				//Ŀ��ֵ,����NOW�� LAST�� LLAST���ϴ�
    float get[3];				//����ֵ
    float err[3];				//���
	
    
    float pout;							//p���
    float iout;							//i���
    float dout;							//d���
    
    float speed_out;						//����λ��ʽ���
    float last_speed_out;				//�ϴ����
    float delta_u;						//��������ֵ
    float delta_out;					//��������ʽ��� = last_delta_out + delta_u
    float last_delta_out;
    
	  float max_err;
	  float deadband;				//err < deadband return
    uint32_t pid_mode;
    uint32_t MaxOutput;				//����޷�
    uint32_t IntegralLimit;		//�����޷�
    
    void (*f_param_init)(struct __pid_t *pid,  //PID������ʼ��
                    uint32_t pid_mode,
                    uint32_t maxOutput,
                    uint32_t integralLimit,
                    float p,
                    float i,
                    float d);
    void (*f_pid_reset)(struct __pid_t *pid, float p, float i, float d);		//pid���������޸�

}pid_t;

void my_can_filter_init(CAN_HandleTypeDef* hcan);
void PID_struct_init(
    pid_t* pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    
    float 	kp, 
    float 	ki, 
    float 	kd);
    
float pid_calc(pid_t* pid, float fdb, float ref);
void  get_target(RC rc_reciever);
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void cal_motor_speed(void);	
		
#endif
