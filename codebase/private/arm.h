#include "main.h"
typedef struct _arm_struct_t
{
    float pos[3];           //coordiante of the gripper in centimeter
    float v0;               //basic speed
    float angle[3];         //expected angle for the servo motors
    int res[4];             //parameter for the servo motors
    int l1,l2,l12,l22;      //length and square of the length in centimeter
    int cal[4];             //calibration of the starting angles
    int start[4];           //angle limitations
    int end[4];
}arm_struct_t;
void Init_Robotic_Arm(arm_struct_t* arm, float x, float y, float z, float v0, float l1, float l2, int start[4], int end[4], int cal[4]);
void Move_Robotic_Arm(arm_struct_t* arm, int vx, int vy, int vz);
void Set_Robotic_Arm(arm_struct_t* arm, int a1, int a2, int a3, int a4);
