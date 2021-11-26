#include "main.h"
typedef struct _arm_struct_t
{
    float pos[3];           //coordiante of the gripper in centimeter
    float v0;               //basic speed
    float angle[3];         //expected angle for the servo motors
    int res[3];             //parameter for the servo motors
    int l1,l2,l12,l22;      //length and square of the length in centimeter
    int cal[3];             //calibration of the starting angles
    int start[3];           //angle limitations
    int end[3];
}arm_struct_t;
void Init_Robotic_Arm(arm_struct_t* arm, float x, float y, float z, float v0, float l1, float l2, int start[3], int end[3], int cal[3]);
void Set_Robotic_Arm(arm_struct_t* arm, int vx, int vy, int vz);
