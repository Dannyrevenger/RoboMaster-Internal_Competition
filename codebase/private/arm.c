#include "arm.h"
void Init_Robotic_Arm(arm_struct_t* arm, float x, float y, float z, float v0, float l1, float l2, int start[3], int end[3], int cal[3])
{
    arm->pos[0]=x; arm->pos[1]=y; arm->pos[2]=z;
    arm->v0=v0;
    arm->l1=l1; arm->l2=l2;
    arm->l12=l1*l1; arm->l22=l2*l2;
    for(int i=0;i<4;i++){
        arm->start[i]=start[i];
        arm->end[i]=end[i];
        arm->cal[i]=cal[i];
    }
    return;
}
void Move_Robotic_Arm(arm_struct_t* arm, int vx, int vy, int vz)
{
    float x,y,z,A,B;
    x=(arm->pos[0])+(arm->v0)*vx;
    y=(arm->pos[1])+(arm->v0)*vy;
    z=(arm->pos[2])+(arm->v0)*vz;
    A=(x*x+y*y+z*z),B=sqrt(A);
    arm->angle[0]=atan2(x,y);
    arm->angle[1]=acos(((arm->l12)-(arm->l22)+A)/(2*(arm->l1)*B)) + acos((sqrt(x*x+y*y))/(B));
    arm->angle[2]=acos((A-(arm->l12)-(arm->l22))/(2*(arm->l1)*(arm->l2)));
    for(int i=0;i<3;i++){
        arm->res[i]=(int)(arm->angle[i]+0.5)+arm->cal[i];
        LIMIT_MIN_MAX(arm->res[i],arm->start[i],arm->end[i]);
    }
    arm->res[3]=(arm->res[1]-arm->cal[1])+(arm->res[2]-arm->cal[2])+arm->cal[3];
    return;
}
void Set_Robotic_Arm(arm_struct_t* arm, int a1, int a2, int a3, int a4){
    arm->res[0]=a1+cal[0];
    // LIMIT_MIN_MAX(arm->res[0],arm->start[0],arm->end[0]);
    arm->res[1]=a2+cal[1];
    // LIMIT_MIN_MAX(arm->res[1],arm->start[1],arm->end[1]);
    arm->res[2]=a3+cal[2];
    // LIMIT_MIN_MAX(arm->res[2],arm->start[2],arm->end[2]);
    arm->res[3]=a1+cal[3];
    // LIMIT_MIN_MAX(arm->res[3],arm->start[3],arm->end[3]);
}