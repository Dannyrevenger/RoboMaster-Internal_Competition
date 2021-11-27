#include "arm.h"
void move_arm(arm_struct_t* arm,int vf,int vr,int vz){
    arm->angle[0]+=vr;
    arm->angle[1]-=vf;
    arm->angle[2]+=vf;
    arm->angle[1]+=vz;
    arm->angle[2]+=vz;
    arm->angle[3]+=2*vz;
}
void arm_init(arm_struct_t* arm,int a, int b, int c, int d){
    arm->angle[0]=a;
    arm->angle[1]=b;
    arm->angle[2]=c;
    arm->angle[3]=d;
    return;
}