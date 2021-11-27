#include "arm.h"
void move_arm(arm_struct_t* arm,int vf,int vr,int vz,int vg){
    arm->angle[0]=vr;
    arm->angle[1]-=vr;
    arm->angle[2]+=vr+vz;
    arm->angle[3]=vg;
    LIMIT_MIN_MAX(arm->angle[1],200,1000);
    LIMIT_MIN_MAX(arm->angle[2],200,1000);
}
void arm_init(arm_struct_t* arm,int a, int b, int c, int d){
    arm->angle[0]=a;
    arm->angle[1]=b;
    arm->angle[2]=c;
    arm->angle[3]=d;
    return;
}