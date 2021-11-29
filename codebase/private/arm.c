#include "arm.h"
void move_arm(arm_struct_t* arm,int vf,int vr,int vz,int vg){
    arm->angle[0]=vr;
    arm->angle[1]+=vf;
    arm->angle[2]+=vz;
    arm->angle[3]=vg;
    LIMIT_MIN_MAX(arm->angle[1],(50<<10),(250<<10));
    LIMIT_MIN_MAX(arm->angle[2],(50<<12),(250<<12));
}
void arm_init(arm_struct_t* arm,int a, int b, int c, int d){
    arm->angle[0]=a;
    arm->angle[1]=b<<8;
    arm->angle[2]=c<<12;
    arm->angle[3]=d;
    return;
}