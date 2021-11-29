#include <stdint.h>
#ifndef LIMIT_MIN_MAX(x, min, max)
#define LIMIT_MIN_MAX(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))
#endif
typedef struct _arm_struct_t{
    uint32_t angle[4];
}arm_struct_t;
void move_arm(arm_struct_t* arm,int vf,int vr,int vz,int vg);
void arm_init(arm_struct_t* arm,int a, int b, int c, int d);