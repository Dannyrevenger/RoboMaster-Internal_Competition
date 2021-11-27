#include <stdint.h>
typedef struct _arm_struct_t{
    uint16_t angle[4];
}arm_struct_t;
void move_arm(arm_struct_t* arm,int vf,int vr,int vz);
void arm_init(arm_struct_t* arm,int a, int b, int c, int d);