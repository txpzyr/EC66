#ifndef __ANGLE_H
#define __ANGLE_H

#include "main.h"

#define CODER_ROUND 5760
#define CODER_PAIR 5
#define CODER_ROUND_PER_PAIR 1152
#define CODER_ANGLE_PER_STEP PI / 3 / 1152

float angle_getAngle(void);
float angle_get_e_angle(void);
i32 angle_tick(void);

#endif