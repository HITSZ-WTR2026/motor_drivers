#ifndef CURVE_MOVE_H
#define CURVE_MOVE_H

#include <stdint.h>
#include <stdlib.h>
#include "cmsis_os2.h"
#include "interfaces/motor_if.h"
#include "math.h"

// 轨迹类型
typedef enum
{
    LINEAR,
    TRAPEZOID,
    S_CURVE
} TrajectoryType;
typedef struct
{
    Motor_PosCtrl_t* pid;
    float targetPos;
    float totalTime;
    TrajectoryType curve;
} TrajectoryParam_t;
void Move_Along_Curve(Motor_PosCtrl_t* pos_dji, float targetPos, float time, TrajectoryType curve);

#endif // CURVE_MOVE_H
