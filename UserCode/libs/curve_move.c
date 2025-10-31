#include "curve_move.h"

/*
使用说明：
1. 包含头文件 "curve_move.h"。
2. 调用 Move_Along_Curve 函数，传入电机位置控制结构体指针、目标位置、运动时间和曲线类型。
3. 该函数会创建一个独立的任务来执行运动轨迹计算和位置更新。
4. 任务会根据指定的曲线类型计算位置，并在每个时间步长内更新电机位置。
5. 任务结束后会自动释放参数内存。

注意事项：
1. 没有锁，重复调用时请确保上一个任务已结束。

示例代码：

Move_Along_Curve(&pos_dji, 100.0f, 2000.0f, S_CURVE);
osDelay(2001);
Move_Along_Curve(&pos_dji, 0.0f, 2000.0f, S_CURVE);
osDelay(2001);

曲线类型：
- LINEAR: 线性运动
- TRAPEZOID: 梯形加速运动
- S_CURVE: S型曲线运动

*/




float Trajectory_GetPos(uint32_t t, float targetPos, float totalTime, TrajectoryType curve)
{
    float pos = 0.0f;
    switch (curve)
    {
    case LINEAR:
        pos = (targetPos / totalTime) * t;
        break;
    case TRAPEZOID:
        {
            float accelTime  = totalTime / 4.0f;
            float cruiseTime = totalTime / 2.0f;
            float decelTime  = totalTime / 4.0f;
            float accelDist  = (targetPos / totalTime) * accelTime / 2.0f;
            float cruiseDist = (targetPos / totalTime) * cruiseTime;
            if (t < accelTime)
            {
                pos = (targetPos / (totalTime - accelTime)) * (t * t) / (2.0f * accelTime);
            }
            else if (t < (accelTime + cruiseTime))
            {
                pos = accelDist + (targetPos / totalTime) * (t - accelTime);
            }
            else if (t <= totalTime)
            {
                float dt = t - (accelTime + cruiseTime);
                pos      = accelDist + cruiseDist + (targetPos / (totalTime - decelTime)) * dt -
                      (targetPos / (totalTime - decelTime)) * (dt * dt) / (2.0f * decelTime);
            }
            break;
        }
    case S_CURVE:
        {
            float a = targetPos / totalTime;
            pos     = a * (t - (totalTime / (2.0f * M_PI)) * sin((2.0f * M_PI * t) / totalTime));
            break;
        }
    default:
        pos = 0.0f;
        break;
    }
    return pos;
}

void TrajectoryTask(void* argument)
{
    TrajectoryParam_t* param = (TrajectoryParam_t*)argument;
    Motor_PosCtrl_t* pid     = param->pid;

    float startPos       = pid->position_pid.fdb;
    float targetPos      = param->targetPos;
    float totalTime      = param->totalTime; // 单位: ms
    TrajectoryType curve = param->curve;

    const uint8_t dt = 10; // 10ms
    uint32_t t       = 0;

    while (t < totalTime)
    {
        float pos_ref = Trajectory_GetPos(t, targetPos - startPos, totalTime, curve);
        pid->position = startPos + pos_ref;

        t += dt;
        osDelay(dt); // ms
    }

    // 最后确保目标位置到达
    pid->position_pid.ref = targetPos;

    free(param); // 释放参数内存

    osThreadExit(); // 结束任务
}

// 实现电机按照指定曲线运动的功能

void Move_Along_Curve(Motor_PosCtrl_t* pid, float targetPos, float time, TrajectoryType curve)
{
    TrajectoryParam_t* param = malloc(sizeof(TrajectoryParam_t));
    if (param == NULL)
        return; // 内存不足保护

    param->pid       = pid;
    param->targetPos = targetPos;
    param->totalTime = time;
    param->curve     = curve;

    // 定义任务属性
    const osThreadAttr_t tuntoTaskAttr = {
        .name       = "TuntoTask",
        .priority   = osPriorityNormal,
        .stack_size = 512 // 根据曲线计算复杂度调整
    };
    // 创建轨迹任务
    osThreadId_t tuntoTaskHandle = osThreadNew(TrajectoryTask, param, &tuntoTaskAttr);

    if (tuntoTaskHandle == NULL)
    {
        free(param); // 创建失败释放内存
    }
}
