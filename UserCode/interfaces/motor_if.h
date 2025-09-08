/**
 * @file    motor_if.h
 * @author  syhanjin
 * @date    2025-09-04
 * @brief   unified motor control interface
 *
 * 支持的电机类型和依赖
 *   - drivers/DJI.h: 大疆电机
 */
#ifndef MOTOR_IF_H
#define MOTOR_IF_H

#define __MOTOR_IF_VERSION__ "0.1.1"

#include <stdbool.h>
#include "libs/pid_motor.h"

#define USE_DJI

#ifdef USE_DJI
#include "drivers/DJI.h"
#endif


typedef enum
{
#ifdef USE_DJI
    MOTOR_TYPE_DJI, //< 大疆电机，依赖 drivers/DJI.h
#endif
} MotorType_t;

/**
 * 位置环控制对象
 */
typedef struct
{
    bool enable;                 //< 是否启用控制
    MotorType_t motor_type;      //< 受控电机类型
    void* motor;                 //< 受控电机
    MotorPID_t velocity_pid;     //< 内环，速度环
    MotorPID_t position_pid;     //< 外环，位置环
    uint32_t pos_vel_freq_ratio; //< 内外环频率比
    uint32_t count;              //< 计数

    struct
    {
        float error_threshold; //< 允许的误差范围
        uint32_t count_max;    //< 保持的计数范围
        uint32_t counter;      //< 就位计数
    } settle;                  //< 就位判断

} Motor_PosCtrl_t;

/**
 * 位置环控制配置
 */
typedef struct
{
    MotorType_t motor_type; //< 受控电机类型
    void* motor;            //< 受控电机
    MotorPID_Config_t velocity_pid;
    MotorPID_Config_t position_pid;
    uint32_t pos_vel_freq_ratio; //< 内外环频率比

    float error_threshold;     //< 允许的误差范围
    uint32_t settle_count_max; //< 在误差内多少周期认为就位
} Motor_PosCtrlConfig_t;

/**
 * 速度环控制对象
 */
typedef struct
{
    bool enable;            //< 是否启用控制
    MotorType_t motor_type; //< 受控电机类型
    void* motor;            //< 受控电机
    MotorPID_t pid;         //< 速度环
} Motor_VelCtrl_t;

/**
 * 速度环控制配置
 */
typedef struct
{
    MotorType_t motor_type; //< 受控电机类型
    void* motor;            //< 受控电机
    MotorPID_Config_t pid;
} Motor_VelCtrlConfig_t;

void Motor_PosCtrl_Init(Motor_PosCtrl_t* hctrl, Motor_PosCtrlConfig_t config);
void Motor_VelCtrl_Init(Motor_VelCtrl_t* hctrl, Motor_VelCtrlConfig_t config);
void Motor_PosCtrlCalculate(Motor_PosCtrl_t* hctrl);
void Motor_VelCtrlCalculate(Motor_VelCtrl_t* hctrl);

/**
 * 启用电机控制
 * @param __CTRL_HANDLE__ 受控对象 (Motor_PosCtrl_t* 或 Motor_VelCtrl_t*)
 */
#define __MOTOR_CTRL_ENABLE(__CTRL_HANDLE__) ((__CTRL_HANDLE__)->enable = true)

/**
 * 禁用电机控制
 * @param __CTRL_HANDLE__ 受控对象 (Motor_PosCtrl_t* 或 Motor_VelCtrl_t*)
 */
#define __MOTOR_CTRL_DISABLE(__CTRL_HANDLE__) ((__CTRL_HANDLE__)->enable = false)

/**
 * 设置阈值
 * @param hctrl 受控对象
 * @param threshold 阈值
 */
static inline void Motor_PosCtrl_SetErrorThreshold(Motor_PosCtrl_t* hctrl, const float threshold)
{
    if (threshold > 0)
        hctrl->settle.error_threshold = threshold;
}

/**
 * 判断电机位置环控制是否就位
 * @param hctrl 受控对象
 * @return 是否就位
 */
static inline bool Motor_PosCtrl_IsSettle(Motor_PosCtrl_t* hctrl)
{
    return hctrl->settle.counter >= hctrl->settle.count_max;
}

/**
 * 设置位置环目标值
 * @param hctrl 受控对象
 * @param ref 目标值 (unit: deg)
 */
static inline void Motor_PosCtrl_SetRef(Motor_PosCtrl_t* hctrl, const float ref)
{
    hctrl->position_pid.ref = ref;
}

/**
 * 设置速度环目标值
 * @param hctrl 受控对象
 * @param ref 目标值 (unit: rpm)
 */
static inline void Motor_VelCtrl_SetRef(Motor_VelCtrl_t* hctrl, const float ref)
{
    hctrl->pid.ref = ref;
}

#endif // MOTOR_IF_H
