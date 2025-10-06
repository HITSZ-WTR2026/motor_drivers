/**
 * @file    motor_if.h
 * @author  syhanjin
 * @date    2025-09-04
 * @brief   unified motor control interface
 *
 * 支持的电机类型和依赖
 *   - drivers/DJI.h: 大疆电机
 *   - drivers/tb6612.h: 由 TB6612 芯片驱动的直流电机
 *   - drivers/vesc.h: VESC 电调驱动的电机
 *
 * --------------------------------------------------------------------------
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * Project repository: https://github.com/HITSZ-WTR2026/motor_drivers
 */
#ifndef MOTOR_IF_H
#define MOTOR_IF_H

#define __MOTOR_IF_VERSION__ "0.2.0"

#include <stdbool.h>
#include "libs/pid_motor.h"

#define USE_DJI
#define USE_TB6612
#define USE_VESC

#ifdef USE_DJI
#include "drivers/DJI.h"
#endif

#ifdef USE_TB6612
#include "drivers/tb6612.h"
#endif

#ifdef USE_VESC
#include "drivers/vesc.h"
#endif


typedef enum
{
#ifdef USE_DJI
    MOTOR_TYPE_DJI, //< 大疆电机，依赖 drivers/DJI.h
#endif
#ifdef USE_TB6612
    MOTOR_TYPE_TB6612,
#endif
#ifdef USE_VESC
    MOTOR_TYPE_VESC,
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
    float position;              //< 当前控制的位置

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
    float velocity;         //< 当前控制的速度
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
void Motor_PosCtrlUpdate(Motor_PosCtrl_t* hctrl);
void Motor_VelCtrlUpdate(Motor_VelCtrl_t* hctrl);

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
static inline void Motor_PosCtrl_SetRef(Motor_PosCtrl_t* hctrl, const float ref) { hctrl->position = ref; }

/**
 * 设置速度环目标值
 * @param hctrl 受控对象
 * @param ref 目标值 (unit: rpm)
 */
static inline void Motor_VelCtrl_SetRef(Motor_VelCtrl_t* hctrl, const float ref)
{
    hctrl->velocity = ref;
#ifdef USE_VESC
    if (hctrl->motor_type == MOTOR_TYPE_VESC)
    {
        // 在纯速度控制下 VESC 的发送频率可能不会很高（节约 CAN 总线），所以 SetRef 时就进行一次发送
        Motor_VelCtrlUpdate(hctrl);
    }
#endif
}

/* 电机反馈量 */

/**
 * 获取电机轴输出角度
 * @param motor_type 电机类型
 * @param hmotor 电机数据
 * @return 电机轴输出角度
 */
static inline float Motor_GetAngle(const MotorType_t motor_type, void* hmotor)
{
    switch (motor_type)
    {
#ifdef USE_DJI
    case MOTOR_TYPE_DJI:
        return __DJI_GET_ANGLE(hmotor);
#endif
#ifdef USE_TB6612
    case MOTOR_TYPE_TB6612:
        return __TB6612_GET_ANGLE(hmotor);
#endif
#ifdef USE_VESC
    case MOTOR_TYPE_VESC:
        return __VESC_GET_ANGLE(hmotor);
#endif
    default:
        return 0.0f;
    }
}

static inline void Motor_ResetAngle(const MotorType_t motor_type, void* hmotor)
{
    switch (motor_type)
    {
#ifdef USE_DJI
    case MOTOR_TYPE_DJI:
        DJI_ResetAngle(hmotor);
        break;
#endif
#ifdef USE_TB6612
    case MOTOR_TYPE_TB6612:
        __TB6612_RESET_ANGLE(hmotor);
        break;
#endif
#ifdef USE_VESC
    case MOTOR_TYPE_VESC:
        VESC_ResetAngle(hmotor);
        break;
#endif
    default:;
    }
}

/**
 * 获取电机转速
 * @param motor_type 电机类型
 * @param hmotor 电机数据
 * @return 电机输出转速
 */
static inline float Motor_GetVelocity(const MotorType_t motor_type, void* hmotor)
{
    switch (motor_type)
    {
#ifdef USE_DJI
    case MOTOR_TYPE_DJI:
        return __DJI_GET_VELOCITY(hmotor);
#endif
#ifdef USE_TB6612
    case MOTOR_TYPE_TB6612:
        return __TB6612_GET_VELOCITY(hmotor);
#endif
#ifdef USE_VESC
    case MOTOR_TYPE_VESC:
        return __VESC_GET_VELOCITY(hmotor);
#endif
    default:
        return 0.0f;
    }
}

#endif // MOTOR_IF_H
