/**
 * @file    motor_if.c
 * @author  syhanjin
 * @date    2025-09-04
 * @brief
 *
 * Detailed description (optional).
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
#include "motor_if.h"
#include <math.h>
#include <string.h>

#ifdef __cpluscplus
extern "C" {
#endif

/**
 * 应用电流控制
 * @param motor_type 电机类型
 * @param hmotor 电机数据
 * @param current 电流 （或占空比）
 */
static inline void motor_apply_current(const MotorType_t motor_type, void* hmotor, const float current)
{
    // ATTENTION: 此处不做输出限幅校验，输出限幅应当放在 PID 参数中
    switch (motor_type)
    {
#ifdef USE_DJI
    case MOTOR_TYPE_DJI:
        __DJI_SET_IQ_CMD(hmotor, current);
        break;
#endif
#ifdef USE_TB6612
    case MOTOR_TYPE_TB6612:
        return TB6612_SetSpeed(hmotor, current);
#endif
#ifdef USE_VESC
    case MOTOR_TYPE_VESC:
        /* VESC 电调不应在控制时设置电流 */
        return;
#endif
    default:
        break;
    }
}

/**
 * 发送电机内部速度控制指令
 * @param motor_type 电机类型
 * @param hmotor 电机对象
 * @param speed 速度
 */
static inline void motor_send_internal_velocity(const MotorType_t motor_type, void* hmotor, const float speed)
{
    switch (motor_type)
    {
#ifdef USE_VESC
    case MOTOR_TYPE_VESC:
        VESC_SendSetCmd(hmotor, VESC_CAN_SET_RPM, speed);
        break;
#endif
    default:
        break;
    }
}

static inline void motor_send_internal_position(const MotorType_t motor_type, void* hmotor, const float position)
{
    switch (motor_type)
    {
#ifdef USE_VESC
    case MOTOR_TYPE_VESC:
        // 这里并不是普遍意义下的多圈位置，这里仅是单圈位置
        // VESC_SendSetCmd(hmotor, VESC_CAN_SET_POS, position);
        // break;
        return;
#endif
    default:
        break;
    }
}

static inline MotorCtrlMode_t get_default_ctrl_mode(const MotorType_t motor_type)
{
    switch (motor_type)
    {
#ifdef USE_DJI
    case MOTOR_TYPE_DJI:
        return MOTOR_CTRL_EXTERNAL_PID;
#endif
#ifdef USE_TB6612
    case MOTOR_TYPE_TB6612:
        return MOTOR_CTRL_EXTERNAL_PID;
#endif
#ifdef USE_VESC
    case MOTOR_TYPE_VESC:
        return MOTOR_CTRL_INTERNAL_VEL;
#endif
    default:
        return MOTOR_CTRL_EXTERNAL_PID;
    }
}

/**
 * 根据控制模式初始化位置控制器
 */
static inline void motor_posctrl_mode_init(Motor_PosCtrl_t* hctrl, const Motor_PosCtrlConfig_t* config)
{
    switch (hctrl->ctrl_mode)
    {
#ifdef MOTOR_IF_INTERNAL_VEL_POS
    case MOTOR_CTRL_INTERNAL_VEL_POS:
        // 完全使用内部PID控制，外部PID全部禁用
        memset(&hctrl->velocity_pid, 0, sizeof(MotorPID_t));
        memset(&hctrl->position_pid, 0, sizeof(MotorPID_t));
        hctrl->pos_vel_freq_ratio = 1;
        break;
#endif

#ifdef MOTOR_IF_INTERNAL_VEL
    case MOTOR_CTRL_INTERNAL_VEL:
        // 使用电调内部速度环，仅位置环有效
        memset(&hctrl->velocity_pid, 0, sizeof(MotorPID_t));
        MotorPID_Init(&hctrl->position_pid, config->position_pid);
        hctrl->pos_vel_freq_ratio = 1;
        break;
#endif

    default:
        // 完全外部PID控制
        MotorPID_Init(&hctrl->velocity_pid, config->velocity_pid);
        MotorPID_Init(&hctrl->position_pid, config->position_pid);
        hctrl->pos_vel_freq_ratio = config->pos_vel_freq_ratio ? config->pos_vel_freq_ratio : 1;
        break;
    }
}

/**
 * 初始化位置环控制参数
 * @param hctrl 受控对象
 * @param config 配置
 * @attention 计算函数不会对输出限幅，务必将内环输出限幅设为最大电流值
 */
void Motor_PosCtrl_Init(Motor_PosCtrl_t* hctrl, const Motor_PosCtrlConfig_t config)
{
    hctrl->motor_type = config.motor_type;
    hctrl->motor      = config.motor;
#ifdef USE_CUSTOM_CTRL_MODE
    hctrl->ctrl_mode = config.ctrl_mode;
#else
    hctrl->ctrl_mode = get_default_ctrl_mode(config.motor_type);
#endif

    motor_posctrl_mode_init(hctrl, &config);

    hctrl->settle.count_max       = config.settle_count_max ? config.settle_count_max : 50;
    hctrl->settle.error_threshold = config.error_threshold;
    hctrl->settle.counter         = 0;

    hctrl->enable = true;
}


/**
 * 根据控制模式初始化速度控制器
 */
static inline void motor_velctrl_mode_init(Motor_VelCtrl_t* hctrl, const Motor_VelCtrlConfig_t* config)
{
    switch (hctrl->ctrl_mode)
    {
#ifdef MOTOR_IF_INTERNAL_VEL_POS
    case MOTOR_CTRL_INTERNAL_VEL_POS:
        // 完全使用内部PID控制，外部PID全部禁用
#endif
#ifdef MOTOR_IF_INTERNAL_VEL
    case MOTOR_CTRL_INTERNAL_VEL:
        // 使用电调内部速度环
        memset(&hctrl->pid, 0, sizeof(MotorPID_t));
        break;
#endif
    default:
        // 完全外部PID控制
        MotorPID_Init(&hctrl->pid, config->pid);
        break;
    }
}

/**
 * 初始化速度环受控对象
 * @param hctrl 受控对象
 * @param config 配置
 * @attention 计算函数不会对输出限幅，务必将输出限幅设为最大电流值
 */
void Motor_VelCtrl_Init(Motor_VelCtrl_t* hctrl, const Motor_VelCtrlConfig_t config)
{
    hctrl->motor_type = config.motor_type;
    hctrl->motor      = config.motor;
#ifdef USE_CUSTOM_CTRL_MODE
    hctrl->ctrl_mode = config.ctrl_mode;
#else
    hctrl->ctrl_mode = get_default_ctrl_mode(config.motor_type);
#endif

    motor_velctrl_mode_init(hctrl, &config);

    hctrl->enable = true;
}

/**
 * 位置环控制计算
 * @param hctrl 受控对象
 */
void Motor_PosCtrlUpdate(Motor_PosCtrl_t* hctrl)
{
    if (!hctrl->enable)
        return;

    ++hctrl->count;

    const float angle = Motor_GetAngle(hctrl->motor_type, hctrl->motor);
    // 检测电机是否就位
    if (fabsf(angle - hctrl->position_pid.ref) < hctrl->settle.error_threshold)
        ++hctrl->settle.counter;
    else
        hctrl->settle.counter = 0;

#ifdef MOTOR_IF_INTERNAL_VEL_POS
    if (hctrl->ctrl_mode == MOTOR_CTRL_INTERNAL_VEL_POS)
    {
        motor_send_internal_position(hctrl->motor_type, hctrl->motor, hctrl->position);
        return;
    }
#endif

    if (hctrl->count == hctrl->pos_vel_freq_ratio)
    {
        hctrl->position_pid.ref = hctrl->position;
        // 反馈为当前电机输出角度
        hctrl->position_pid.fdb = angle;
        MotorPID_Calculate(&hctrl->position_pid);
        hctrl->count = 0;
    }

#ifdef MOTOR_IF_INTERNAL_VEL
    if (hctrl->ctrl_mode == MOTOR_CTRL_INTERNAL_VEL)
    {
        motor_send_internal_velocity(hctrl->motor_type, hctrl->motor, hctrl->velocity_pid.output);
        return;
    }
#endif

    hctrl->velocity_pid.ref = hctrl->position_pid.output;
    hctrl->velocity_pid.fdb = Motor_GetVelocity(hctrl->motor_type, hctrl->motor);
    MotorPID_Calculate(&hctrl->velocity_pid);
    motor_apply_current(hctrl->motor_type, hctrl->motor, hctrl->velocity_pid.output);
}

/**
 * 速度环控制计算
 * @param hctrl 受控对象
 */
void Motor_VelCtrlUpdate(Motor_VelCtrl_t* hctrl)
{
    if (!hctrl->enable)
        return;

#ifdef MOTOR_IF_INTERNAL_VEL
    if (hctrl->ctrl_mode == MOTOR_CTRL_INTERNAL_VEL)
    {
        motor_send_internal_velocity(hctrl->motor_type, hctrl->motor, hctrl->velocity);
        return;
    }
#endif

    hctrl->pid.ref = hctrl->velocity;
    hctrl->pid.fdb = Motor_GetVelocity(hctrl->motor_type, hctrl->motor);
    MotorPID_Calculate(&hctrl->pid);

    motor_apply_current(hctrl->motor_type, hctrl->motor, hctrl->pid.output);
}

#ifdef __cplusplus
}
#endif
