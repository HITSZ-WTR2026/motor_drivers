/**
 * @file    motor_if.c
 * @author  syhanjin
 * @date    2025-09-04
 * @brief
 */
#include "motor_if.h"
#include <math.h>

/**
 * 设置控制量
 * @param motor_type 电机类型
 * @param hmotor 电机数据
 * @param output 控制量
 */
static inline void set_output(const MotorType_t motor_type, void* hmotor, float output)
{
    // ATTENTION: 此处不做输出限幅校验，输出限幅应当放在 PID 参数中
    switch (motor_type)
    {
#ifdef USE_DJI
    case MOTOR_TYPE_DJI:
        __DJI_SET_IQ_CMD(hmotor, output);
        break;
#endif
#ifdef USE_TB6612
    case MOTOR_TYPE_TB6612:
        return TB6612_SetSpeed(hmotor, output);
#endif
    default:
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
    /* VESC 电调不能用于位置环控制 */
#ifdef USE_VESC
    if (config.motor_type == MOTOR_TYPE_VESC)
    {
        hctrl->enable     = false;
        hctrl->motor_type = MOTOR_TYPE_VESC;
        return;
    }
#endif
    hctrl->motor_type = config.motor_type;
    hctrl->motor      = config.motor;
    MotorPID_Init(&hctrl->velocity_pid, config.velocity_pid);
    MotorPID_Init(&hctrl->position_pid, config.position_pid);

    hctrl->pos_vel_freq_ratio = config.pos_vel_freq_ratio ? config.pos_vel_freq_ratio : 1;

    hctrl->settle.count_max       = config.settle_count_max ? config.settle_count_max : 50;
    hctrl->settle.error_threshold = config.error_threshold;
    hctrl->settle.counter         = 0;

    hctrl->enable = true;
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
    hctrl->enable     = true;

    /* VESC 电调忽略 PID 配置*/
#ifdef USE_VESC
    if (config.motor_type == MOTOR_TYPE_VESC)
        return;
#endif

    MotorPID_Init(&hctrl->pid, config.pid);
}

/**
 * 位置环控制计算
 * @param hctrl 受控对象
 */
void Motor_PosCtrlUpdate(Motor_PosCtrl_t* hctrl)
{
    if (!hctrl->enable)
        return;

    /* 本驱动不设计支持 VESC 的位置环控制功能 */
#ifdef USE_VESC
    if (hctrl->motor_type == MOTOR_TYPE_VESC)
        return;
#endif

    ++hctrl->count;

    const float angle = Motor_GetAngle(hctrl->motor_type, hctrl->motor);
    // 检测电机是否就位
    if (fabsf(angle - hctrl->position_pid.ref) < hctrl->settle.error_threshold)
        ++hctrl->settle.counter;
    else
        hctrl->settle.counter = 0;


    if (hctrl->count == hctrl->pos_vel_freq_ratio)
    {
        hctrl->position_pid.ref = hctrl->position;
        // 反馈为当前电机输出角度
        hctrl->position_pid.fdb = angle;
        MotorPID_Calculate(&hctrl->position_pid);
        hctrl->count = 0;
    }

    hctrl->velocity_pid.ref = hctrl->position_pid.output;
    hctrl->velocity_pid.fdb = Motor_GetVelocity(hctrl->motor_type, hctrl->motor);
    MotorPID_Calculate(&hctrl->velocity_pid);

    set_output(hctrl->motor_type, hctrl->motor, hctrl->velocity_pid.output);
}

/**
 * 速度环控制计算
 * @param hctrl 受控对象
 */
void Motor_VelCtrlUpdate(Motor_VelCtrl_t* hctrl)
{
    if (!hctrl->enable)
        return;

    /**
     * VESC 电调的 PID 控制由他自己完成，我们只需要发送控制指令
     * 控制指令频率不小于 5Hz
     */
#ifdef USE_VESC
    if (hctrl->motor_type == MOTOR_TYPE_VESC)
    {
        VESC_SendSetCmd(hctrl->motor, VESC_CAN_SET_RPM, hctrl->velocity);
        return;
    }
#endif

    hctrl->pid.ref = hctrl->velocity;
    hctrl->pid.fdb = Motor_GetVelocity(hctrl->motor_type, hctrl->motor);
    MotorPID_Calculate(&hctrl->pid);

    set_output(hctrl->motor_type, hctrl->motor, hctrl->pid.output);
}
