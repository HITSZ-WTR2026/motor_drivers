/**
 * @file    tb6612.c
 * @author  syhanjin
 * @date    2025-09-10
 */
#include "tb6612.h"
#include <string.h>

/**
 * 设置速度
 * @param hmotor handle
 * @param speed 速度 [-1, 1]
 */
void TB6612_SetSpeed(TB6612_t* hmotor, const float speed)
{
    if (speed >= 0)
    {
        HAL_GPIO_WritePin(hmotor->in1.port, hmotor->in1.pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(hmotor->in2.port, hmotor->in2.pin, GPIO_PIN_SET);
        PWM_SetDutyCircle(&hmotor->pwm, speed);
    }
    else
    {
        HAL_GPIO_WritePin(hmotor->in1.port, hmotor->in1.pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(hmotor->in2.port, hmotor->in2.pin, GPIO_PIN_RESET);
        PWM_SetDutyCircle(&hmotor->pwm, -speed);
    }
}

/**
 * 使能电机
 * @param hmotor handle
 */
void TB6612_Enable(TB6612_t* hmotor)
{
    HAL_TIM_Encoder_Start(hmotor->encoder, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(hmotor->pwm.htim, hmotor->pwm.channel);
    TB6612_SetSpeed(hmotor, 0);
    hmotor->enable = true;
}

/**
 * 禁用电机
 * @param hmotor handle
 */
void TB6612_Disable(TB6612_t* hmotor)
{
    HAL_TIM_Encoder_Stop(hmotor->encoder, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Stop(hmotor->pwm.htim, hmotor->pwm.channel);
    TB6612_SetSpeed(hmotor, 0);
    hmotor->enable = false;
}

/**
 * 初始化电机
 * @param hmotor handle
 * @param config 配置
 */
void TB6612_Init(TB6612_t* hmotor, const TB6612_Config_t config)
{
    memset(hmotor, 0, sizeof(TB6612_t));

    hmotor->encoder         = config.encoder;
    hmotor->in1.port        = config.in1.port;
    hmotor->in1.pin         = config.in1.pin;
    hmotor->in2.port        = config.in2.port;
    hmotor->in2.pin         = config.in2.pin;
    hmotor->pwm.htim        = config.pwm.htim;
    hmotor->pwm.channel     = config.pwm.channel;
    hmotor->sampling_period = config.sampling_period;
    hmotor->roto_radio      = config.roto_radio;
    hmotor->reduction_radio = config.reduction_radio;
}

/**
 * 电机编码器数据解算
 * @note 本函数应当放置在周期为 hmotor->sampling_period 的定时器回调中调用
 * @param hmotor handle
 */
void TB6612_Encoder_DataDecode(TB6612_t* hmotor)
{
    /* @note: 假定转速不会快到数值溢出 */
    const int16_t counter = __HAL_TIM_GET_COUNTER(hmotor->encoder);
    /* 计算间隔内旋转的角度 */
    const float delta = (float)counter /
                        ((float)hmotor->roto_radio * hmotor->reduction_radio) *
                        360.0f;
    hmotor->angle += delta;
    hmotor->velocity =
        delta / hmotor->sampling_period / 360.0f * 60.0f; // 实际转速 (unit: rpm)
    /* 清零计数 */
    __HAL_TIM_SET_COUNTER(hmotor->encoder, 0);
}
