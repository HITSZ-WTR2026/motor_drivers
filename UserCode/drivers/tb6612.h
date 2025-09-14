/**
 * @file    tb6612.h
 * @author  syhanjin
 * @date    2025-09-10
 * @brief   motor driver based on tb6612
 */
#ifndef TB6612_H
#define TB6612_H
#include <stdbool.h>

#define __TB6612_VERSION__ "0.1.0"

#include "bsp/gpio_driver.h"
#include "bsp/pwm.h"

typedef struct
{
    bool enable;                //< 是否启用
    TIM_HandleTypeDef* encoder; //< 使用的编码器对应的定时器
    GPIO_t in1, in2;            //<
    PWM_t pwm;                  //< 使用的PWM通道
    float sampling_period;      //< 编码器采样间隔 (unit: s)
    uint32_t roto_radio;        //< 倍频器 * 线数
    float reduction_radio;      //< 减速比

    float angle;    //< 输出轴角度 (unit: deg)
    float velocity; //< 输出轴转速 (unit: rpm)

    float duty_cmd; //< -1 ~ 1 占空比
} TB6612_t;

typedef struct
{
    TIM_HandleTypeDef* encoder; //< 使用的编码器对应的定时器
    GPIO_t in1, in2;            //<
    PWM_t pwm;                  //< 使用的PWM通道
    float sampling_period;      //< 编码器采样间隔 (unit: s)
    uint32_t roto_radio;        //< 倍频器 * 线数
    float reduction_radio;      //< 减速比
} TB6612_Config_t;

#define __TB6612_GET_ANGLE(__TB6612_HANDLE__)    (((TB6612_t*)(__TB6612_HANDLE__))->angle)
#define __TB6612_GET_VELOCITY(__TB6612_HANDLE__) (((TB6612_t*)(__TB6612_HANDLE__))->velocity)

void TB6612_SetSpeed(TB6612_t* hmotor, float speed);
void TB6612_Enable(TB6612_t* hmotor);
void TB6612_Disable(TB6612_t* hmotor);
void TB6612_Init(TB6612_t* hmotor, TB6612_Config_t config);
void TB6612_Encoder_DataDecode(TB6612_t* hmotor);
#endif // TB6612_H
