/**
 * @file    tb6612.h
 * @author  syhanjin
 * @date    2025-09-10
 * @brief   motor driver based on tb6612
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
#ifndef TB6612_H
#define TB6612_H
#include <stdbool.h>

#define __TB6612_VERSION__ "0.2.0"

#include "bsp/gpio_driver.h"
#include "bsp/pwm.h"

typedef struct
{
    bool               enable;           //< 是否启用
    bool               output_reverse;   //< 输出反向
    bool               feedback_reverse; //< 编码器反馈反向
    TIM_HandleTypeDef* encoder;          //< 使用的编码器对应的定时器
    GPIO_t             in1, in2;         //<
    PWM_t              pwm;              //< 使用的PWM通道
    float              sampling_period;  //< 编码器采样间隔 (unit: s)
    uint32_t           roto_radio;       //< 倍频器 * 线数
    float              reduction_radio;  //< 减速比

    float angle;    //< 输出轴角度 (unit: deg)
    float velocity; //< 输出轴转速 (unit: rpm)

    float duty_cmd; //< -1 ~ 1 占空比
} TB6612_t;

typedef struct
{
    bool               motor_reverse;   //< 电机反向，希望电机实际旋转方向与设置的控制方向相反时启用
    bool               encoder_reverse; //< 编码器反向，编码器方向与实际电机方向相反时启用
    TIM_HandleTypeDef* encoder;         //< 使用的编码器对应的定时器
    GPIO_t             in1, in2;        //<
    PWM_t              pwm;             //< 使用的PWM通道
    float              sampling_period; //< 编码器采样间隔 (unit: s)
    uint32_t           roto_radio;      //< 倍频器 * 线数
    float              reduction_radio; //< 减速比
} TB6612_Config_t;

#define __TB6612_GET_ANGLE(__TB6612_HANDLE__)    (((TB6612_t*) (__TB6612_HANDLE__))->angle)
#define __TB6612_GET_VELOCITY(__TB6612_HANDLE__) (((TB6612_t*) (__TB6612_HANDLE__))->velocity)
#define __TB6612_RESET_ANGLE(__TB6612_HANDLE__)  (((TB6612_t*) (__TB6612_HANDLE__))->angle = 0.0f)

void TB6612_SetSpeed(TB6612_t* hmotor, float speed);
void TB6612_Enable(TB6612_t* hmotor);
void TB6612_Disable(TB6612_t* hmotor);
void TB6612_Init(TB6612_t* hmotor, TB6612_Config_t config);
void TB6612_Encoder_DataDecode(TB6612_t* hmotor);
#endif // TB6612_H
