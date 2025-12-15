/**
 * @file    pid_pd.h
 * @author  syhanjin
 * @date    2025-11-08
 * @brief   PD 控制器
 *
 * 本库只包含基本的 比例 - 微分 控制以及抗饱和算法
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
 * Project repository: https://github.com/HITSZ-WTRobot/chassises_controller
 */
#ifndef PID_PD_H
#define PID_PD_H

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    /* Arguments */
    float Kp;             //< 比例系数
    float Kd;             //< 微分系数
    float abs_output_max; //< 输出限幅

    /* Runtime Data */
    float fdb;        //< 反馈量
    float cur_error;  //< 当前误差
    float prev_error; //< 上一次误差
    float output;     //< 输出量

    /**
     * 本人认为 target 表意更合适，但是战队传承的代码使用的是 ref
     * 处于对前人的尊重，此处沿用 ref 表示 PD 控制目标值
     */
    float ref; //< 目标值

} PD_t;

typedef struct
{
    float Kp;             //< 比例系数
    float Kd;             //< 微分系数
    float abs_output_max; //< 输出限幅
} PD_Config_t;

void PD_Init(PD_t* pd, const PD_Config_t* config);
void PD_Calculate(PD_t* pd);

#ifdef __cplusplus
}
#endif

#endif // PID_PD_H
