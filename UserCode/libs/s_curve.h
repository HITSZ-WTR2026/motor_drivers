/**
 * @file    s_curve.h
 * @author  syhanjin
 * @date    2025-11-29
 * @brief   jerk 限制的 S 型速度规划（七段式）
 *
 * @attention 考虑到使用场景，本库只做初始状态 (xs, vs, as) 衔接，末状态为 (xe, 0, 0)
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
 * Project repository: https://github.com/HITSZ-WTRobot/s-curve-planner
 */
#ifndef S_CURVE_H
#define S_CURVE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include "main.h"

// 最大二分查找误差
#ifndef S_CURVE_MAX_BS_ERROR
#    define S_CURVE_MAX_BS_ERROR (0.001f)
#endif

/**
 * @brief 曲线加速过程
 * 带有 am jm 限制的曲线加速过程，初末加速度均为零
 */
typedef struct
{
    bool  has_uniform; ///< 是否有匀加速段
    float vs;
    float jm;

    float total_time;
    float total_distance;

    float t1; ///< 加加速段与匀加速段时刻分界
    float x1; ///< 加加速段与匀加速段距离分界
    float v1; ///< 加加速段与匀加速段速度分界
    float t2; ///< 匀加速段与减加速段时刻分界
    float x2; ///< 匀加速段与减加速段距离分界
    float v2; ///< 匀加速段与减加速段速度分界

    float ap;
    float vp;
} SCurveAccel_t;

typedef struct
{
    bool  has_const; ///< 是否有匀速段
    float direction; ///< 运行方向
    float vp;        ///< 最大速度
    float vs;        ///< 初始速度
    float as;        ///< 初始加速度
    float jm;        ///< 最大加加速度

    // 可能的加速度刹车过程
    float t0;
    float x0;

    float xs; ///< 初始位置
    float x1; ///< 加速与匀速过程位置分界
    float x2; ///< 匀速与减速过程位置分界
    float xe; ///< 末位置

    SCurveAccel_t process1;
    float         ts1; ///< 第一段非对称过程的时间偏移
    float         xs1; ///< 第一段非对称过程的起始位置
    float         t1;  ///< 加速与匀速过程时刻分界

    float t2; ///< 匀速与减速过程时刻分界

    SCurveAccel_t process3;

    float total_time;

#ifdef DEBUG
    uint32_t binary_search_count;
#endif
} SCurve_t;

typedef enum
{
    S_CURVE_SUCCESS = 0U,
    S_CURVE_FAILED,
} SCurve_Result_t;

SCurve_Result_t SCurve_Init(
        SCurve_t* s, float xs, float xe, float vs, float as, float vm, float am, float jm);

float SCurve_CalcX(const SCurve_t* s, float t);
float SCurve_CalcV(const SCurve_t* s, float t);
float SCurve_CalcA(const SCurve_t* s, float t);

#ifdef __cplusplus
}
#endif

#endif // S_CURVE_H
