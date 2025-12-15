/**
 * @file    s_curve.c
 * @author  syhanjin
 * @date    2025-11-30
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
#include "s_curve.h"

#include <stdbool.h>
#include <math.h>

#ifdef __cplusplus
extern "C"
{
#endif

static void SCurveAccel_Init(
        SCurveAccel_t* s, const float vs, const float vp, const float am, const float jm)
{
    s->has_uniform = jm * (vp - vs) > am * am;
    s->vs          = vs;
    s->vp          = vp;
    s->jm          = jm;
    if (s->has_uniform)
    {
        s->ap = am;

        s->t1           = am / jm;
        s->t2           = (vp - vs) / am;
        const float dt2 = s->t2 - s->t1;

        s->v1 = vs + 0.5f * am * s->t1;
        s->v2 = vp - 0.5f * am * s->t1;

        s->x1 = vs * s->t1 + 1 / 6.0f * am * s->t1 * s->t1;
        s->x2 = s->x1 + s->v1 * dt2 + 0.5f * am * dt2 * dt2;

        s->total_time     = s->t2 + s->t1;
        s->total_distance = (vp * vp - vs * vs) / (2.0f * am) +
                            0.5f * (vs + vp) * s->t1 /*(vs + vp) * am / (2.0f * jm)*/;
    }
    else
    {
        s->ap = sqrtf(jm * (vp - vs));

        s->t1 = s->ap / jm;
        s->t2 = s->t1; // 没有匀加速段

        s->v1 = vs + 0.5f * s->ap * s->ap / jm;
        s->v2 = s->v1;

        s->x1 = vs * s->t1 + 1 / 6.0f * s->ap * s->t1 * s->t1;
        s->x2 = s->x1;

        s->total_time     = 2.0f * sqrtf((vp - vs) / jm);
        s->total_distance = (vs + vp) * sqrtf((vp - vs) / jm);
    }
};

static float SCurveAccel_GetDistance(const SCurveAccel_t* s, const float t)
{
    if (t <= 0)
    {
        return 0;
    }
    if (t < s->t1)
    {
        return s->vs * t + 1 / 6.0f * s->jm * t * t * t;
    }
    // 除了这段以外两种情况都一样
    if (s->has_uniform && t < s->t2)
    {
        const float _t = t - s->t1;
        return s->x1 + s->v1 * _t + 0.5f * s->ap * _t * _t;
    }
    if (t < s->total_time)
    {
        const float _t = s->total_time - t;
        return s->total_distance - s->vp * _t + 1 / 6.0f * s->jm * _t * _t * _t;
    }
    return s->total_distance;
}

static float SCurveAccel_GetVelocity(const SCurveAccel_t* s, const float t)
{
    if (t <= 0)
    {
        return s->vs;
    }
    if (t < s->t1)
    {
        return s->vs + 0.5f * s->jm * t * t;
    }
    if (s->has_uniform && t < s->t2)
    {
        return s->v1 + s->ap * (t - s->t1);
    }
    if (t < s->total_time)
    {
        const float _t = s->total_time - t;
        return s->vp - 0.5f * s->jm * _t * _t;
    }
    return s->vp;
}

static float SCurveAccel_GetAcceleration(const SCurveAccel_t* s, const float t)
{
    if (t <= 0)
        return 0;
    if (t < s->t1)
        return s->jm * t;
    if (s->has_uniform && t < s->t2)
        return s->ap;
    if (t < s->total_time)
        return s->jm * (s->total_time - t);
    return 0;
}

SCurve_Result_t SCurve_Init(SCurve_t*   s,
                            const float xs,
                            const float xe,
                            float       vs,
                            float       as,
                            float       vm,
                            float       am,
                            float       jm)
{
    vm = fabsf(vm), am = fabsf(am), jm = fabsf(jm);

    // 全部归到正向移动判断
    const float dir = xe > xs ? 1.0f : -1.0f;
    s->direction    = dir;
    s->xs           = xs;
    s->xe           = xe;
    const float len = fabsf(xe - xs);
    vs = vs * dir, as = as * dir;
    s->vs = vs, s->as = as;
    s->jm = jm;

    if (fabsf(vs) > vm || fabsf(as) > am)
        // 如果初速度或者初加速度超限，则生成失败
        return S_CURVE_FAILED;

    // 首先假定存在匀速段
    const float vp = vm;
    float       vs0, dx0, vp_min;

    float vs1, ts1;

    if (as < 0)
    {
        // 如果初加速度与目标方向相反，则必须先刹到 0
        // 这段位移是必须的

        // 加速度刹到 0 之后的速度，作为之后过程的初速度
        vs0 = vs - 0.5f * as * as / jm;
        // 已经超限则无法构建曲线
        if (fabsf(vs0) > vm)
            return S_CURVE_FAILED;
        vp_min = vs0;

        const float ts0 = -as / jm;
        s->t0           = ts0;

        // 加速度刹到 0 的位移
        dx0   = vs * ts0 + 1 / 3.0f * as * ts0 * ts0;
        s->x0 = xs + dir * dx0;
        vs1   = vs0; // 之后无须构造偏移
        ts1   = 0;
    }
    else
    {
        // 如果初始带有同方向的加速度，必须预留速度刹车
        vp_min = vs + 0.5f * as * as / jm;
        vs0    = vs;
        dx0    = 0;
        s->t0  = 0;
        s->x0  = xs;
        // 构造带偏移的梯形加速
        if (vm < vp_min)
            // 即使 vp = vm 也无法构造出带偏移的梯形加速
            return S_CURVE_FAILED;
        ts1 = as / jm;
        vs1 = vs - 0.5f * as * ts1;
    }
    if (vp_min < 0)
        vp_min = 0;
    // 实际需要使用的 len0
    const float len0 = len - dx0;
    {
        SCurveAccel_Init(&s->process1, vs1, vp, am, jm);
        // 构造逆过程梯形加速
        SCurveAccel_Init(&s->process3, 0, vp, am, jm);
        s->xs1              = SCurveAccel_GetDistance(&s->process1, ts1);
        const float dx1     = s->process1.total_distance - s->xs1;
        const float dx3     = s->process3.total_distance;
        const float x_const = len0 - dx1 - dx3;
        if (x_const > 0)
        {
            // 存在匀速段
            s->has_const = true;
            s->ts1       = ts1;

            // 这里是时刻分界点，所以需要加上开头部分
            s->t1 = s->t0 + s->process1.total_time - ts1;

            const float t_const = x_const / vm;
            s->t2               = s->t1 + t_const;
            s->total_time       = s->t2 + s->process3.total_time;

            s->x1 = s->x0 + s->direction * dx1;
            s->x2 = s->x1 + s->direction * x_const;

            s->vp = vm;
            return S_CURVE_SUCCESS;
        }
    }
    // 不存在匀速段，求最大速度
    // 由于代数表达式太过复杂，这里直接上二分查找
    float l = vp_min, r = vm;
    float delta_d = len0, dx1 = 0, dx3 = 0;

#ifdef DEBUG
    s->binary_search_count = 0;
#endif
    // 最大迭代次数约 13 次
    while (r - vp_min > 0.001f)
    {
#ifdef DEBUG
        s->binary_search_count++;
#endif
        const float mid = 0.5f * (l + r);
        SCurveAccel_Init(&s->process1, vs1, mid, am, jm);
        SCurveAccel_Init(&s->process3, 0, mid, am, jm);
        // 更新，用于判断是否找到解
        dx1     = s->process1.total_distance - SCurveAccel_GetDistance(&s->process1, ts1);
        dx3     = s->process3.total_distance;
        delta_d = dx1 + dx3 - len0;
        if (delta_d < S_CURVE_MAX_BS_ERROR && delta_d > -S_CURVE_MAX_BS_ERROR)
        {
            r = l = mid;
            break;
        }
        if (delta_d > 0)
            r = mid;
        else
            l = mid;
    }
    if (delta_d > S_CURVE_MAX_BS_ERROR)
    {
        // 即使 vp 降到最低也无法找到解
        return S_CURVE_FAILED;
    }

    s->has_const = false;
    s->ts1       = ts1;
    // 这里是时刻分界点，所以需要加上开头部分
    s->t1         = s->t0 + s->process1.total_time - ts1;
    s->t2         = s->t1;
    s->total_time = s->t2 + s->process3.total_time;

    s->x1 = s->x0 + s->direction * dx1;
    s->x2 = s->x1;

    s->vp = r;
    return S_CURVE_SUCCESS;
}

float SCurve_CalcX(const SCurve_t* s, const float t)
{
    // 起始之前
    if (t <= 0)
        return s->xs;
    // 反向加速度刹车
    if (t < s->t0)
    {
        const float t2 = t * t;
        const float t3 = t2 * t;
        return s->xs + s->direction * (s->vs * t + 0.5f * s->as * t2 + 1 / 6.0f * s->jm * t3);
    }
    // 加速过程
    if (t < s->t1)
        return s->x0 +
               s->direction * (SCurveAccel_GetDistance(&s->process1, t - s->t0 + s->ts1) - s->xs1);
    // 匀速过程
    if (s->has_const && t < s->t2)
        return s->x1 + s->direction * s->vp * (t - s->t1);
    // 减速过程
    if (t < s->total_time)
        return s->xe - s->direction * SCurveAccel_GetDistance(&s->process3, s->total_time - t);
    return s->xe;
}

float SCurve_CalcV(const SCurve_t* s, const float t)
{
    // 起始之前
    if (t <= 0)
        return s->direction * s->vs;
    // 反向加速度刹车
    if (t < s->t0)
        return s->direction * (s->vs + s->as * t + 0.5f * s->jm * t * t);
    // 加速过程
    if (t < s->t1)
        return s->direction * SCurveAccel_GetVelocity(&s->process1, t - s->t0 + s->ts1);
    // 匀速过程
    if (s->has_const && t < s->t2)
        return s->direction * s->vp;
    // 减速过程
    if (t < s->total_time)
        return s->direction * SCurveAccel_GetVelocity(&s->process3, s->total_time - t);
    return 0;
}

float SCurve_CalcA(const SCurve_t* s, const float t)
{
    // 起始之前
    if (t <= 0)
        return s->direction * s->as;
    // 反向加速度刹车
    if (t < s->t0)
        return s->direction * (s->as + s->jm * t);
    // 加速过程
    if (t < s->t1)
        return s->direction * SCurveAccel_GetAcceleration(&s->process1, t - s->t0 + s->ts1);
    // 匀速过程
    if (s->has_const && t < s->t2)
        return 0;
    // 减速过程
    if (t < s->total_time)
        return -s->direction * SCurveAccel_GetAcceleration(&s->process3, s->total_time - t);
    return 0;
}

#ifdef __cplusplus
}
#endif