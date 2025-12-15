/**
 * @file    s_curve_traj_follower.c
 * @author  syhanjin
 * @date    2025-12-15
 */
#include "s_curve_traj_follower.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * 初始化轨迹执行器
 * @param follower
 * @param config
 */
void SCurveTraj_Axis_Init(SCurveTrajFollower_Axis_t*             follower,
                          const SCurveTrajFollower_AxisConfig_t* config)
{
    if (config->update_interval == 0)
        return;

    PD_Init(&follower->pd, &config->error_pd);

    follower->ctrl = config->motor_vel_ctrl;

    follower->update_interval = config->update_interval;
    follower->v_max           = config->v_max;
    follower->a_max           = config->a_max;
    follower->j_max           = config->j_max;

    follower->now     = 0.0f;
    follower->running = false;
}

/**
 * 更新函数
 * @note 更新顺序：
 *      SCurveTraj_Axis_Update
 *      Motor_VelCtrl_Update
 * @param follower
 */
void SCurveTraj_Axis_Update(SCurveTrajFollower_Axis_t* follower)
{
    if (!follower->running)
        return;

    const float now = follower->now + follower->update_interval;
    // 计算速度前馈量
    const float ff_velocity = SCurve_CalcV(&follower->s, now);
    // 计算当前目标位置
    const float target = SCurve_CalcX(&follower->s, now);
    // 计算 PD 输出
    follower->pd.ref = target;
    follower->pd.fdb = MotorCtrl_GetAngle(follower->ctrl);
    PD_Calculate(&follower->pd);
    // 计算总速度
    const float velocity = ff_velocity + follower->pd.output;
    // 设置电机速度
    Motor_VelCtrl_SetRef(follower->ctrl, velocity);
}

/**
 * 设置目标位置
 * @param follower
 * @param target 目标角度
 * @return 是否规划成功
 */
SCurve_Result_t SCurveTraj_Axis_SetTarget(SCurveTrajFollower_Axis_t* follower, const float target)
{
    const float running = follower->running;

    follower->running = false;

    const SCurve_Result_t r = SCurve_Init(&follower->s,
                                          MotorCtrl_GetAngle(follower->ctrl),    // 从当前位置起始,
                                          target,                                // 到目标位置
                                          MotorCtrl_GetVelocity(follower->ctrl), // 保证速度连续
                                          running ? SCurve_CalcA(&follower->s, follower->now)
                                                  : 0, // 尽量保证加速度也连续,
                                          follower->v_max,
                                          follower->a_max,
                                          follower->j_max);
    follower->now           = 0;
    if (r == S_CURVE_SUCCESS)
        follower->running = true;
    else
        follower->running = running;

    return r;
}

/**
 * 初始化轨迹执行器
 * @param follower
 * @param config
 */
void SCurveTraj_Group_Init(SCurveTrajFollower_Group_t*             follower,
                           const SCurveTrajFollower_GroupConfig_t* config)
{
    if (config->update_interval == 0)
        return;

    follower->items      = config->items;
    follower->item_count = config->item_count;

    for (size_t i = 0; i < follower->item_count; i++)
    {
        follower->items[i].ctrl = config->item_configs[i].ctrl;
        PD_Init(&follower->items[i].pd, &config->item_configs[i].error_pd);
    }

    follower->update_interval = config->update_interval;
    follower->v_max           = config->v_max;
    follower->a_max           = config->a_max;
    follower->j_max           = config->j_max;

    follower->now     = 0.0f;
    follower->running = false;
}

/**
 * 更新函数
 * @note 更新顺序：
 *      SCurveTraj_Group_Update
 *      Motor_VelCtrl_Update
 * @param follower
 */
void SCurveTraj_Group_Update(SCurveTrajFollower_Group_t* follower)
{
    if (!follower->running)
        return;

    const float now = follower->now + follower->update_interval;
    // 计算速度前馈量
    const float ff_velocity = SCurve_CalcV(&follower->s, now);
    // 计算当前目标位置
    const float target = SCurve_CalcX(&follower->s, now);
    // 计算 PD 输出
    for (size_t i = 0; i < follower->item_count; i++)
    {
        follower->items[i].pd.ref = target;
        follower->items[i].pd.fdb = MotorCtrl_GetAngle(follower->items[i].ctrl);
        PD_Calculate(&follower->items[i].pd);
        // 计算总速度
        const float velocity = ff_velocity + follower->items[i].pd.output;
        // 设置电机速度
        Motor_VelCtrl_SetRef(follower->items[i].ctrl, velocity);
    }
}

/**
 * 设置目标位置
 * @param follower
 * @param target 目标角度
 * @return 是否规划成功
 */
SCurve_Result_t SCurveTraj_Group_SetTarget(SCurveTrajFollower_Group_t* follower, const float target)
{
    const float running = follower->running;

    follower->running = false;

    float start_position = 0, start_velocity = 0;

    for (size_t i = 0; i < follower->item_count; i++)
    {
        // 以平均速度和平均位置作为起始点
        start_position += MotorCtrl_GetAngle(follower->items[i].ctrl);
        start_velocity += MotorCtrl_GetVelocity(follower->items[i].ctrl);
    }
    start_position /= follower->item_count;
    start_velocity /= follower->item_count;

    const SCurve_Result_t r = SCurve_Init(&follower->s,
                                          start_position,
                                          target,         // 到目标位置
                                          start_velocity, // 保证速度连续
                                          running ? SCurve_CalcA(&follower->s, follower->now)
                                                  : 0, // 尽量保证加速度也连续,
                                          follower->v_max,
                                          follower->a_max,
                                          follower->j_max);
    follower->now           = 0;
    if (r == S_CURVE_SUCCESS)
        follower->running = true;
    else
        follower->running = running;

    return r;
}

#ifdef __cplusplus
}
#endif