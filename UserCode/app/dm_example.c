/**
 * @file DM_example.c
 * @author rediduck
 * @brief an example using the dmdriver
 * @date 2025-10-11
 *
 *
 * note：达妙电机的控制比较高级，你需要先下载一个电机调试助手
 * https://gitee.com/kit-miao/dm-tools/tree/master/USB2FDCAN/%E4%B8%8A%E4%BD%8D%E6%9C%BA
 * 在调试助手上通过串口对电机的参数进行修改并选择模式，目前该驱动只支持速度位置模式和速度模式，
 * 速度位置模式理论上位置是任意的，上电给电机使能后即为位置0点；速度模式下实测输出轴最快能达到近50rpm/s
 * 另外，电机的id以及反馈的报文id都是在调试助手中设置的，可以根据你自己设定的修改电机驱动中的滤波器设置
 * --------------------------------------------------------------------
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

#include "dm_example.h"
#include "bsp/can_driver.h"
#include "can.h"
#include "drivers/DM.h"
#include "interfaces/motor_if.h"
#include "tim.h"

/**
 *
 * 电机实例
 */
DM_t dm;

/**
 * 位置控制实例
 */
Motor_PosCtrl_t pos_dm;

/**
 * 速度控制实例
 *
 */
Motor_VelCtrl_t vel_dm;
uint32_t prescaler = 0;


void TIM_Callback(TIM_HandleTypeDef* htim)
{
    ++prescaler;
    Motor_PosCtrlUpdate(&pos_dm); // 位置控制函数，目前只支持定速度控制，速度为初始化电机时的VEL_MAX
    if (prescaler == 5)
    {
        prescaler = 0;

        Motor_VelCtrlUpdate(&vel_dm);
        // Motor_VelCtrlUpdate(&vel_dm);
    }
}


void DM_Control_Init()
{
    /**
     *
     * 初始化can滤波器
     */
    DM_CAN_FilterInit(&hcan1, 0);

    /**
     * 开启can接收回调
     *
     */
    HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, DM_CAN_Fifo0ReceiveCallback);


    /**
     * 开启can
     *
     */
    CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    /**
     *
     * 电机初始化，位置、速度、力矩最大参数需与调试助手初始化一致
     * 如果使用位置速度模式，需要把VEL_MAX设置成自己想要的运行速度。
     *
     */
    DM_Init(&dm, (DM_Config_t){
                     .hcan        = &hcan1,
                     .id0         = 0,
                     .POS_MAX_RAD = 3.1416,
                     .VEL_MAX_RAD = 40,
                     .T_MAX       = 10,
                     .mode        = DM_MODE_VEL,
                     .motor_type  = DM_S3519});

    /**
     * 位置控制实例初始化
     *
     */
    Motor_PosCtrl_Init(&pos_dm, (Motor_PosCtrlConfig_t){
                                    .motor_type   = MOTOR_TYPE_DM,
                                    .motor        = &dm,
                                    .position_pid = (MotorPID_Config_t){
                                        .Kp             = 0.0000f,
                                        .Ki             = 0.0000f,
                                        .Kd             = 0.000f,
                                        .abs_output_max = 200,
                                    }});
    /**
     * 速度控制实例初始化
     *
     */
    Motor_VelCtrl_Init(&vel_dm,
                       (Motor_VelCtrlConfig_t){
                           .motor_type = MOTOR_TYPE_DM,
                           .motor      = &dm,
                       });


    __MOTOR_CTRL_DISABLE(&vel_dm);
    __MOTOR_CTRL_ENABLE(&pos_dm);


    // Motor_PosCtrl_SetRef(&pos_dm,20000)
    // Motor_VelCtrl_SetRef(&vel_dm, 10);

    HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback);
    HAL_TIM_Base_Start_IT(&htim6);
}
