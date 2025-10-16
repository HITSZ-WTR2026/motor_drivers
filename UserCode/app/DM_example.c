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

#include "bsp/can_driver.h"
#include "can.h"
#include "drivers/DM.h"
#include "interfaces/motor_if.h"
#include "tim.h"
#include "cmsis_os2.h"

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

void TIM_Callback(TIM_HandleTypeDef* htim)
{
    /**
     * 将更新后控制实例的数据更新到电机实例
     * 
     */
    Motor_PosCtrlUpdate(&pos_dm);

    /**
     * 将电机实例的数据发送给电机，达妙电机的速度环是自带的，不需要那么高的发送频率，可以不用写在定时器中断中
     * 
     */
    DM_POS_CONTROL(&hcan1,1);
}


void update()
{
    int i = 0;
    for(;;)
    {
        i=i+10;
        /**
         * 更新控制实例中的位置数据
         * 
         */
        Motor_PosCtrl_SetRef(&pos_dm,-300.0f+i);
        osDelay(1000);
    }
}




void DM_Control_Init()
{
    DM_CAN_FilterInit(&hcan1,0);

    HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, DJI_CAN_Fifo0ReceiveCallback);

    CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    /** 
     * 电机初始化，位置、速度、力矩最大参数需与调试助手初始化一致
     * 
     */
    DM_Init(&dm,(DM_Config_t){
                    .hcan       =&hcan1,
                    .id         =1     ,
                    .POS_MAX    =300   ,
                    .VEL_MAX    =100   ,
                    .T_MAX      =10    ,
                    .setvel     =10    ,
                },&hcan1);
    /**
     * 控制实例初始化
     * 
     */
    Motor_PosCtrl_Init(&pos_dm,
                        (Motor_PosCtrlConfig_t){
                            .motor_type     = MOTOR_TYPE_DM,
                            .motor          = &dm,
                        });
    /**
     * 速度实例初始化
     * 
     */
    Motor_VelCtrl_Init(&vel_dm,
                        (Motor_VelCtrlConfig_t){
                            .motor_type     = MOTOR_TYPE_DM,
                            .motor          = &dm,
                        });


    HAL_TIM_RegisterCallback(&htim3, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback);
    HAL_TIM_Base_Start_IT(&htim3);
    
    osThreadExit();

}

