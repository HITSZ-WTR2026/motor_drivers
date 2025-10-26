#ifndef DM_H
#define DM_H

#define MST_ID 0x114    //反馈id，如果不喜欢这个数字可以自己改（
#define DM_CAN_NUM (2)     
#define DM_NUM  (16)    //达妙电机数量上限

typedef enum {
    DM_MODE_POS = 0x100,
    DM_MODE_VEL = 0x200
} DM_MODE_T;


#include "main.h"

typedef struct
{
    uint32_t feedback_count;

    struct{
        float pos;//反馈位置信息
        float vel;//反馈速度信息
        float T;//反馈力矩信息
        int8_t T_MOS;//反馈mos温度    
        int8_t T_Rotor;//反馈电机内部线圈平均温度
        int8_t ERR;// 电机目前状态
    }feedback;
    uint8_t id;//电机id
    CAN_HandleTypeDef* hcan;//电机挂载的can线
    
    //param
    float POS_MAX;
    float VEL_MAX;
    float T_MAX;
}DM_t;


typedef struct
{
    CAN_HandleTypeDef* hcan; //< CAN 实例
    DM_t* motors[8]; //< 电机指针数组
} DM_FeedbackMap;

typedef struct 
{
    CAN_HandleTypeDef* hcan;
    uint8_t id;
    float setvel;
    float setpos;
    float POS_MAX;
    float VEL_MAX;
    float T_MAX;
}DM_Config_t;


#define __DM_GET_ANGLE(__DM_HANDLE__)              (((DM_t*)(__DM_HANDLE__))->feedback.pos)
#define __DM_GET_VELOCITY(__DM_HANDLE__)           (((DM_t*)(__DM_HANDLE__))->feedback.vel)


void DM_ERROR_HANDLER();
void DM_CAN_FilterInit(CAN_HandleTypeDef* hcan, const uint32_t filter_bank);
void DM_Init(DM_t* hdm,const DM_Config_t dm_config,CAN_HandleTypeDef* hcan,DM_MODE_T mode);
void DM_DataDecode(DM_t* hdm, const uint8_t data[8]);
void DM_CAN_Fifo0ReceiveCallback(CAN_HandleTypeDef* hcan);
void DM_CAN_Fifo1ReceiveCallback(CAN_HandleTypeDef* hcan);
void DM_CAN_BaseReceiveCallback(const CAN_HandleTypeDef* hcan, const CAN_RxHeaderTypeDef* header, uint8_t data[]);
void DM_Vel_SendSetCmd(DM_t* hdm, const float value_vel);
void DM_Pos_SendSetCmd(DM_t* hdm, const float value_pos);

#endif // !DM_H
