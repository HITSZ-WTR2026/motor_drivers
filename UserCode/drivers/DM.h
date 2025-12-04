#ifndef DM_H
#define DM_H

#include "main.h"
#include "stdbool.h"

#define MST_ID     0x114 // 反馈id，如果不喜欢这个数字可以自己改（
#define DM_CAN_NUM (2)
#define DM_NUM     (16) // 达妙电机数量上限

typedef enum
{
    DM_S3519 = 0U,

    DM_MOTOR_TYPE_COUNT
} DM_MotorType_t;

typedef enum
{
    DM_MODE_POS = 0x100,
    DM_MODE_VEL = 0x200,
    DM_MODE_MIT = 0x000
} DM_MODE_T;

typedef struct
{
    uint32_t feedback_count;
    bool     reverse;   // 是否反转
    bool     auto_zero; //  是否自动判断零点
    float    angle_zero;
    struct
    {
        float   angle;   // 目前单圈位置信息
        float   vel;     // 反馈速度信息
        float   T;       // 反馈力矩信息
        int8_t  T_MOS;   // 反馈mos温度
        int8_t  T_Rotor; // 反馈电机内部线圈平均温度
        uint8_t ERR;     // 电机目前状态

    } feedback;
    int32_t            round_cnt;
    uint8_t            id0;  // 电机id
    CAN_HandleTypeDef* hcan; // 电机挂载的can线

    // param
    float     POS_MAX;     // 最大位置角度
    float     POS_MAX_RAD; // 最大位置弧度
    float     VEL_MAX;     // 最大速度 (unit: degree/s)
    float     VEL_MAX_RAD; // 最大速度 (unit: rad/s)
    float     T_MAX;
    DM_MODE_T mode;

    float          abs_angle;          //< 电机轴输出角度 (unit: degree)
    float          vel;                // 电机轴输出速度 (unit: rpm)
    DM_MotorType_t motor_type;         //< 电机类型
    float          inv_reduction_rate; ///< 减速比
} DM_t;

typedef struct
{
    CAN_HandleTypeDef* hcan;      //< CAN 实例
    DM_t*              motors[8]; //< 电机指针数组
} DM_FeedbackMap;

typedef struct
{
    CAN_HandleTypeDef* hcan;
    uint8_t            id0;
    float              setvel;
    float              setpos;
    float              POS_MAX_RAD;
    float              VEL_MAX_RAD;
    float              T_MAX;
    DM_MODE_T          mode;
    DM_MotorType_t     motor_type;     //< 电机类型
    float              reduction_rate; ///< 外接减速比
} DM_Config_t;

#define __DM_GET_ANGLE(__DM_HANDLE__)    (((DM_t*) (__DM_HANDLE__))->abs_angle)
#define __DM_GET_VELOCITY(__DM_HANDLE__) (((DM_t*) (__DM_HANDLE__))->vel)

void DM_ERROR_HANDLER();
void DM_CAN_FilterInit(CAN_HandleTypeDef* hcan, const uint32_t filter_bank);
void DM_Init(DM_t* hdm, const DM_Config_t* dm_config);
void DM_DataDecode(DM_t* hdm, const uint8_t data[8]);
void DM_CAN_Fifo0ReceiveCallback(CAN_HandleTypeDef* hcan);
void DM_CAN_Fifo1ReceiveCallback(CAN_HandleTypeDef* hcan);
void DM_CAN_BaseReceiveCallback(const CAN_HandleTypeDef*   hcan,
                                const CAN_RxHeaderTypeDef* header,
                                const uint8_t              data[]);
void DM_Vel_SendSetCmd(DM_t* hdm, const float value_vel);
void DM_Pos_SendSetCmd(DM_t* hdm, const float value_pos);
void DM_ResetAngle(DM_t* hdm);

#endif // !DM_H
