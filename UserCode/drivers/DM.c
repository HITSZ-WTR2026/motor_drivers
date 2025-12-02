#include "DM.h"
#include "bsp/can_driver.h"
#include "string.h"

static DM_FeedbackMap map[DM_CAN_NUM];
static size_t         map_size = 0;

static float reduction_rate_map[DM_MOTOR_TYPE_COUNT] = {
    [DM_S3519] = (19.203),
};

static inline DM_t* getDMHandle(DM_t* motors[8], uint8_t* data, const CAN_RxHeaderTypeDef* header)
{
    if (header->IDE != CAN_ID_STD)
        return NULL;
    const int8_t id0 = (int8_t) (data[0] & 0x0f);
    // 不是 DM 的反馈数据
    if (id0 >= 16)
        return NULL;
    if (motors[id0] == NULL)
    {
        DM_ERROR_HANDLER();
        return NULL;
    }
    return motors[id0];
}
/**
 * @brief can滤波器配置，在此将你在调试助手里设置的master id对应
 *
 * @param hcan can句柄
 * @param filter_bank bank
 */
void DM_CAN_FilterInit(CAN_HandleTypeDef* hcan, const uint32_t filter_bank)
{
    const CAN_FilterTypeDef sFilterConfig = {
        .FilterIdHigh     = MST_ID << 5, // 务必匹配！
        .FilterIdLow      = 0x0000,
        .FilterMaskIdHigh = 0x7FF << 5,
        .FilterMaskIdLow  = 0x0000,
        .FilterFIFOAssignment =
                CAN_FILTER_FIFO0, // 根据需要选择，如果同一can线上挂载了不同类型的电机，请使用fifo1避免数据接收异常
        .FilterBank           = filter_bank,
        .FilterMode           = CAN_FILTERMODE_IDMASK,
        .FilterScale          = CAN_FILTERSCALE_32BIT,
        .FilterActivation     = ENABLE,
        .SlaveStartFilterBank = 14
    };
    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
    {
        DM_ERROR_HANDLER();
    }
}
/**
 * @brief 达妙电机初始化
 *
 * @param hdm 初始化的电机实例`
 * @param dm_config 配置初始化电机的配置实例
 */
void DM_Init(DM_t* hdm, const DM_Config_t* dm_config)
{
    static uint8_t initdata[8] = {
        0xFF, 0XFF, 0XFF, 0xFF, 0XFF, 0XFF, 0XFF, 0XFC
    }; // DM电机初始化需要发送的数据
    memset(hdm, 0, sizeof(DM_t));
    hdm->id0                = dm_config->id0;
    hdm->hcan               = dm_config->hcan;
    hdm->POS_MAX            = dm_config->POS_MAX_RAD * 180.0f / 3.1416f;
    hdm->VEL_MAX            = dm_config->VEL_MAX_RAD / (2 * 3.1416);
    hdm->POS_MAX_RAD        = dm_config->POS_MAX_RAD;
    hdm->VEL_MAX_RAD        = dm_config->VEL_MAX_RAD;
    hdm->T_MAX              = dm_config->T_MAX;
    hdm->mode               = dm_config->mode;
    hdm->inv_reduction_rate = 1.0f / // 取倒数将除法转为乘法加快运算速度
                              ((dm_config->reduction_rate > 0 ? dm_config->reduction_rate
                                                              : 1.0f)        // 外接减速比
                               * reduction_rate_map[dm_config->motor_type]); // 电机内部减速比
    /* 注册回调 */
    DM_t** mapped_motors = NULL;
    for (int i = 0; i < map_size; i++)
        if (map[i].hcan == hdm->hcan)
            mapped_motors = map[i].motors;
    if (mapped_motors == NULL)
    {
        // CAN 未被注册，添加到 map
        map[map_size] = (DM_FeedbackMap) {
            .hcan = hdm->hcan, .motors = { NULL } // 为了好看(
        };
        mapped_motors = map[map_size].motors;
        map_size++;
    }
    if (mapped_motors[hdm->id0] != NULL)
    {
        // 电调 id0 冲突
        DM_ERROR_HANDLER();
    }
    else
    {
        mapped_motors[hdm->id0] = hdm;
    }
    CAN_SendMessage(dm_config.hcan,
                    &(CAN_TxHeaderTypeDef) { .StdId = dm_config.mode | hdm->id0,
                                             .IDE   = CAN_ID_STD,
                                             .RTR   = CAN_RTR_DATA,
                                             .DLC   = 8 },
                    initdata);
}

/**
 * DM CAN 反馈数据解包
 * @param hdji DJI handle
 * @param data 反馈数据
 */
void DM_DataDecode(DM_t* hdm, const uint8_t data[8])
{
    float scale_angle = 2.0f * hdm->POS_MAX_RAD /
                        65535.0f; // 读取到的浮点数是和16位位置数据成线性关系，计算k值
    float scale_vel = 2.0f * hdm->VEL_MAX_RAD /
                      4095.0f; // 读取到的浮点数是和12位位置数据成线性关系，计算k值
    float scale_t = 2.0f * hdm->T_MAX / 4095.0f;

    const float feedback_angle =
            scale_angle * (uint16_t) (data[1] << 8 | data[2]) -
            hdm->POS_MAX_RAD; // 反馈位置数据，达妙3519和2520反馈的数据是减速前的
    const float feedback_vel = scale_vel * (uint16_t) (data[3] << 4 | data[4] >> 4) -
                               hdm->VEL_MAX_RAD; // 反馈速度数据，达妙3519和2520反馈的数据是减速后的
    const float feedback_t = scale_t * (uint16_t) ((data[4] & 0x0F) << 8 | data[5]); // 反馈力矩数据
    const float angle      = feedback_angle * 180.0f / 3.1416f;
    const float vel        = feedback_vel / 2.0f / 3.1416f * 60.0f;

    if (angle < -90 && hdm->feedback.angle >= 1.5708)
        hdm->round_cnt++;
    if (angle > 90 && hdm->feedback.angle < -1.5708)
        hdm->round_cnt--;

    hdm->feedback.vel     = feedback_vel;
    hdm->feedback.angle   = feedback_angle;
    hdm->feedback.T       = feedback_t;
    hdm->feedback.T_MOS   = data[6];
    hdm->feedback.T_Rotor = data[7];
    hdm->feedback_count++;
    hdm->feedback.ERR = data[0] & 0x0F;

    hdm->abs_angle = (hdm->reverse ? -1.0f : 1.0f) * // 反转时需要反转角度输入
                     ((float) hdm->round_cnt * 360.0f + angle - hdm->angle_zero) *
                     hdm->inv_reduction_rate;
    hdm->vel = (hdm->reverse ? -1.0f : 1.0f) * vel;
    hdm->feedback_count++;

    if (hdm->feedback_count == 10 && hdm->auto_zero)
    {
        // 上电后第 10 次反馈执行输出轴清零操作
        DM_ResetAngle(hdm);
    }
}

/**
 * 清零 DM 输出角度
 * @param hdm DM handle
 */
void DM_ResetAngle(DM_t* hdm)
{
    hdm->round_cnt  = 0;
    hdm->angle_zero = hdm->feedback.angle;
    hdm->abs_angle  = 0;
}

static void dm_vel_set_command_data(DM_t* hdm, const float value_vel, uint8_t data[])
{
    uint8_t* vbuf;
    vbuf    = (uint8_t*) &value_vel;
    data[0] = *(vbuf);
    data[1] = *(vbuf + 1);
    data[2] = *(vbuf + 2);
    data[3] = *(vbuf + 3);
}

static void dm_pos_set_command_data(DM_t*       hdm,
                                    const float value_vel_rad,
                                    const float value_angle_rad,
                                    uint8_t     data[])
{
    uint8_t* vbuf;
    uint8_t* pbuf;
    vbuf    = (uint8_t*) &value_vel_rad;
    pbuf    = (uint8_t*) &value_angle_rad;
    data[0] = *(pbuf);
    data[1] = *(pbuf + 1);
    data[2] = *(pbuf + 2);
    data[3] = *(pbuf + 3);
    data[4] = *(vbuf);
    data[5] = *(vbuf + 1);
    data[6] = *(vbuf + 2);
    data[7] = *(vbuf + 3);
}

void DM_Vel_SendSetCmd(DM_t* hdm, const float value_vel)
{
    static uint8_t data[8] = { 0 };

    float value_vel_rad = value_vel * 2 * 3.1416f /
                          60.0f; // 达妙电机控制的即为输出轴的速度（uint:rad/s）
    dm_vel_set_command_data(hdm, value_vel_rad, data);
    CAN_SendMessage(hdm->hcan,
                    &(CAN_TxHeaderTypeDef) {
                            .StdId = DM_MODE_VEL | hdm->id0,
                            .IDE   = CAN_ID_STD,
                            .RTR   = CAN_RTR_DATA,
                            .DLC   = 8,
                    },
                    data);
}

void DM_Pos_SendSetCmd(DM_t* hdm, const float value_pos)
{
    static uint8_t data[8]       = { 0 };
    float          value_pos_rad = value_pos * 3.1416f / 180.0f;
    dm_pos_set_command_data(hdm, hdm->VEL_MAX, value_pos_rad, data);
    CAN_SendMessage(hdm->hcan,
                    &(CAN_TxHeaderTypeDef) {
                            .StdId = DM_MODE_POS | hdm->id0,
                            .IDE   = CAN_ID_STD,
                            .RTR   = CAN_RTR_DATA,
                            .DLC   = 8,
                    },
                    data);
}

/**
 * @brief 错误处理
 *
 */
void DM_ERROR_HANDLER()
{
    // 错误处理
}

/**
 * CAN FIFO0 接收回调函数
 * @attention 必须*注册*回调函数或者在更高级的回调函数内调用此回调函数
 * @note 使用
 *          HAL_CAN_RegisterCallback(hcan, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID,
 * DJI_CAN_Fifo0ReceiveCallback); 来注册回调函数
 * @param hcan
 */
void DM_CAN_Fifo0ReceiveCallback(CAN_HandleTypeDef* hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t             data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
    {
        DM_ERROR_HANDLER();
        return;
    }
    DM_CAN_BaseReceiveCallback(hcan, &header, data);
}

/**
 * CAN FIFI1 接收回调函数
 * @deprecated Fifo1 should not be used in DJI feedback! Use Fifo0
 * @param hcan
 */
void DM_CAN_Fifo1ReceiveCallback(CAN_HandleTypeDef* hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t             data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &header, data) != HAL_OK)
    {
        DM_ERROR_HANDLER();
        return;
    }
    DM_CAN_BaseReceiveCallback(hcan, &header, data);
}

/**
 * CAN 基本接收回调函数
 * @param hcan
 * @param header
 * @param data
 */
void DM_CAN_BaseReceiveCallback(const CAN_HandleTypeDef*   hcan,
                                const CAN_RxHeaderTypeDef* header,
                                const uint8_t              data[])
{
    for (int i = 0; i < map_size; i++)
    {
        if (hcan == map[i].hcan)
        {
            DM_t* hdm = getDMHandle(map[i].motors, data, header);
            if (hdm != NULL)
                DM_DataDecode(hdm, data);
            return;
        }
    }
}
