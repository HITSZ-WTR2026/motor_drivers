#include "DM.h"
#include "bsp/can_driver.h"
#include "string.h"

static DM_FeedbackMap map[CAN_NUM];
static size_t map_size = 0;
uint8_t initdata[8]={0xFF,0XFF,0XFF,0xFF,0XFF,0XFF,0XFF,0XFC};


static inline DM_t* getDMHandle(DM_t* motors[8], uint8_t *data,const CAN_RxHeaderTypeDef* header)
{
    if (header->IDE != CAN_ID_STD)
        return NULL;
    const int8_t id = (int8_t)data[0]>>4 ;
    // 不是 DM 的反馈数据
    if (id >= 8)
        return NULL;
    if (motors[id] == NULL)
    {
        DM_ERROR_HANDLER();
        return NULL;
    }
    return motors[id];
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
        .FilterIdHigh         = 0x000 << 5,
        .FilterIdLow          = 0x0000,
        .FilterMaskIdHigh     = 0x7F0 << 5, //< 高 7 位匹配，第 4 位忽略
        .FilterMaskIdLow      = 0x0000,
        .FilterFIFOAssignment = CAN_FILTER_FIFO0,
        .FilterBank           = filter_bank,
        .FilterMode           = CAN_FILTERMODE_IDMASK,
        .FilterScale          = CAN_FILTERSCALE_32BIT,
        .FilterActivation     = ENABLE,
        .SlaveStartFilterBank = 14};
    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
    {
        DM_ERROR_HANDLER();
    }
}
/**
 * @brief 达妙电机初始化
 * 
 * @param hdm 初始化的电机实例
 * @param dm_config 配置初始化电机的配置实例
 * @param hcan can句柄
 */
void DM_Init(DM_t* hdm, const DM_Config_t dm_config,CAN_HandleTypeDef* hcan)
{
    memset(hdm, 0, sizeof(DM_t));

    hdm->id = dm_config.id;
    hdm->can = dm_config.hcan->Instance;
    hdm->POS_MAX = dm_config.POS_MAX;
    hdm->VEL_MAX = dm_config.VEL_MAX;
    hdm->T_MAX = dm_config.T_MAX;
    hdm->setvel = dm_config.setvel;
    /* 注册回调 */
    DM_t** mapped_motors = NULL;
    for (int i = 0; i < map_size; i++)
        if (map[i].can == hdm->can)
            mapped_motors = map[i].motors;
    if (mapped_motors == NULL)
    {
        // CAN 未被注册，添加到 map
        map[map_size] = (DM_FeedbackMap){
            .can    = hdm->can,
            .motors = {NULL} // 为了好看(
        };
        mapped_motors = map[map_size].motors;
        map_size++;
    }
    if (mapped_motors[hdm->id - 1] != NULL)
    {
        // 电调 ID 冲突
        DM_ERROR_HANDLER();
    }
    else
    {
        mapped_motors[hdm->id - 1] = hdm;
    }
    CAN_SendMessage(hcan,
                            &(CAN_TxHeaderTypeDef){
                                .StdId = 0x100+hdm->id,
                                .IDE   = CAN_ID_STD,
                                .RTR   = CAN_RTR_DATA,
                                .DLC   = 8},
                            initdata);
}


/**
 * DM CAN 反馈数据解包
 * @param hdji DJI handle
 * @param data 反馈数据
 */
void DM_DataDecode(DM_t* hdm, const uint8_t data[8])
{
    float scale_pos = 2.0f*hdm->POS_MAX/65535.0f;  //读取到的浮点数是和16位位置数据成线性关系，计算k值
    float scale_vel = 2.0f*hdm->VEL_MAX/4095.0f;   //读取到的浮点数是和12位位置数据成线性关系，计算k值
    float scale_t = 2.0f*hdm->T_MAX/4095.0f;
    const float feedback_pos = scale_pos *(uint16_t)(data[1] << 8 |data[2])- hdm->POS_MAX;   //反馈位置数据
    const float feedback_vel = scale_vel *(uint16_t)(data[3] << 8 |data[4] >> 4)- hdm->VEL_MAX;  //反馈速度数据
    const float feedback_t = scale_t * (uint16_t)((data[4]&0x0F) << 8 |data[5]);  //反馈力矩数据
    hdm->feedback.vel = feedback_vel ;
    hdm->feedback.pos = feedback_pos ;
    hdm->feedback.T = feedback_t ;
    hdm->feedback.T_MOS = data[6] ;
    hdm->feedback.T_Rotor = data[7] ;
    hdm->feedback_count++ ;
    hdm->feedback.ERR = data[0]&0x0F ;
}


/**
 * @brief 位置控制，在顶层调用
 * 
 * @param hcan 想要控制的电机所在的can总线
 * @param id 想要控制的电机在相应can总线上的id
 */
void DM_POS_CONTROL(CAN_HandleTypeDef* hcan,uint8_t id)
{
    for (int i = 0; i < map_size; i++)
    {
        if (hcan->Instance == map[i].can)
        {
            uint8_t pos_data[8] = {};
            if(map[i].motors[id-1] == NULL)
            {
                DM_ERROR_HANDLER();
            }
            uint8_t *pbuf,*vbuf;
            pbuf=(uint8_t *)&map[i].motors[id-1]->setpos;
            vbuf=(uint8_t *)&map[i].motors[id-1]->setvel;
            pos_data[0] = *(pbuf);
            pos_data[1] = *(pbuf+1);
            pos_data[2] = *(pbuf+2);
            pos_data[3] = *(pbuf+3);
            pos_data[4] = *(vbuf);
            pos_data[5] = *(vbuf+1);
            pos_data[6] = *(vbuf+2);
            pos_data[7] = *(vbuf+3);

            CAN_SendMessage(hcan,
                            &(CAN_TxHeaderTypeDef){
                                .StdId = 0x100+id,
                                .IDE   = CAN_ID_STD,
                                .RTR   = CAN_RTR_DATA,
                                .DLC   = 8},
                            pos_data);
            
            
            return;
        }
    }
}

/**
 * @brief 速度控制，在顶层调用
 * 
 * @param hcan 想要控制的电机所在的can总线
 * @param id 想要控制的电机在相应can总线上的id
 */

void DM_VEL_CONTROL(CAN_HandleTypeDef* hcan,uint8_t id)
{
    for (int i = 0; i < map_size; i++)
    {
        if (hcan->Instance == map[i].can)
        {
            uint8_t pos_data[8] = {};
            if(map[i].motors[id-1] == NULL)
            {
                DM_ERROR_HANDLER();
            }
            uint8_t *vbuf;
            vbuf=(uint8_t *)&map[i].motors[id-1]->setvel;
            pos_data[0] = *(vbuf);
            pos_data[1] = *(vbuf+1);
            pos_data[2] = *(vbuf+2);
            pos_data[3] = *(vbuf+3);
            
            CAN_SendMessage(hcan,
                            &(CAN_TxHeaderTypeDef){
                                .StdId = 0x200+id,
                                .IDE   = CAN_ID_STD,
                                .RTR   = CAN_RTR_DATA,
                                .DLC   = 8},
                            pos_data);
                        
            return;
        }
    }
}

/**
 * @brief 错误处理
 * 
 */
void DM_ERROR_HANDLER()
{
    //错误处理
}

/**
 * CAN FIFO0 接收回调函数
 * @attention 必须*注册*回调函数或者在更高级的回调函数内调用此回调函数
 * @note 使用
 *          HAL_CAN_RegisterCallback(hcan, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, DJI_CAN_Fifo0ReceiveCallback);
 *       来注册回调函数
 * @param hcan
 */
void DM_CAN_Fifo0ReceiveCallback(CAN_HandleTypeDef* hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
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
    uint8_t data[8];
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
void DM_CAN_BaseReceiveCallback(const CAN_HandleTypeDef* hcan, const CAN_RxHeaderTypeDef* header, uint8_t data[])
{
    for (int i = 0; i < map_size; i++)
    {
        if (hcan->Instance == map[i].can)
        {
            DM_t* hdm = getDMHandle(map[i].motors, data , header);
            if (hdm != NULL)
                DM_DataDecode(hdm, data);
            return;
        }
    }
}









