/**
 * @file    can_driver.c
 * @author  syhanjin
 * @date    2025-09-04
 * @brief
 */
#include "can_driver.h"

static CAN_CallbackMap maps[CAN_NUM];
static size_t map_size = 0;

static CAN_FifoReceiveCallback_t* get_callbacks(const CAN_HandleTypeDef* hcan)
{
    for (int i = 0; i < map_size; i++)
        if (maps[i].hcan == hcan)
            return maps[i].callbacks;

    return NULL;
}

/**
 * 发送一条 CAN 消息
 * @param hcan can handle
 * @param header CAN_TxHeaderTypeDef
 * @param data 数据
 * @note 本身想做成内联展开，但是必须写到 .h 文件，调研发现性能损失不大，所以直接放到此处
 * @return mailbox
 */
uint32_t CAN_SendMessage(CAN_HandleTypeDef* hcan, const CAN_TxHeaderTypeDef header, uint8_t data[])
{
    uint32_t mailbox = 0xFFFF;

    // 等待上一个发送完成
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)
        ;

    if (HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox) != HAL_OK)
    {
        CAN_ERROR_HANDLER();
    }

    return mailbox;
}

/**
 * CAN 初始化
 * @param hcan can handle
 * @param ActiveITs CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING
 */
void CAN_Start(CAN_HandleTypeDef* hcan, const uint32_t ActiveITs)
{
    if (HAL_CAN_Start(hcan) != HAL_OK)
    {
        CAN_ERROR_HANDLER();
    }

    if (HAL_CAN_ActivateNotification(hcan, ActiveITs) != HAL_OK)
    {
        CAN_ERROR_HANDLER();
    }
}

/**
 * 注册 CAN Fifo 处理回调
 *
 * @note 重复注册将会覆盖之前的回调
 * @param hcan hcan
 * @param filter_bank 注册对应的 filter 编号
 * @param callback 回调函数指针
 */
void CAN_RegisterCallback(CAN_HandleTypeDef* hcan, const uint32_t filter_bank, CAN_FifoReceiveCallback_t callback)
{
    CAN_FifoReceiveCallback_t* callbacks = get_callbacks(hcan);

    if (callbacks == NULL)
    {
        maps[map_size] = (CAN_CallbackMap){
            .hcan      = hcan,
            .callbacks = {NULL}};
        callbacks = maps[map_size].callbacks;
        map_size++;
    }

    callbacks[filter_bank] = callback;
}

/**
 * 取消注册 CAN Fifo 处理回调
 * @param hcan can handle
 * @param filter_bank 需要取消注册对应的过滤器对应的 id
 */
void CAN_UnregisterCallback(CAN_HandleTypeDef* hcan, const uint32_t filter_bank)
{
    CAN_FifoReceiveCallback_t* callbacks = get_callbacks(hcan);
    if (callbacks != NULL)
        callbacks[filter_bank] = NULL;
}

/**
 * CAN Fifo0 接收处理函数
 *
 * 本函数将会根据 hcan 和 rx_header 内部的 filter_id 来调用对应的回调函数
 * @param hcan can handle
 */
void CAN_Fifo0ReceiveCallback(CAN_HandleTypeDef* hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
    {
        CAN_ERROR_HANDLER();
        return;
    }
    const CAN_FifoReceiveCallback_t* callbacks = get_callbacks(hcan);
    if (callbacks != NULL && callbacks[header.FilterMatchIndex] != NULL)
        callbacks[header.FilterMatchIndex](hcan, &header, data);
}

/**
 * CAN Fifo1 接收处理函数
 *
 * 本函数将会根据 hcan 和 rx_header 内部的 filter_id 来调用对应的回调函数
 * @param hcan can handle
 */
void CAN_Fifo1ReceiveCallback(CAN_HandleTypeDef* hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &header, data) != HAL_OK)
    {
        CAN_ERROR_HANDLER();
        return;
    }
    const CAN_FifoReceiveCallback_t* callbacks = get_callbacks(hcan);
    if (callbacks != NULL && callbacks[header.FilterMatchIndex] != NULL)
        callbacks[header.FilterMatchIndex](hcan, &header, data);
}
