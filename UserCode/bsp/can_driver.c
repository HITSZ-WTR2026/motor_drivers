/**
 * @file    can_driver.c
 * @author  syhanjin
 * @date    2025-09-04
 * @brief
 */
#include "can_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef USE_RTOS
#include "cmsis_os2.h"
static osMutexId_t can_mutex = NULL;
static osMutexId_t get_can_mutex()
{
    if (can_mutex == NULL)
        can_mutex = osMutexNew(&(osMutexAttr_t){.name = "can_mutex"});
    return can_mutex;
}
#else
#include "cmsis_compiler.h"
#endif


/**
 * 发送一条 CAN 消息
 * @param hcan can handle
 * @param header CAN_TxHeaderTypeDef
 * @param data 数据
 * @note 本身想做成内联展开，但是必须写到 .h 文件，调研发现性能损失不大，所以直接放到此处
 * @attention 本函数大部分情况是线程安全的，少数情况（中断被中断打断）会出现不安全的情况。
 * @return mailbox, 0xFFFF 表示发送失败
 */
uint32_t CAN_SendMessage(CAN_HandleTypeDef* hcan, const CAN_TxHeaderTypeDef header, uint8_t data[])
{
    uint32_t mailbox = CAN_SEND_FAILED;

    // 等待上一个发送完成
    // while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)
    //     ;
    if (__get_IPSR() != 0)
    {
        // 在中断中直接调用
        if (HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox) != HAL_OK)
        {
            CAN_ERROR_HANDLER();
        }
    }
    else
#ifdef USE_RTOS
    { // 任务中调用需要加临界保护
        if (osMutexAcquire(get_can_mutex(), CAN_SEND_TIMEOUT) != osOK)
            // 超时
            return CAN_SEND_FAILED;
        if (HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox) != HAL_OK)
        {
            CAN_ERROR_HANDLER();
        }
        osMutexRelease(get_can_mutex());
    }
#else
    {
        // 裸机状态下非中断调用需要保护
        __disable_irq();
        if (HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox) != HAL_OK)
        {
            CAN_ERROR_HANDLER();
        }
        __enable_irq();
    }
#endif

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

#ifdef __cplusplus
}
#endif
