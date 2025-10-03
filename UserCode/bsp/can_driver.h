/**
 * @file    can.h
 * @author  syhanjin
 * @date    2025-09-04
 * @brief   CAN wrapper based on HAL library
 *
 * 本驱动是对 HAL 库的一层简要封装
 */
#ifndef CAN_H
#define CAN_H

#include "main.h"

#define CAN_ERROR_HANDLER() Error_Handler()
#define CAN_SEND_FAILED     (0xFFFF)
#define CAN_SEND_TIMEOUT    (10)

#ifdef __cplusplus
extern "C" {
#endif
#define CAN_NUM             (2)

typedef void (*CAN_FifoReceiveCallback_t)(CAN_HandleTypeDef* hcan, CAN_RxHeaderTypeDef* header, uint8_t data[]);

typedef struct
{
    CAN_HandleTypeDef* hcan;
    CAN_FifoReceiveCallback_t callbacks[28];
} CAN_CallbackMap;

// TODO: 增加更完善的错误返回逻辑


uint32_t CAN_SendMessage(CAN_HandleTypeDef* hcan, const CAN_TxHeaderTypeDef* header, const uint8_t data[]);
void CAN_Start(CAN_HandleTypeDef* hcan, uint32_t ActiveITs);

void CAN_RegisterCallback(CAN_HandleTypeDef* hcan, uint32_t filter_bank, CAN_FifoReceiveCallback_t callback);
void CAN_UnregisterCallback(CAN_HandleTypeDef* hcan, uint32_t filter_bank);
void CAN_Fifo0ReceiveCallback(CAN_HandleTypeDef* hcan);
void CAN_Fifo1ReceiveCallback(CAN_HandleTypeDef* hcan);

#ifdef __cplusplus
}
#endif
#endif // CAN_H
