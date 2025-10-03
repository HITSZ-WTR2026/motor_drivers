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

// TODO: 增加更完善的错误返回逻辑


uint32_t CAN_SendMessage(CAN_HandleTypeDef* hcan, CAN_TxHeaderTypeDef header, uint8_t data[]);
void CAN_Start(CAN_HandleTypeDef* hcan, uint32_t ActiveITs);


#ifdef __cplusplus
}
#endif
#endif // CAN_H
