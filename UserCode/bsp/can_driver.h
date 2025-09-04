/**
 * @file    can.h
 * @author  syhanjin
 * @date    2025-09-04
 * @brief   CAN wrapper based on HAL library
 */
#ifndef CAN_H
#define CAN_H

#include "main.h"

#define CAN_ERROR_HANDLER() Error_Handler()

uint32_t CAN_SendMessage(CAN_HandleTypeDef* hcan, CAN_TxHeaderTypeDef header, uint8_t data[]);
void CAN_Init(CAN_HandleTypeDef* hcan, uint32_t ActiveITs);

#endif // CAN_H
