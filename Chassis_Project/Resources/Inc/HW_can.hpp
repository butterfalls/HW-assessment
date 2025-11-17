/**
*******************************************************************************
* @file      :HW_can.hpp (Chassis)
* @brief     : 底盘C板 - CAN 驱动头文件 (V1.5.0)
*******************************************************************************
*/
#ifndef _HW_CAN_H_
#define _HW_CAN_H_
/* ------------------------------ Include ------------------------------ */
#include "stm32f407xx.h"
#include "stm32f4xx.h"
#include "system_user.hpp"
#include "can.h"

/* ------------------------------ Function Declaration --------------------- */
#ifdef __cplusplus
extern "C" {
#endif

void CanFilter_Init(CAN_HandleTypeDef *hcan);

void CAN_Send_Msg(CAN_HandleTypeDef *hcan, uint8_t *msg, uint32_t id,
                  uint8_t len);

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);

#ifdef __cplusplus
}
#endif

#endif