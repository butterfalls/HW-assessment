/**
*******************************************************************************
* @file      :HW_can.hpp (Gimbal)
* @brief     : 云台C板 - CAN 驱动头文件 (V1.5.0)
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

// [BUG 修复] HAL_CAN 回调函数是 C 函数，必须在 C++ 头文件中声明
// 否则C++的 HW_can.cpp 定义它们时，C的 stm32f4xx_it.c 找不到
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);


#ifdef __cplusplus
}
#endif

#endif