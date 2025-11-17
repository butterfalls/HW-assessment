/**
 *******************************************************************************
 * @file      :HW_can.cpp (Gimbal)
 * @brief     : 云台C板 - CAN 驱动和回调 (V1.5.0)
 * @note      : [BUG 修复] 此文件现在是定义 CAN 函数的唯一位置
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "HW_can.hpp" // [BUG 修复] 必须包含对应的头文件
#include "system_user.hpp"
#include "stdint.h"
#include <string.h> // for memcpy

// 引入电机库 C 语言 API
#include "dm4310_drv.hpp" 

/* Private variables ---------------------------------------------------------*/
static CAN_RxHeaderTypeDef rx_header1, rx_header2;
static uint8_t can1_rx_data[8], can2_rx_data[8];
uint32_t pTxMailbox; // [BUG 修复] 全局变量定义

/* External variables --------------------------------------------------------*/
// 声明在 main_task.cpp 中定义的电机句柄
extern Joint_Motor_t g_pitch_motor; // 云台 Pitch 电机

/* Private function prototypes -----------------------------------------------*/
static void CAN_Rx_Decode_Gimbal(CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data);

/**
 * @brief CAN 过滤器初始化 (通用，接收所有)
 */
void CanFilter_Init(CAN_HandleTypeDef *hcan) {
  CAN_FilterTypeDef canfilter;
  canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilter.FilterIdHigh = 0x0000;
  canfilter.FilterIdLow = 0x0000;
  canfilter.FilterMaskIdHigh = 0x0000;
  canfilter.FilterMaskIdLow = 0x0000; 
  canfilter.FilterActivation = ENABLE;
  canfilter.SlaveStartFilterBank = 14; 

  if (hcan == &hcan1) {
    canfilter.FilterFIFOAssignment = CAN_FilterFIFO0;
    canfilter.FilterBank = 0;
    HAL_CAN_ConfigFilter(hcan, &canfilter);
  } else if (hcan == &hcan2) {
    canfilter.FilterFIFOAssignment = CAN_FilterFIFO1; // FIFO1
    canfilter.FilterBank = 14;
    HAL_CAN_ConfigFilter(hcan, &canfilter);
  }
}

/**
 * @brief   CAN1 FIFO0 中断回调 (板间通信)
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (hcan == &hcan1) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header1, can1_rx_data) == HAL_OK) 
    {
      // 可选：解码来自底盘的状态报文
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING); 
}

/**
 * @brief   CAN2 FIFO1 中断回调 (云台电机)
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (hcan == &hcan2) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header2, can2_rx_data) == HAL_OK)
    {
      // 解码云台电机的反馈
      CAN_Rx_Decode_Gimbal(&rx_header2, can2_rx_data);
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}

/**
 * @brief   云台电机数据解码 (CAN2)
 */
static void CAN_Rx_Decode_Gimbal(CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data) {
  // DM4310 反馈帧 ID = 0x00 + 电机 ID
  if (rx_header->StdId == g_pitch_motor.para.id) // ID 2
  {
      dm4310_fbdata(&g_pitch_motor, rx_data, rx_header->DLC);
  }
}

/**
 * @brief   向can总线发送数据
 */
void CAN_Send_Msg(CAN_HandleTypeDef *hcan, uint8_t *msg, uint32_t id,
                  uint8_t len) {
  CAN_TxHeaderTypeDef TxMessageHeader = {0};
  TxMessageHeader.StdId = id;
  TxMessageHeader.IDE = CAN_ID_STD;
  TxMessageHeader.RTR = CAN_RTR_DATA;
  TxMessageHeader.DLC = len;
  
  if (HAL_CAN_AddTxMessage(hcan, &TxMessageHeader, msg, &pTxMailbox) != HAL_OK) {
      // 发送失败
  }
}