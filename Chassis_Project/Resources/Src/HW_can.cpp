/**
 *******************************************************************************
 * @file      :HW_can.cpp (Chassis)
 * @brief     : 底盘C板 - CAN 驱动和回调 (V1.5.0)
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "HW_can.hpp"
#include "system_user.hpp" 
#include "stdint.h"
#include <string.h> // for memcpy

// 引入电机库 C 语言 API
#include "gm6020.hpp"
#include "m3508.hpp"
#include "dm4310_drv.hpp" 

/* Private variables ---------------------------------------------------------*/
static CAN_RxHeaderTypeDef rx_header1, rx_header2;
static uint8_t can1_rx_data[8], can2_rx_data[8];
uint32_t pTxMailbox; 

/* External variables --------------------------------------------------------*/
// 电机句柄 (from main_task.cpp)
extern M3508Handle g_wheel_motors[4]; 
extern GM6020Handle g_steer_motors[4]; 
extern Joint_Motor_t g_yaw_motor; 

// [V1.5.0] 全局变量
extern volatile float g_rc_cmd_vx;
extern volatile float g_rc_cmd_vy;
extern volatile RobotControlMode g_control_mode;
extern volatile float g_target_gimbal_yaw_rad; 
extern volatile float g_current_gimbal_yaw_rad;

/* Private function prototypes -----------------------------------------------*/
static void CAN_Rx_Decode_Chassis(CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data);
static void CAN_Rx_Decode_Interboard(CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data);

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
 * @brief   CAN1 FIFO0 中断回调 (板间通信 + 舵电机)
 * @note    [V1.5.0] 拓扑图: CAN1 = (上下通信) + (GM6020)
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (hcan == &hcan1) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header1, can1_rx_data) == HAL_OK) 
    {
      // 1. 解码来自云台的 RC 指令
      CAN_Rx_Decode_Interboard(&rx_header1, can1_rx_data);
      
      // 2. 解码 GM6020 舵电机的反馈
      CAN_Rx_Decode_Chassis(&rx_header1, can1_rx_data);
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING); 
}

/**
 * @brief   CAN2 FIFO1 中断回调 (底盘电机)
 * @note    [V1.5.0] 拓扑图: CAN2 = (M3508) + (Yaw DM4310)
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (hcan == &hcan2) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header2, can2_rx_data) == HAL_OK)
    {
      // 解码 M3508 轮电机 和 Yaw 电机
      CAN_Rx_Decode_Chassis(&rx_header2, can2_rx_data);
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}

/**
 * @brief   [V1.5.0] 板间通信解码 (CAN1)
 */
static void CAN_Rx_Decode_Interboard(CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data) {
  
  // 考核要求 #1: 在板间通信解包处喂狗
  if (rx_header->StdId == CAN_ID_RX_CMD_1 || 
      rx_header->StdId == CAN_ID_RX_CMD_2 ||
      rx_header->StdId == CAN_ID_RX_YAW_TARGET ||
      rx_header->StdId == CAN_ID_RX_YAW_CURRENT)
  {
      HAL_IWDG_Refresh(&hiwdg);
  }

  switch (rx_header->StdId)
  {
    case CAN_ID_RX_CMD_1: // 0x300 (来自云台的 [vx][vy])
      memcpy((void*)&g_rc_cmd_vx, rx_data,     sizeof(float));
      memcpy((void*)&g_rc_cmd_vy, rx_data + 4, sizeof(float));
      break;
      
    case CAN_ID_RX_CMD_2: // 0x301 (来自云台的 [mode])
      g_control_mode = (RobotControlMode)rx_data[0];
      break;

    case CAN_ID_RX_YAW_TARGET: // 0x302 (来自云台的 [target_yaw_f])
      memcpy((void*)&g_target_gimbal_yaw_rad, rx_data, sizeof(float));
      break;
    
    case CAN_ID_RX_YAW_CURRENT: // 0x303 (来自云台的 [current_yaw_f])
      memcpy((void*)&g_current_gimbal_yaw_rad, rx_data, sizeof(float));
      break;
  }
}


/**
 * @brief   底盘电机数据解码 (由 CAN1 和 CAN2 共同调用)
 */
static void CAN_Rx_Decode_Chassis(CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data) {
  uint32_t id = rx_header->StdId;

  switch (id) {
    // 1. M3508 轮电机反馈 (CAN2: 0x201 - 0x204)
    case 0x201: 
      M3508_Decode(g_wheel_motors[SWERVE_FR_MODULE], rx_data);
      break;
    case 0x202: 
      M3508_Decode(g_wheel_motors[SWERVE_FL_MODULE], rx_data);
      break;
    case 0x203: 
      M3508_Decode(g_wheel_motors[SWERVE_BL_MODULE], rx_data);
      break;
    case 0x204: 
      M3508_Decode(g_wheel_motors[SWERVE_BR_MODULE], rx_data);
      break;

    // 2. GM6020 航向电机反馈 (CAN1: 0x205 - 0x208)
    case 0x205: 
      GM6020_Decode(g_steer_motors[SWERVE_FR_MODULE], rx_data);
      break;
    case 0x206: 
      GM6020_Decode(g_steer_motors[SWERVE_FL_MODULE], rx_data);
      break;
    case 0x207: 
      GM6020_Decode(g_steer_motors[SWERVE_BL_MODULE], rx_data);
      break;
    case 0x208: 
      GM6020_Decode(g_steer_motors[SWERVE_BR_MODULE], rx_data);
      break;
      
    // 3. DM4310 Yaw 电机反馈 (CAN2: ID 0x01)
    case CHASSIS_YAW_MOTOR_ID:
      dm4310_fbdata(&g_yaw_motor, rx_data, rx_header->DLC);
      break;
      
    default:
      break;
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