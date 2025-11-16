/**
*******************************************************************************
* @file      ：main_task.hpp (Chassis)
* @brief     : 底盘C板 - 主任务头文件 (C/C++ 桥接)
*******************************************************************************
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_TASK_HPP__
#define __MAIN_TASK_HPP__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "main.h" 
#include "usart.h" // 包含 huart
#include "system_user.hpp"

/* Exported function prototypes ----------------------------------------------*/

extern volatile float g_rc_cmd_wz;
extern volatile RobotControlMode g_control_mode;
extern volatile float g_gimbal_target_yaw_rad; // 云台目标绝对角度
extern volatile float g_gimbal_current_yaw_rad; // 云台当前绝对角度
extern volatile float g_chassis_imu_yaw_rad; // 底盘IMU绝对角度
extern volatile float g_rc_cmd_vx; // 无人机遥控器X轴命令

void MainInit(void);
void MainTask_Loop(void);
void Set_Chassis_Mode(RobotControlMode mode);
RobotControlMode Get_Chassis_Mode(void);

// [修改] 底盘板不再需要 UART 回调
// void MainTask_UART_Callback(UART_HandleTypeDef *huart, uint16_t Size);


#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __MAIN_TASK_HPP__ */