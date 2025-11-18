/**
*******************************************************************************
* @file      ：system_user.hpp (Gimbal)
* @brief     : 云台C板 - 全局定义和配置 (V1.5.0 绝对姿态)
*******************************************************************************
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SYSTEM_USER_HPP__
#define __SYSTEM_USER_HPP__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "stm32f407xx.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "can.h"
#include "gpio.h"
#include "usart.h"
#include "iwdg.h"
#include "spi.h" 
#include <stdint.h>
#include <math.h>

/* Exported macro ------------------------------------------------------------*/
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// [V1.5.0] IMU 坐标
#define ROLL 0
#define PITCH 1
#define YAW 2

// ------------------- 云台电机 ID (舵轮拓扑图 P6) --------------------
#define GIMBAL_PITCH_MOTOR_ID 2 // DM-J4310 (ID 2, CAN2)

// ------------------- [V1.5.0] 板间通信 CAN ID (CAN1) --------------------
#define CAN_ID_TX_CMD_1 0x150 // (发送 [vx][vy])
#define CAN_ID_TX_YAW_TARGET 0x151 // (发送 [g_target_gimbal_yaw_rad])

// ------------------- [V1.5.0] 遥控器宏 --------------------
// 遥控器右摇杆水平 -> 云台 Yaw 轴目标角速度 (rad/s)
#define RC_MAX_YAW_SPEED (M_PI) // (180 deg/s)
// 遥控器右摇杆垂直 -> 云台 Pitch 轴目标角度 (rad)
#define RC_MAX_PITCH_RAD (M_PI / 6.0f) // (30 deg)
// 遥控器左摇杆 -> 底盘平移速度 (m/s)
#define RC_MAX_SPEED_X 1.0f 
#define RC_MAX_SPEED_Y 1.0f 

/* Exported constants --------------------------------------------------------*/
extern const float kCtrlPeriod; // = 0.001f (1ms)

/* Exported types ------------------------------------------------------------*/

// 机器人控制模式 (云台负责定义和转发)
typedef enum {
    MODE_SAFETY_CALIB, // 安全模式 (启动时校准IMU)
    MODE_ESTOP,        // 紧急停止模式 (左拨杆-下)
    MODE_SEPARATE,     // 分离模式 (右拨杆-上)
    MODE_FOLLOW,       // 跟随模式 (右拨杆-中)
    MODE_SPIN          // 小陀螺模式 (右拨杆-下)
} RobotControlMode;


typedef struct _imu_datas_t
{
    float euler_vals[3]; // 欧拉角度 [roll, pitch, yaw]
    float gyro_vals[3];  // 角速度 (rad/s)
    float acc_vals[3];   // 加速度 (m/s^2)
} ImuDatas_t;

/* Exported variables --------------------------------------------------------*/
// [BUG 修复] 必须声明为 volatile，因为它们在中断和主循环中使用
extern volatile uint32_t tick;
extern volatile RobotControlMode g_control_mode; 
// [BUG 修复] imu_datas 在 imu_task.cpp 中定义, 这里只声明
extern ImuDatas_t imu_datas; 


#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __SYSTEM_USER_HPP__ */