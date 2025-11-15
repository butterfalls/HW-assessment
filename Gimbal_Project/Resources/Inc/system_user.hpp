/**
*******************************************************************************
* @file      ：system_user.hpp (Gimbal)
* @brief     : 云台C板 - 全局定义和配置
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

// ------------------- 云台电机 ID (舵轮拓扑图 P6) --------------------
#define GIMBAL_PITCH_MOTOR_ID 2 // DM-J4310 (ID 2, CAN2)
// (Yaw 电机在底盘板上)

// ------------------- 板间通信 CAN ID (CAN1) --------------------
// 云台 -> 底盘: 发送遥控器指令
#define CAN_ID_TX_GIMBAL_TO_CHASSIS 0x300

// 遥控器通道最大缩放值
#define RC_MAX_SPEED_X 1.0f // m/s
#define RC_MAX_SPEED_Y 1.0f // m/s
#define RC_MAX_SPEED_WZ (M_PI / 2.0f) // rad/s
#define RC_MAX_PITCH_RAD (M_PI / 6.0f) // 假设 Pitch 范围 30 度

/* Exported constants --------------------------------------------------------*/
extern const float kCtrlPeriod; // = 0.001f (1ms)

/* Exported types ------------------------------------------------------------*/

// 机器人控制模式 (云台只负责转发)
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
extern uint32_t tick;
extern ImuDatas_t imu_datas;


#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __SYSTEM_USER_HPP__ */