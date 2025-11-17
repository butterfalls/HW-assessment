/**
*******************************************************************************
* @file      ：system_user.hpp (Chassis)
* @brief     : 底盘C板 - 全局定义和配置 (V1.5.0 绝对姿态)
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

// ------------------- 考核说明中的机器人几何参数 (舵轮组) --------------------
#define SWERVE_WHEELBASE_X (0.384f)
#define SWERVE_WHEELBASE_Y (0.310f)
#define SWERVE_WHEEL_RADIUS (0.104f / 2.0f)
#define SWERVE_WHEEL_GEAR_RATIO (14.0f)

// 舵轮索引
#define SWERVE_FR_MODULE 0 // 前右
#define SWERVE_FL_MODULE 1 // 前左
#define SWERVE_BL_MODULE 2 // 后左
#define SWERVE_BR_MODULE 3 // 后右

// ------------------- 底盘电机 ID (舵轮拓扑图 P6) --------------------
// [V1.5.0] 拓扑修正: CAN1 = GM6020, CAN2 = M3508 + DM4310(Yaw)
// CAN1:
#define GM6020_STEER_FR_ID 1
#define GM6020_STEER_FL_ID 2
#define GM6020_STEER_BL_ID 3
#define GM6020_STEER_BR_ID 4
// CAN2:
#define M3508_WHEEL_FR_ID 1
#define M3508_WHEEL_FL_ID 2
#define M3508_WHEEL_BL_ID 3
#define M3508_WHEEL_BR_ID 4
#define CHASSIS_YAW_MOTOR_ID 1 // DM-J4310 (ID 1, CAN2)


// ------------------- [V1.5.0] 板间通信 CAN ID (CAN1) --------------------
#define CAN_ID_RX_CMD_1 0x300 // (接收 [vx][vy])
#define CAN_ID_RX_CMD_2 0x301 // (接收 [mode])
#define CAN_ID_RX_YAW_TARGET 0x302 // (接收 [g_target_gimbal_yaw_rad])
#define CAN_ID_RX_YAW_CURRENT 0x303 // (接收 [g_current_gimbal_yaw_rad])

// [V1.5.0] 小陀螺模式旋转速度 (rad/s)
#define SPIN_MODE_W_SPEED (M_PI * 2.0f) // 1 r/s

/* Exported constants --------------------------------------------------------*/
extern const float kCtrlPeriod; // = 0.001f (1ms)

/* Exported types ------------------------------------------------------------*/

// 机器人控制模式 (从 CAN 接收)
typedef enum {
    MODE_SAFETY_CALIB, 
    MODE_ESTOP,        
    MODE_SEPARATE,     
    MODE_FOLLOW,       
    MODE_SPIN          
} RobotControlMode;


typedef struct _imu_datas_t
{
    float euler_vals[3]; // [roll, pitch, yaw]
    float gyro_vals[3];  // (rad/s)
    float acc_vals[3];   // (m/s^2)
} ImuDatas_t;

/* Exported variables --------------------------------------------------------*/
// [BUG 修复] 必须声明为 volatile
extern volatile uint32_t tick;
extern ImuDatas_t imu_datas; // 底盘板自己的IMU数据

// [V1.5.0] 由 CAN 中断填充, 必须声明为 volatile
extern volatile float g_rc_cmd_vx;
extern volatile float g_rc_cmd_vy;
extern volatile RobotControlMode g_control_mode;
extern volatile float g_target_gimbal_yaw_rad; // 云台的 *目标* 绝对 Yaw
extern volatile float g_current_gimbal_yaw_rad; // 云台的 *当前* 绝对 Yaw


#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __SYSTEM_USER_HPP__ */