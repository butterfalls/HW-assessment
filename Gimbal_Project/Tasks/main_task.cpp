/**
*******************************************************************************
* @file      :main_task.cpp (Gimbal)
* @brief     : 云台C板 - 主循环任务 (V1.5.0 绝对姿态)
* @note      : [BUG 修复] 修复了 V1.4.x 的所有链接器错误
*******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "main_task.hpp"
#include "system_user.hpp"

// 外设
#include "DT7.hpp"
#include "HW_can.hpp"
#include "iwdg.h"
#include "imu_task.hpp" // 云台板自己的IMU

// 算法和驱动
#include "pid.hpp"
#include "dm4310_drv.hpp" // 云台电机

#include <math.h>
#include <string.h> // for memset

/* Private macro -------------------------------------------------------------*/
// Pitch 轴 (DM4310) - 角度环 (rad) (输出 T_ff 力矩)
#define PID_PITCH_PARAMS {30.0f, 0.0f, 0.5f, 5.0f, 1.0f} // Kp, Ki, Kd, max_out, max_int

/* Private types -------------------------------------------------------------*/
// [V1.5.0] 用于打包发送给底盘的遥控器数据
typedef struct {
    float vx;       // m/s
    float vy;       // m/s
    uint8_t mode;   // RobotControlMode
} ChassisCmd_t;


/* Private variables ---------------------------------------------------------*/
// [BUG 修复] volatile 变量定义在唯一的 .cpp 文件中
volatile uint32_t tick = 0;
const float kCtrlPeriod = 0.001f; // 1ms
volatile RobotControlMode g_control_mode = MODE_SAFETY_CALIB; 
// [BUG 修复] imu_datas 定义移至 imu_task.cpp

// -- 遥控器 --
namespace remote_control = hello_world::devices::remote_control;
static const uint8_t kRxBufLen = remote_control::kRcRxDataLen;
static uint8_t rc_rx_buf[kRxBufLen];
remote_control::DT7 *rc_ptr = nullptr;

// -- 云台C板设备句柄 --
Joint_Motor_t g_pitch_motor = {0};
PidHandle g_pitch_pid = NULL;

// -- [V1.5.0] 云台控制目标 --
float g_target_gimbal_yaw_rad = 0.0f; // 绝对 Yaw 目标 (积分产生)
float g_target_pitch_rad = 0.0f;      // 绝对 Pitch 目标 (摇杆给定)

// CAN 发送缓冲区
static ChassisCmd_t g_chassis_cmd = {0};
static uint8_t g_tx_buf[8] = {0}; // 通用发送缓冲

/* Private function prototypes -----------------------------------------------*/
static void RobotInit(void);
static void RobotTask(void);
static void UpdateControlMode(void);
static void RunSafetyMode(void);
static void RunEStopMode(void);
static void RunControlMode(void);
static void SetAllMotorsZero(void);
static void SendGimbalCanCommands(void);


// -----------------------------------------------------------------
// C/C++ 桥接函数 (供 main.c 调用)
// -----------------------------------------------------------------

void MainInit(void) {
  RobotInit();
  CanFilter_Init(&hcan1);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  CanFilter_Init(&hcan2);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rc_rx_buf, kRxBufLen);
  HAL_TIM_Base_Start_IT(&htim6);
}

/**
 * @brief [BUG 修复] 重命名为 Loop, 此函数由 C 的 HAL_TIM_PeriodElapsedCallback 调用
 */
void MainTask_Loop(void) {
  tick = tick + 1; // 修复 volatile 警告
  
  if(tick <= IMU_CALIB_TIME) {
      ImuCalibrate(); 
      if (tick == IMU_CALIB_TIME) {
          ImuFinalizeCalibration(); 
          g_target_gimbal_yaw_rad = imu_datas.euler_vals[YAW];
      }
      RunSafetyMode();
  }
  else {
      ImuUpdate(); // 持续更新 imu_datas
      RobotTask();
  }

  SendGimbalCanCommands();
}

/**
 * @brief [BUG 修复] 移至此处, 此函数由 C 的 HAL_UARTEx_RxEventCallback 调用
 */
void MainTask_UART_Callback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == &huart3) {
    if (Size == kRxBufLen) {
      HAL_IWDG_Refresh(&hiwdg);
      if (rc_ptr != NULL) {
        rc_ptr->decode(rc_rx_buf);
      }
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rc_rx_buf, kRxBufLen);
  }
}


// -----------------------------------------------------------------
// C++ 内部实现
// -----------------------------------------------------------------

static void RobotInit(void) {
  rc_ptr = new remote_control::DT7();
  ImuInit(); // 初始化云台板自己的 IMU

  PidParams pitch_pid_params = PID_PITCH_PARAMS;
  g_pitch_pid = Pid_Create(&pitch_pid_params);

  joint_motor_init(&g_pitch_motor, GIMBAL_PITCH_MOTOR_ID, MIT_MODE);
  
  HAL_Delay(100); 
  enable_motor_mode(&hcan2, g_pitch_motor.para.id, MIT_MODE);
  HAL_Delay(10);
  
  memset(&g_chassis_cmd, 0, sizeof(g_chassis_cmd));
}


static void RobotTask(void) {
  UpdateControlMode();
  
  switch (g_control_mode) {
    case MODE_ESTOP:
      RunEStopMode();
      break;
    case MODE_SEPARATE:
    case MODE_FOLLOW:
    case MODE_SPIN:
      RunControlMode(); 
      break;
    default:
      RunEStopMode(); 
      break;
  }
}

static void UpdateControlMode(void) {
  if (rc_ptr == NULL) return;

  if (rc_ptr->rc_l_switch() == remote_control::kSwitchStateDown) {
    g_control_mode = MODE_ESTOP;
    return;
  }
  
  switch (rc_ptr->rc_r_switch()) {
    case remote_control::kSwitchStateUp:
      g_control_mode = MODE_SEPARATE;
      break;
    case remote_control::kSwitchStateMid:
      g_control_mode = MODE_FOLLOW;
      break;
    case remote_control::kSwitchStateDown:
      g_control_mode = MODE_SPIN;
      break;
    default:
      g_control_mode = MODE_SEPARATE;
      break;
  }
}

static void RunSafetyMode(void) {
  SetAllMotorsZero();
}

static void RunEStopMode(void) {
  SetAllMotorsZero();
  g_chassis_cmd = {0}; 
  g_chassis_cmd.mode = (uint8_t)MODE_ESTOP;
  g_target_gimbal_yaw_rad = imu_datas.euler_vals[YAW]; 
  g_target_pitch_rad = imu_datas.euler_vals[PITCH];
}

static void RunControlMode(void) {
  // 1. [V1.5.0] 计算 Yaw 目标 (积分)
  float yaw_speed_cmd = -rc_ptr->rc_rh() * RC_MAX_YAW_SPEED;
  
  if (g_control_mode != MODE_SPIN) {
    g_target_gimbal_yaw_rad += yaw_speed_cmd * kCtrlPeriod; // (rad/s) * (s)
  }
  // (如果是小陀螺, g_target_gimbal_yaw_rad 保持不变)

  // 2. [V1.5.0] 计算 Pitch 目标 (角度)
  g_target_pitch_rad = rc_ptr->rc_rv() * RC_MAX_PITCH_RAD; 
  
  // 3. 运行 Pitch PID
  float fdb_pitch = imu_datas.euler_vals[PITCH];
  float t_ff = Pid_CalcAngle(g_pitch_pid, g_target_pitch_rad, fdb_pitch);
  mit_ctrl(&hcan2, g_pitch_motor.para.id, 0.0f, 0.0f, 0.0f, 0.0f, t_ff);

  // 4. [V1.5.0] 准备底盘平移指令
  g_chassis_cmd.vx = rc_ptr->rc_lv() * RC_MAX_SPEED_X;
  g_chassis_cmd.vy = -rc_ptr->rc_lh() * RC_MAX_SPEED_Y; 
  g_chassis_cmd.mode = (uint8_t)g_control_mode;
}


static void SetAllMotorsZero(void) {
  mit_ctrl(&hcan2, g_pitch_motor.para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  if(g_pitch_pid) Pid_Reset(g_pitch_pid);
}

static void SendGimbalCanCommands(void) {
  // [V1.5.0] 打包并发送所有4帧数据

  // 1. (ID 0x300: [vx_f] [vy_f])
  memcpy(g_tx_buf,     &g_chassis_cmd.vx, sizeof(float));
  memcpy(g_tx_buf + 4, &g_chassis_cmd.vy, sizeof(float));
  CAN_Send_Msg(&hcan1, g_tx_buf, CAN_ID_TX_CMD_1, 8);
  
  // 2. (ID 0x301: [mode_u8])
  memcpy(g_tx_buf, &g_chassis_cmd.mode, sizeof(uint8_t));
  CAN_Send_Msg(&hcan1, g_tx_buf, CAN_ID_TX_CMD_2, 1);
  
  // 3. (ID 0x302: [target_yaw_f])
  memcpy(g_tx_buf, &g_target_gimbal_yaw_rad, sizeof(float));
  CAN_Send_Msg(&hcan1, g_tx_buf, CAN_ID_TX_YAW_TARGET, 4);

  // 4. (ID 0x303: [current_yaw_f])
  float current_yaw = imu_datas.euler_vals[YAW];
  memcpy(g_tx_buf, &current_yaw, sizeof(float));
  CAN_Send_Msg(&hcan1, g_tx_buf, CAN_ID_TX_YAW_CURRENT, 4);
}