/**
*******************************************************************************
* @file      :main_task.cpp (Gimbal)
* @brief     : 云台C板 - 主循环任务
* @history   :
* Version     Date            Author          Note
* V1.4.5      2025-11-15      Gemini          1. [BUG修复] 移除 'imu_datas' 的多重定义
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
// PID 参数 (KP, KI, KD, MaxOut, MaxIntegral)
#define PID_PITCH_PARAMS {30.0f, 0.0f, 0.5f, 5.0f, 1.0f} // Kp, Ki, Kd, max_out (T_ff), max_int

/* Private types -------------------------------------------------------------*/
typedef struct {
    float vx;       // m/s
    float vy;       // m/s
    float wz;       // rad/s
    uint8_t mode;   // RobotControlMode
    uint8_t _pad[3]; // 4字节对齐
} ChassisCommand_t;


/* Private variables ---------------------------------------------------------*/
// (由 system_user.hpp 声明, 由 V1.4.2 修正)
volatile uint32_t tick = 0;
const float kCtrlPeriod = 0.001f; // 1ms
volatile RobotControlMode g_control_mode = MODE_SAFETY_CALIB; 

// [BUG 修复] 移除 imu_datas 的重复定义
// ImuDatas_t imu_datas = {0}; // <--- 这一行 (V1.4.2 中的第54行) 被删除
// [BUG 修复结束]

// -- 遥控器 --
namespace remote_control = hello_world::devices::remote_control;
static const uint8_t kRxBufLen = remote_control::kRcRxDataLen;
static uint8_t rc_rx_buf[kRxBufLen];
remote_control::DT7 *rc_ptr = nullptr;

// -- 云台C板设备句柄 --
Joint_Motor_t g_pitch_motor = {0};
PidHandle g_pitch_pid = NULL;

// CAN 发送缓冲区
static ChassisCommand_t g_chassis_cmd = {0};
static uint8_t g_chassis_tx_buf[8] = {0};

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

void MainTask_Loop(void) {
  tick = tick + 1; // 修复 volatile 警告
  
  if(tick <= IMU_CALIB_TIME) {
      ImuCalibrate(); 
      if (tick == IMU_CALIB_TIME) {
          ImuFinalizeCalibration(); 
      }
      RunSafetyMode();
  }
  else {
      ImuUpdate();
      RobotTask();
  }

  SendGimbalCanCommands();
}

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
// C++ 内部实现 (此部分无逻辑变更)
// -----------------------------------------------------------------

static void RobotInit(void) {
  rc_ptr = new remote_control::DT7();
  ImuInit(); 

  PidParams pitch_pid_params = PID_PITCH_PARAMS;
  g_pitch_pid = Pid_Create(&pitch_pid_params);

  joint_motor_init(&g_pitch_motor, GIMBAL_PITCH_MOTOR_ID, MIT_MODE);
  
  HAL_Delay(100); 
  enable_motor_mode(&hcan2, g_pitch_motor.para.id, MIT_MODE);
  HAL_Delay(10);
  
  memset(&g_chassis_cmd, 0, sizeof(g_chassis_cmd));
  memset(g_chassis_tx_buf, 0, sizeof(g_chassis_tx_buf));
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
}

static void RunControlMode(void) {
  // 1. 控制 Pitch 轴
  float target_pitch = rc_ptr->rc_rv() * RC_MAX_PITCH_RAD; 
  float fdb_pitch = imu_datas.euler_vals[1]; // PITCH=1
  float t_ff = Pid_CalcAngle(g_pitch_pid, target_pitch, fdb_pitch);
  mit_ctrl(&hcan2, g_pitch_motor.para.id, 0.0f, 0.0f, 0.0f, 0.0f, t_ff);

  // 2. 准备底盘指令
  g_chassis_cmd.vx = rc_ptr->rc_lv() * RC_MAX_SPEED_X;
  g_chassis_cmd.vy = -rc_ptr->rc_lh() * RC_MAX_SPEED_Y; 
  g_chassis_cmd.wz = -rc_ptr->rc_rh() * RC_MAX_SPEED_WZ;
  g_chassis_cmd.mode = (uint8_t)g_control_mode;
}


static void SetAllMotorsZero(void) {
  mit_ctrl(&hcan2, g_pitch_motor.para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  if(g_pitch_pid) Pid_Reset(g_pitch_pid);
}

static void SendGimbalCanCommands(void) {
  // (ID 0x300: [vx_f] [vy_f])
  memcpy(g_chassis_tx_buf,     &g_chassis_cmd.vx, sizeof(float));
  memcpy(g_chassis_tx_buf + 4, &g_chassis_cmd.vy, sizeof(float));
  CAN_Send_Msg(&hcan1, g_chassis_tx_buf, CAN_ID_TX_GIMBAL_TO_CHASSIS, 8);
  
  // (ID 0x301: [wz_f] [mode_u8] [pad*3])
  memcpy(g_chassis_tx_buf,     &g_chassis_cmd.wz, sizeof(float));
  memcpy(g_chassis_tx_buf + 4, &g_chassis_cmd.mode, sizeof(uint8_t));
  CAN_Send_Msg(&hcan1, g_chassis_tx_buf, CAN_ID_TX_GIMBAL_TO_CHASSIS + 1, 8);
}