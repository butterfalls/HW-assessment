/**
*******************************************************************************
* @file      :main_task.cpp (Chassis)
* @brief     : 底盘C板 - 主循环任务
* @history   :
* Version     Date            Author          Note
* V1.4.5      2025-11-15      Gemini          1. [BUG修复] 修复 volatile 警告/定义
*******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "main_task.hpp"
#include "system_user.hpp"

// 外设
#include "HW_can.hpp"
#include "iwdg.h"
#include "imu_task.hpp" // 底盘板自己的IMU

// 算法和驱动
#include "pid.hpp"
#include "gm6020.hpp"
#include "m3508.hpp"
#include "dm4310_drv.hpp" // Yaw 电机
#include "swerve_drive.hpp"

#include <math.h>
#include <string.h> // for memset

/* Private macro -------------------------------------------------------------*/
// PID
#define PID_STEER_PARAMS {8000.0f, 0.0f, 100.0f, 25000.0f, 5000.0f}
#define PID_WHEEL_PARAMS {1000.0f, 50.0f, 0.0f, 16384.0f, 8000.0f}
#define PID_FOLLOW_PARAMS {5.0f, 0.0f, 0.1f, RC_MAX_SPEED_WZ * 1.5f, 1.0f}
#define PID_CHASSIS_YAW_PARAMS {20.0f, 0.0f, 0.2f, 5.0f, 1.0f}

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// [BUG 修复] 确保定义与 .hpp 中的 extern 声明完全匹配
volatile uint32_t tick = 0;
const float kCtrlPeriod = 0.001f; // 1ms
volatile RobotControlMode g_control_mode = MODE_SAFETY_CALIB; 
volatile float g_rc_cmd_vx = 0.0f;
volatile float g_rc_cmd_vy = 0.0f;
volatile float g_rc_cmd_wz = 0.0f;
volatile float g_gimbal_yaw_rad = 0.0f; 
// [BUG 修复结束]

// -- 底盘C板设备句柄 --
GM6020Handle g_steer_motors[4] = {NULL, NULL, NULL, NULL};
M3508Handle g_wheel_motors[4] = {NULL, NULL, NULL, NULL};
PidHandle g_steer_pids[4] = {NULL, NULL, NULL, NULL};
PidHandle g_wheel_pids[4] = {NULL, NULL, NULL, NULL};
PidHandle g_follow_pid = NULL; 
PidHandle g_chassis_yaw_pid = NULL; 
SwerveDriveHandle g_swerve_drive = NULL;
Joint_Motor_t g_yaw_motor = {0}; 

// CAN 发送缓冲区
static uint8_t g_m3508_tx_buf[8] = {0}; // 0x200 (CAN2)
static uint8_t g_gm6020_tx_buf[8] = {0}; // 0x1FF (CAN1)

/* Private function prototypes -----------------------------------------------*/
static void RobotInit(void);
static void RobotTask(void);
static void RunSafetyMode(void);
static void RunEStopMode(void);
static void RunSeparateMode(void);
static void RunFollowMode(void);
static void RunSpinMode(void);
static void SetAllMotorsZero(void);
static void SendChassisCanCommands(void);
static void RunSwerveControl(float vx, float vy, float wz);
static void ControlChassisYaw(void); 

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
  
  HAL_TIM_Base_Start_IT(&htim6);
}

void MainTask_Loop(void) {
  // [BUG 修复] 修复 C++20 volatile 警告
  tick = tick + 1;
  
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
  
  SendChassisCanCommands();
}

// -----------------------------------------------------------------
// C++ 内部实现 (此部分无逻辑变更)
// -----------------------------------------------------------------

static void RobotInit(void) {
  ImuInit(); 

  PidParams steer_pid_params = PID_STEER_PARAMS;
  PidParams wheel_pid_params = PID_WHEEL_PARAMS;
  PidParams follow_pid_params = PID_FOLLOW_PARAMS;
  PidParams yaw_pid_params = PID_CHASSIS_YAW_PARAMS; 

  g_follow_pid = Pid_Create(&follow_pid_params);
  g_chassis_yaw_pid = Pid_Create(&yaw_pid_params); 

  g_steer_motors[SWERVE_FR_MODULE] = GM6020_Create(GM6020_STEER_FR_ID);
  g_steer_motors[SWERVE_FL_MODULE] = GM6020_Create(GM6020_STEER_FL_ID);
  g_steer_motors[SWERVE_BL_MODULE] = GM6020_Create(GM6020_STEER_BL_ID);
  g_steer_motors[SWERVE_BR_MODULE] = GM6020_Create(GM6020_STEER_BR_ID);
  g_wheel_motors[SWERVE_FR_MODULE] = M3508_Create(M3508_WHEEL_FR_ID);
  g_wheel_motors[SWERVE_FL_MODULE] = M3508_Create(M3508_WHEEL_FL_ID);
  g_wheel_motors[SWERVE_BL_MODULE] = M3508_Create(M3508_WHEEL_BL_ID);
  g_wheel_motors[SWERVE_BR_MODULE] = M3508_Create(M3508_WHEEL_BR_ID);

  for (int i = 0; i < 4; ++i) {
      g_steer_pids[i] = Pid_Create(&steer_pid_params);
      g_wheel_pids[i] = Pid_Create(&wheel_pid_params);
  }
  
  joint_motor_init(&g_yaw_motor, CHASSIS_YAW_MOTOR_ID, MIT_MODE);

  g_swerve_drive = SwerveDrive_Create(
      SWERVE_WHEELBASE_X / 2.0f,
      SWERVE_WHEELBASE_Y / 2.0f,
      SWERVE_WHEEL_RADIUS
  );

  HAL_Delay(100); 
  enable_motor_mode(&hcan2, g_yaw_motor.para.id, MIT_MODE);
  HAL_Delay(10);

  memset(g_m3508_tx_buf, 0, sizeof(g_m3508_tx_buf));
  memset(g_gm6020_tx_buf, 0, sizeof(g_gm6020_tx_buf));
}


static void RobotTask(void) {
  ControlChassisYaw();

  switch (g_control_mode) {
    case MODE_ESTOP:
      RunEStopMode();
      break;
    case MODE_SEPARATE:
      RunSeparateMode();
      break;
    case MODE_FOLLOW:
      RunFollowMode();
      break;
    case MODE_SPIN:
      RunSpinMode();
      break;
    default:
      RunEStopMode(); 
      break;
  }
}

static void RunSafetyMode(void) {
  SetAllMotorsZero();
}

static void RunEStopMode(void) {
  SetAllMotorsZero();
}

static void RunSwerveControl(float vx, float vy, float wz)
{
  if (g_swerve_drive == NULL) return;
  
  float current_angles[4] = {
      GM6020_GetOutputAngleRad(g_steer_motors[SWERVE_FR_MODULE]),
      GM6020_GetOutputAngleRad(g_steer_motors[SWERVE_FL_MODULE]),
      GM6020_GetOutputAngleRad(g_steer_motors[SWERVE_BL_MODULE]),
      GM6020_GetOutputAngleRad(g_steer_motors[SWERVE_BR_MODULE])
  };

  SwerveDrive_Calculate(g_swerve_drive, vx, vy, wz, current_angles);

  for (int i = 0; i < 4; ++i) {
    SwerveModuleState target_state = SwerveDrive_GetModuleState(g_swerve_drive, i);
    
    float steer_fdb = GM6020_GetOutputAngleRad(g_steer_motors[i]);
    float steer_output = Pid_CalcAngle(g_steer_pids[i], target_state.angle_rad, steer_fdb);
    GM6020_SetInput(g_steer_motors[i], (int16_t)steer_output);

    float wheel_fdb = M3508_GetOutputVelRadS(g_wheel_motors[i]);
    float wheel_output = Pid_Calc(g_wheel_pids[i], target_state.speed_rads, wheel_fdb);
    M3508_SetInputCurrent(g_wheel_motors[i], (int16_t)wheel_output);
  }
}

static void RunSeparateMode(void) {
  RunSwerveControl(g_rc_cmd_vx, g_rc_cmd_vy, g_rc_cmd_wz);
}

static void RunFollowMode(void) {
  float wz_follow = Pid_CalcAngle(g_follow_pid, 0.0f, g_gimbal_yaw_rad);
  float chassis_wz = g_rc_cmd_wz + wz_follow; 

  float cos_yaw = cosf(g_gimbal_yaw_rad);
  float sin_yaw = sinf(g_gimbal_yaw_rad);
  float chassis_vx = g_rc_cmd_vx * cos_yaw - g_rc_cmd_vy * sin_yaw;
  float chassis_vy = g_rc_cmd_vx * sin_yaw + g_rc_cmd_vy * cos_yaw;

  RunSwerveControl(chassis_vx, chassis_vy, chassis_wz);
}

static void RunSpinMode(void) {
  float chassis_wz = SPIN_MODE_W_SPEED;

  float cos_yaw = cosf(g_gimbal_yaw_rad);
  float sin_yaw = sinf(g_gimbal_yaw_rad);
  float chassis_vx = g_rc_cmd_vx * cos_yaw - g_rc_cmd_vy * sin_yaw;
  float chassis_vy = g_rc_cmd_vx * sin_yaw + g_rc_cmd_vy * cos_yaw;

  RunSwerveControl(chassis_vx, chassis_vy, chassis_wz);
}

static void ControlChassisYaw(void)
{
    if (g_chassis_yaw_pid == NULL) return;
    float t_ff = Pid_CalcAngle(g_chassis_yaw_pid, 0.0f, g_gimbal_yaw_rad);
    mit_ctrl(&hcan2, g_yaw_motor.para.id, 0.0f, 0.0f, 0.0f, 0.0f, t_ff);
}


static void SetAllMotorsZero(void) {
  for (int i = 0; i < 4; ++i) {
      if(g_steer_motors[i]) GM6020_SetInput(g_steer_motors[i], 0);
      if(g_wheel_motors[i]) M3508_SetInputCurrent(g_wheel_motors[i], 0);
  }
  mit_ctrl(&hcan2, g_yaw_motor.para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  
  for (int i = 0; i < 4; ++i) {
      if(g_steer_pids[i]) Pid_Reset(g_steer_pids[i]);
      if(g_wheel_pids[i]) Pid_Reset(g_wheel_pids[i]);
  }
  if(g_follow_pid) Pid_Reset(g_follow_pid); 
  if(g_chassis_yaw_pid) Pid_Reset(g_chassis_yaw_pid);
}

static void SendChassisCanCommands(void) {
  // 1. M3508 (CAN2)
  int16_t c1 = M3508_GetInputCurrent(g_wheel_motors[M3508_WHEEL_FR_ID - 1]);
  int16_t c2 = M3508_GetInputCurrent(g_wheel_motors[M3508_WHEEL_FL_ID - 1]);
  int16_t c3 = M3508_GetInputCurrent(g_wheel_motors[M3508_WHEEL_BL_ID - 1]);
  int16_t c4 = M3508_GetInputCurrent(g_wheel_motors[M3508_WHEEL_BR_ID - 1]);
  g_m3508_tx_buf[0] = (c1 >> 8); g_m3508_tx_buf[1] = (c1);
  g_m3508_tx_buf[2] = (c2 >> 8); g_m3508_tx_buf[3] = (c2);
  g_m3508_tx_buf[4] = (c3 >> 8); g_m3508_tx_buf[5] = (c3);
  g_m3508_tx_buf[6] = (c4 >> 8); g_m3508_tx_buf[7] = (c4);
  CAN_Send_Msg(&hcan2, g_m3508_tx_buf, 0x200, 8);

  // 2. GM6020 (CAN1)
  int16_t v1 = GM6020_GetInput(g_steer_motors[GM6020_STEER_FR_ID - 1]);
  int16_t v2 = GM6020_GetInput(g_steer_motors[GM6020_STEER_FL_ID - 1]);
  int16_t v3 = GM6020_GetInput(g_steer_motors[GM6020_STEER_BL_ID - 1]);
  int16_t v4 = GM6020_GetInput(g_steer_motors[GM6020_STEER_BR_ID - 1]);
  g_gm6020_tx_buf[0] = (v1 >> 8); g_gm6020_tx_buf[1] = (v1);
  g_gm6020_tx_buf[2] = (v2 >> 8); g_gm6020_tx_buf[3] = (v2);
  g_gm6020_tx_buf[4] = (v3 >> 8); g_gm6020_tx_buf[5] = (v3);
  g_gm6020_tx_buf[6] = (v4 >> 8); g_gm6020_tx_buf[7] = (v4);
  CAN_Send_Msg(&hcan1, g_gm6020_tx_buf, 0x1FF, 8);
  
  // 3. DM4310 Yaw 电机 (CAN2)
  // (已在 ControlChassisYaw() 和 SetAllMotorsZero() 中通过 mit_ctrl() 发送)
}