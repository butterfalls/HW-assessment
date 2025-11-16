/**
*******************************************************************************
* @file      :main_task.cpp (Chassis)
* @brief     : 底盘C板 - 主循环任务 (V1.5.0 绝对姿态)
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
// PID 参数 (KP, KI, KD, MaxOut, MaxIntegral)
// **注意**: 必须在机器人上进行实际整定!
// 1. 航向电机 (GM6020) - 角度环 (电流模式)
#define PID_STEER_PARAMS {1000.0f, 0.0f, 10.0f, 16384.0f, 5000.0f}
// 2. 轮速电机 (M3508) - 速度环 (rad/s)
#define PID_WHEEL_PARAMS {1000.0f, 50.0f, 0.0f, 16384.0f, 8000.0f}
// 3. 底盘跟随PID (输出 wz, rad/s)
#define PID_FOLLOW_PARAMS {5.0f, 0.0f, 0.1f, M_PI * 2.0f, 1.0f}
// 4. 云台Yaw电机PID (输出 rad/s)
#define PID_CHASSIS_YAW_PARAMS {20.0f, 0.0f, 0.2f, 21.0f, 5.0f} // MaxOut (21 rad/s = DM4310 V_MAX)

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint32_t tick = 0;
const float kCtrlPeriod = 0.001f; // 1ms

// -- [V1.5.0] 由 CAN 回调 (HW_can.cpp) 填充
volatile RobotControlMode g_control_mode = MODE_SAFETY_CALIB; 
volatile float g_rc_cmd_vx = 0.0f;
volatile float g_rc_cmd_vy = 0.0f;
volatile float g_target_gimbal_yaw_rad = 0.0f; 
volatile float g_current_gimbal_yaw_rad = 0.0f; 

// -- 底盘C板设备句柄 --
GM6020Handle g_steer_motors[4] = {NULL, NULL, NULL, NULL};
M3508Handle g_wheel_motors[4] = {NULL, NULL, NULL, NULL};
PidHandle g_steer_pids[4] = {NULL, NULL, NULL, NULL};
PidHandle g_wheel_pids[4] = {NULL, NULL, NULL, NULL};
PidHandle g_follow_pid = NULL; 
PidHandle g_chassis_yaw_pid = NULL; // [V1.5.0] Yaw 电机 PID
SwerveDriveHandle g_swerve_drive = NULL;
Joint_Motor_t g_yaw_motor = {0}; 

// CAN 发送缓冲区
static uint8_t g_m3508_tx_buf[8] = {0}; // 0x200 (CAN2)
static uint8_t g_gm6020_tx_buf[8] = {0}; // 0x1FE (CAN1, 电流模式)

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
  tick = tick + 1;
  
  if(tick <= IMU_CALIB_TIME) {
      ImuCalibrate(); 
      if (tick == IMU_CALIB_TIME) {
          ImuFinalizeCalibration(); 
      }
      RunSafetyMode();
  }
  else {
      ImuUpdate(); // 获取底盘板自己的IMU数据
      RobotTask();
  }
  
  SendChassisCanCommands();
}

// -----------------------------------------------------------------
// C++ 内部实现
// -----------------------------------------------------------------

static void RobotInit(void) {
  ImuInit(); // 初始化底盘板自己的 IMU

  // 1. 初始化 PID
  PidParams steer_pid_params = PID_STEER_PARAMS;
  PidParams wheel_pid_params = PID_WHEEL_PARAMS;
  PidParams follow_pid_params = PID_FOLLOW_PARAMS;
  PidParams yaw_pid_params = PID_CHASSIS_YAW_PARAMS; // [V1.5.0]

  g_follow_pid = Pid_Create(&follow_pid_params);
  g_chassis_yaw_pid = Pid_Create(&yaw_pid_params); // [V1.5.0] Yaw 电机 PID (线性)

  // 2. 初始化8个舵轮电机
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
  
  // 3. 初始化 Yaw 电机 (DM4310, ID 1, CAN2)
  // [V1.5.0] 切换到速度模式
  joint_motor_init(&g_yaw_motor, CHASSIS_YAW_MOTOR_ID, SPEED_MODE);

  // 4. 初始化运动学解算器
  g_swerve_drive = SwerveDrive_Create(
      SWERVE_WHEELBASE_X / 2.0f,
      SWERVE_WHEELBASE_Y / 2.0f,
      SWERVE_WHEEL_RADIUS
  );

  // 5. 使能 Yaw 电机
  HAL_Delay(100); 
  // [V1.5.0] 使能速度模式
  enable_motor_mode(&hcan2, g_yaw_motor.para.id, SPEED_MODE);
  HAL_Delay(10);

  // 6. 清空 CAN 发送缓冲区
  memset(g_m3508_tx_buf, 0, sizeof(g_m3508_tx_buf));
  memset(g_gm6020_tx_buf, 0, sizeof(g_gm6020_tx_buf));
}


static void RobotTask(void) {
  // [V1.5.0] 无论什么模式, Yaw 轴电机都必须独立运行
  ControlChassisYaw();

  // 根据从云台 CAN 接收到的模式, 执行底盘逻辑
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

/**
 * @brief [V1.5.0] Yaw 电机控制 (绝对姿态)
 * 目标: 让云台的 *当前* 绝对角度 (来自云台IMU)
 * 追上云台的 *目标* 绝对角度 (来自遥控器积分)
 * @note  此函数处理360°无缝旋转
 */
static void ControlChassisYaw(void)
{
    if (g_chassis_yaw_pid == NULL) return;
    
    // 这是一个线性PID, 不是角度PID
    // 误差 = 目标 (来自CAN) - 反馈 (来自CAN)
    float yaw_error = g_target_gimbal_yaw_rad - g_current_gimbal_yaw_rad;
    
    // [V1.5.0] 考核要求: "在转动角度超过一圈的时候做越界的处理"
    // 我们使用线性PID (Pid_Calc) 而不是角度PID (Pid_CalcAngle)
    // 这样, 误差可以是 100 rad, PID 会持续输出直到误差消除, 完美实现无缝旋转。
    float yaw_speed_output = Pid_Calc(g_chassis_yaw_pid, 
                                     g_target_gimbal_yaw_rad, 
                                     g_current_gimbal_yaw_rad); 
    
    // 使用速度模式控制 Yaw 电机
    speed_ctrl(&hcan2, g_yaw_motor.para.id, yaw_speed_output);
}


/**
 * @brief [V1.5.0] 分离模式 (绝对姿态)
 * 底盘不旋转, 平移跟随云台坐标系
 */
static void RunSeparateMode(void) {
  // 1. 计算云台和底盘的绝对角度差
  // delta_yaw = (云台世界角度 - 底盘世界角度)
  float delta_yaw = g_current_gimbal_yaw_rad - imu_datas.euler_vals[YAW];

  // 2. 将云台坐标系下的平移指令 (vx, vy) 转换到底盘坐标系
  float cos_delta = cosf(delta_yaw);
  float sin_delta = sinf(delta_yaw);
  float chassis_vx = g_rc_cmd_vx * cos_delta - g_rc_cmd_vy * sin_delta;
  float chassis_vy = g_rc_cmd_vx * sin_delta + g_rc_cmd_vy * cos_delta;
  
  // 3. 底盘旋转速度为 0
  float chassis_wz = 0.0f;

  // 4. 运行舵轮解算
  RunSwerveControl(chassis_vx, chassis_vy, chassis_wz);
}

/**
 * @brief [V1.5.0] 跟随模式 (绝对姿态)
 * 底盘旋转跟随云台目标, 平移跟随云台坐标系
 */
static void RunFollowMode(void) {
  // 1. 计算云台和底盘的绝对角度差 (用于平移)
  float delta_yaw = g_current_gimbal_yaw_rad - imu_datas.euler_vals[YAW];
  float cos_delta = cosf(delta_yaw);
  float sin_delta = sinf(delta_yaw);

  // 2. 平移: 将云台坐标系指令转换到底盘坐标系
  float chassis_vx = g_rc_cmd_vx * cos_delta - g_rc_cmd_vy * sin_delta;
  float chassis_vy = g_rc_cmd_vx * sin_delta + g_rc_cmd_vy * cos_delta;
  
  // 3. 旋转: 底盘的目标是追上云台的目标
  //    使用PID让底盘的 *当前* 绝对角度 (底盘IMU) 追上云台的 *目标* 绝对角度 (来自CAN)
  float chassis_wz = Pid_CalcAngle(g_follow_pid, 
                                   g_target_gimbal_yaw_rad, // 目标 (来自CAN)
                                   imu_datas.euler_vals[YAW]); // 反馈 (底盘IMU)

  // 4. 运行舵轮解算
  RunSwerveControl(chassis_vx, chassis_vy, chassis_wz);
}

/**
 * @brief [V1.5.0] 小陀螺模式 (绝对姿态)
 * 底盘恒定旋转, 云台通过Yaw电机保持绝对角度不变
 */
static void RunSpinMode(void) {
  // 1. 计算云台和底盘的绝对角度差 (用于平移)
  float delta_yaw = g_current_gimbal_yaw_rad - imu_datas.euler_vals[YAW];
  float cos_delta = cosf(delta_yaw);
  float sin_delta = sinf(delta_yaw);

  // 2. 平移: 将云台坐标系指令转换到底盘坐标系
  float chassis_vx = g_rc_cmd_vx * cos_delta - g_rc_cmd_vy * sin_delta;
  float chassis_vy = g_rc_cmd_vx * sin_delta + g_rc_cmd_vy * cos_delta;
  
  // 3. 旋转: 底盘强制以恒定速度旋转
  float chassis_wz = SPIN_MODE_W_SPEED;

  // 4. 运行舵轮解算
  RunSwerveControl(chassis_vx, chassis_vy, chassis_wz);
  
  // 5. [重要] 云台 Yaw 电机 (ControlChassisYaw) 仍在独立运行
  //    此时云台板的 g_target_gimbal_yaw_rad 保持不变 (见Gimbal-main_task)
  //    底盘的 g_current_gimbal_yaw_rad 随云台IMU变化
  //    Yaw 电机 PID 会自动使云台在世界上保持静止
}


/**
 * @brief (V1.5.0) 运行舵轮解算和8个PID环
 */
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
    
    // 航向电机 (GM6020) - 角度环
    float steer_fdb = GM6020_GetOutputAngleRad(g_steer_motors[i]);
    float steer_output = Pid_CalcAngle(g_steer_pids[i], target_state.angle_rad, steer_fdb);
    GM6020_SetInputCurrent(g_steer_motors[i], (int16_t)steer_output); 

    // 轮速电机 (M3508) - 速度环
    float wheel_fdb = M3508_GetOutputVelRadS(g_wheel_motors[i]);
    float wheel_output = Pid_Calc(g_wheel_pids[i], target_state.speed_rads, wheel_fdb);
    M3508_SetInputCurrent(g_wheel_motors[i], (int16_t)wheel_output);
  }
}

/**
 * @brief (V1.5.0) 将所有电机目标值设为 0
 */
static void SetAllMotorsZero(void) {
  // 1. 8个舵轮电机
  for (int i = 0; i < 4; ++i) {
      if(g_steer_motors[i]) GM6020_SetInputCurrent(g_steer_motors[i], 0); 
      if(g_wheel_motors[i]) M3508_SetInputCurrent(g_wheel_motors[i], 0);
  }
  // 2. Yaw 电机
  speed_ctrl(&hcan2, g_yaw_motor.para.id, 0.0f);
  
  // 3. 重置所有 10 个 PID
  for (int i = 0; i < 4; ++i) {
      if(g_steer_pids[i]) Pid_Reset(g_steer_pids[i]);
      if(g_wheel_pids[i]) Pid_Reset(g_wheel_pids[i]);
  }
  if(g_follow_pid) Pid_Reset(g_follow_pid); 
  if(g_chassis_yaw_pid) Pid_Reset(g_chassis_yaw_pid);
}

/**
 * @brief (V1.5.0) 打包并发送舵轮 CAN 指令
 */
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
  int16_t v1 = GM6020_GetInputCurrent(g_steer_motors[GM6020_STEER_FR_ID - 1]); 
  int16_t v2 = GM6020_GetInputCurrent(g_steer_motors[GM6020_STEER_FL_ID - 1]);
  int16_t v3 = GM6020_GetInputCurrent(g_steer_motors[GM6020_STEER_BL_ID - 1]);
  int16_t v4 = GM6020_GetInputCurrent(g_steer_motors[GM6020_STEER_BR_ID - 1]);
  g_gm6020_tx_buf[0] = (v1 >> 8); g_gm6020_tx_buf[1] = (v1);
  g_gm6020_tx_buf[2] = (v2 >> 8); g_gm6020_tx_buf[3] = (v2);
  g_gm6020_tx_buf[4] = (v3 >> 8); g_gm6020_tx_buf[5] = (v3);
  g_gm6020_tx_buf[6] = (v4 >> 8); g_gm6020_tx_buf[7] = (v4);
  CAN_Send_Msg(&hcan1, g_gm6020_tx_buf, 0x1FE, 8); // [V1.5.0] 切换到电流帧 0x1FE
  
  // 3. DM4310 Yaw 电机 (CAN2)
  // (已在 ControlChassisYaw() 和 SetAllMotorsZero() 中通过 speed_ctrl() 发送)
}