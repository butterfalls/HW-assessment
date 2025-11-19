/**
*******************************************************************************
* @file      :main_task.cpp (Gimbal)
* @brief     : 云台C板 - 主循环任务 (V1.6.0 绝对姿态与无缝旋转)
* @note      : [用户修正] 
* 1. 实现了 IMU 数据的累积处理，支持 360° 无缝旋转，防止过零回转。
* 2. 修正了云台控制逻辑，云台基于世界坐标系进行绝对姿态控制。
* 3. 底盘速度指令基于云台坐标系发送，坐标转换下放至底盘板处理。
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
#define PID_PITCH_PARAMS {8.0f, 0.0f, 0.8f, 10.0f, 2.0f} // Kp, Ki, Kd, max_out, max_int
#define PI 3.1415926535f

/* Private types -------------------------------------------------------------*/
// 用于打包发送给底盘的遥控器数据
typedef struct {
    float vx;       // m/s (云台坐标系)
    float vy;       // m/s (云台坐标系)
    uint8_t mode;   // RobotControlMode
} ChassisCmd_t;

/* Private variables ---------------------------------------------------------*/
volatile uint32_t tick = 0;
const float kCtrlPeriod = 0.001f; // 1ms
volatile RobotControlMode g_control_mode = MODE_SAFETY_CALIB; 
PidParams pitch_pid_params = PID_PITCH_PARAMS;
float pitch_fb;
float pitch_error;
float target_speed;
float real_motor_target;
// -- 遥控器 --
namespace remote_control = hello_world::devices::remote_control;
static const uint8_t kRxBufLen = remote_control::kRcRxDataLen;
static uint8_t rc_rx_buf[kRxBufLen];
remote_control::DT7 *rc_ptr = nullptr;

// -- 云台C板设备句柄 --
Joint_Motor_t g_pitch_motor = {0};
PidHandle g_pitch_pid = NULL;

// -- 云台控制目标与状态 (核心变量) --
float g_target_gimbal_yaw_rad = 0.0f;      // 绝对 Yaw 目标 (积分产生, 连续无界)
float g_current_gimbal_yaw_cumulative = 0.0f; // 当前绝对 Yaw (处理过零后的连续值)
float g_target_pitch_rad = 0.0f;           // 绝对 Pitch 目标 (摇杆给定)

// -- 累积角度计算辅助变量 --
static float last_raw_yaw = 0.0f;
static int32_t yaw_round_count = 0;

// CAN 发送缓冲区
static ChassisCmd_t g_chassis_cmd = {0};
static uint8_t g_tx_buf[8] = {0}; 

/* Private function prototypes -----------------------------------------------*/
static void RobotInit(void);
static void RobotTask(void);
static void UpdateControlMode(void);
static void RunSafetyMode(void);
static void RunEStopMode(void);
static void RunControlMode(void);
static void SetAllMotorsZero(void);
static void SendGimbalCanCommands(void);
// static void UpdateCumulativeYaw(void); // 新增：处理角度过零突变

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
 * @brief 主循环 (1kHz)
 */
void MainTask_Loop(void) {
  tick = tick +1;
  
  if(tick <= IMU_CALIB_TIME) {
      ImuCalibrate(); 
      if (tick == IMU_CALIB_TIME) {
          ImuFinalizeCalibration(); 
          // 校准完成后，初始化目标角度为当前角度，防止启动飞车
          last_raw_yaw = imu_datas.euler_vals[YAW];
          g_current_gimbal_yaw_cumulative = last_raw_yaw;
          g_target_gimbal_yaw_rad = g_current_gimbal_yaw_cumulative;
      }
      RunSafetyMode();
  }
  else {
      ImuUpdate(); // 更新 imu_datas
      // UpdateCumulativeYaw(); // 计算连续的累积角度
      g_current_gimbal_yaw_cumulative = imu_datas.euler_vals[YAW];
      RobotTask();
      SendGimbalCanCommands();
  }

  
}

/**
 * @brief 遥控器接收回调
 */
void MainTask_UART_Callback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == &huart3) {
    if (Size == kRxBufLen) {
      HAL_IWDG_Refresh(&hiwdg); // 仅在此处喂狗
      if (rc_ptr != NULL) {
        rc_ptr->decode(rc_rx_buf);
      }
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rc_rx_buf, kRxBufLen);
  }
}

// -----------------------------------------------------------------
// 核心功能实现
// -----------------------------------------------------------------

/**
 * @brief 处理 IMU 的 Yaw 轴角度突变，生成连续的累积角度
 * @note  解决 -PI 到 PI 的跳变问题，实现 "不回转" 逻辑
 */
// static void UpdateCumulativeYaw(void) {
//     float current_raw_yaw = imu_datas.euler_vals[YAW];
//     float delta = current_raw_yaw - last_raw_yaw;

//     // 检测过零突变 (阈值设为 1.5 PI，防止噪声误判)
//     if (delta < -1.5f * PI) {
//         yaw_round_count++; // 正向跨越 (179 -> -179)
//     } else if (delta > 1.5f * PI) {
//         yaw_round_count--; // 负向跨越 (-179 -> 179)
//     }

//     // 计算累积角度：原始角度 + 圈数 * 2PI
//     g_current_gimbal_yaw_cumulative = current_raw_yaw + yaw_round_count * 2.0f * PI;
    
//     last_raw_yaw = current_raw_yaw;
// }

static void RobotInit(void) {
  rc_ptr = new remote_control::DT7();
  ImuInit(); 

  g_pitch_pid = Pid_Create(&pitch_pid_params);

  // Pitch 轴使用 DM4310 (MIT模式)
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

  // 左拨杆下拨 = 急停 (最高优先级)
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
  // 急停时，目标角度跟随当前角度，防止恢复时剧烈运动
  g_target_gimbal_yaw_rad = g_current_gimbal_yaw_cumulative; 
  g_target_pitch_rad = imu_datas.euler_vals[PITCH];
}

float CalculateShortestError(float target, float current) {
    float error = target - current;
    
    // 核心：将误差限制在 -PI 到 +PI 之间
    // 如果误差是 350度 (约6.1 rad)，这就变成了 -10度 (-0.17 rad)
    while (error > 3.1415926f) error -= 6.2831853f;
    while (error < -3.1415926f) error += 6.2831853f;
    
    return error;
}

static float NormalizeAngle(float angle) {
    // 使用 fmod 进行快速取模，然后调整范围
    angle = fmodf(angle, M_TWOPI); 
    
    if (angle > M_PI) {
        angle -= M_TWOPI;
    } else if (angle < -M_PI) {
        angle += M_TWOPI;
    }
    return angle;
}

/**
 * @brief 统一控制逻辑
 * @note  无论什么模式，云台始终基于“绝对目标”进行控制。
 * 差异在于发送给底盘的 mode 标志，底盘据此决定是否跟随或自旋。
 */
static void RunControlMode(void) {
  float yaw_stick = -rc_ptr->rc_rh();
  if (fabsf(yaw_stick) < 0.05f) yaw_stick = 0.0f;
  g_target_gimbal_yaw_rad += yaw_stick * RC_MAX_YAW_SPEED * kCtrlPeriod;
  if (g_target_gimbal_yaw_rad > PI) {
      g_target_gimbal_yaw_rad -= 2.0f * PI;
  } else if (g_target_gimbal_yaw_rad < -PI) {
      g_target_gimbal_yaw_rad += 2.0f * PI;
  }
  // 1. 更新 Yaw 目标 (基于世界坐标系积分)
  // 右摇杆水平通道控制 Yaw 速度
  // float yaw_speed_cmd = -rc_ptr->rc_rh() * RC_MAX_YAW_SPEED;
  // g_target_gimbal_yaw_rad += yaw_speed_cmd * kCtrlPeriod;   
  // 2. 更新 Pitch 目标 (基于关节/IMU 角度)
  // 右摇杆垂直通道控制 Pitch 绝对角度
  g_target_pitch_rad = rc_ptr->rc_rv() * RC_MAX_PITCH_RAD; 
  
  // 限制 Pitch 角度 (防止机械碰撞)
  // 假设限位在 +/- 25度 (约 0.43 rad)
  if (g_target_pitch_rad > 0.43f) g_target_pitch_rad = 0.43f;
  if (g_target_pitch_rad < -0.43f) g_target_pitch_rad = -0.43f;
  real_motor_target = g_target_pitch_rad + 3.1415926f;
  real_motor_target = NormalizeAngle(real_motor_target);

  pitch_fb = g_pitch_motor.para.pos;

  // 2. 计算“最短”误差
  // 假设目标是 0 (水平)，反馈是 3.14 (背后) -> 误差 = -3.14
  // 假设目标是 0，反馈是 -3.14 -> 误差 = 3.14
  // 这样电机就会走最近的路，而不会绕大圈
  Pid_SetParams(g_pitch_pid, &pitch_pid_params);
  pitch_error = CalculateShortestError(real_motor_target, pitch_fb);
  target_speed = Pid_Calc(g_pitch_pid, pitch_error, 0.0f); 
  // 注意：因为我们传进去的是 error, 所以 feedback 填 0 即可
  // 或者使用: Pid_CalcAngle(pid, target, current) 如果该函数内部处理了过零

  // 4. 安全限速 (防止 PID 输出过大导致疯车)
  if (target_speed > 10.0f) target_speed = 10.0f;
  if (target_speed < -10.0f) target_speed = -10.0f;

  // 5. 发送速度指令 (推荐使用 MIT 模式模拟速度环)
  // Kp=0 (无位置刚度), Kd=2.0 (速度刚度), Vel=target_speed
  // 这样既利用了 MIT 的高响应，又执行了我们的速度指令
  mit_ctrl(&hcan2, g_pitch_motor.para.id, 0.0f, target_speed, 0.0f, 2.0f, 0.0f);
  
  // 4. 准备底盘指令 (云台坐标系下的速度)
  // 左摇杆控制平移
  g_chassis_cmd.vx = rc_ptr->rc_lv() * RC_MAX_SPEED_X;
  g_chassis_cmd.vy = -rc_ptr->rc_lh() * RC_MAX_SPEED_Y; 
  g_chassis_cmd.mode = (uint8_t)g_control_mode;
  
  // 注意：这里不进行坐标系转换，也不计算底盘旋转速度。
  // 这些全部交给底盘板，依据收到的 mode 和 角度差 进行解算。
}


static void SetAllMotorsZero(void) {
  mit_ctrl(&hcan2, g_pitch_motor.para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  if(g_pitch_pid) Pid_Reset(g_pitch_pid);
}

static void SendGimbalCanCommands(void) {
  // 发送给底盘的板间通信协议
  // 底盘板需要这些信息来控制 Yaw 电机和 轮子
  int16_t vx_int = (int16_t)(g_chassis_cmd.vx * 1000.0f); // 放大1000倍
  int16_t vy_int = (int16_t)(g_chassis_cmd.vy * 1000.0f); 
  g_tx_buf[0] = (vx_int >> 8); 
  g_tx_buf[1] = (vx_int & 0xFF);
  g_tx_buf[2] = (vy_int >> 8); 
  g_tx_buf[3] = (vy_int & 0xFF);
  g_tx_buf[4] = g_chassis_cmd.mode;
  CAN_Send_Msg(&hcan1, g_tx_buf, CAN_ID_TX_CMD_1, 8);
  // 帧 1 (ID 0x300): 平移速度 [vx_float, vy_float] (云台系)
  // memcpy(g_tx_buf,     &g_chassis_cmd.vx, sizeof(float));
  // memcpy(g_tx_buf + 4, &g_chassis_cmd.vy, sizeof(float));
  // CAN_Send_Msg(&hcan1, g_tx_buf, CAN_ID_TX_CMD_1, 8);
  
  // // 帧 2 (ID 0x301): 模式 [mode_u8]
  // memcpy(g_tx_buf, &g_chassis_cmd.mode, sizeof(uint8_t));
  // CAN_Send_Msg(&hcan1, g_tx_buf, CAN_ID_TX_CMD_2, 1);
  
  // 帧 3 (ID 0x302): 云台期望 Yaw (绝对累积角度) [target_yaw_float]
  memcpy(g_tx_buf, &g_target_gimbal_yaw_rad, sizeof(float));
  memcpy(g_tx_buf + 4, &g_current_gimbal_yaw_cumulative, sizeof(float));
  CAN_Send_Msg(&hcan1, g_tx_buf, CAN_ID_TX_YAW_TARGET, 8);
}