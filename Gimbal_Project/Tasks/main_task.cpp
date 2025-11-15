/**
*******************************************************************************
* @file      :main_task.cpp (Gimbal)
* @brief     : 云台C板 - 主循环任务
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
// **注意**: 必须在机器人上进行实际整定!
// Pitch 轴 (DM4310) - 角度环 (rad) (输出 Kp*e, MIT 模式)
#define PID_PITCH_PARAMS {30.0f, 0.0f, 0.5f, 5.0f, 1.0f} // Kp, Ki, Kd, max_out (T_ff), max_int

/* Private types -------------------------------------------------------------*/
// 用于打包发送给底盘的遥控器数据
typedef struct {
    float vx;       // m/s
    float vy;       // m/s
    float wz;       // rad/s
    uint8_t mode;   // RobotControlMode
    uint8_t _pad[3]; // 4字节对齐
} ChassisCommand_t;


/* Private variables ---------------------------------------------------------*/
uint32_t tick = 0;
const float kCtrlPeriod = 0.001f; // 1ms

// -- 遥控器 --
namespace remote_control = hello_world::devices::remote_control;
static const uint8_t kRxBufLen = remote_control::kRcRxDataLen;
static uint8_t rc_rx_buf[kRxBufLen];
remote_control::DT7 *rc_ptr = nullptr;

// -- 机器人状态 --
RobotControlMode g_control_mode = MODE_SAFETY_CALIB; 
ImuDatas_t imu_datas = {0}; // 云台板自己的 IMU 数据

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

/**
 * @brief 主初始化 (由 main.c 调用)
 */
void MainInit(void) {
  RobotInit();

  // 开启CAN1和CAN2
  CanFilter_Init(&hcan1);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  CanFilter_Init(&hcan2);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

  // 开启遥控器接收
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rc_rx_buf, kRxBufLen);

  // 开启定时器 (TIM6, 1kHz)
  HAL_TIM_Base_Start_IT(&htim6);
}

/**
 * @brief 主任务 (由 main.c 的 TIM6 中断 1kHz 调用)
 */
void MainTask_Loop(void) {
  tick++;
  
  // "注意事项 3" : 启动安全期 (1000ms)
  if(tick <= IMU_CALIB_TIME) {
      ImuCalibrate(); // 累加零漂值

      if (tick == IMU_CALIB_TIME) {
          ImuFinalizeCalibration(); // 计算零漂均值
      }
      
      RunSafetyMode();
  }
  else {
      ImuUpdate();
      RobotTask();
  }

  // 持续发送 CAN 指令
  SendGimbalCanCommands();
}

/**
 * @brief C++ 和 C 共享的 UART 中断回调
 */
void MainTask_UART_Callback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == &huart3) {
    if (Size == kRxBufLen) {
      // 考核要求 1: 在遥控器解包处喂狗
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

  // 1. 初始化 PID
  PidParams pitch_pid_params = PID_PITCH_PARAMS;
  g_pitch_pid = Pid_Create(&pitch_pid_params);

  // 2. 初始化云台电机
  // Pitch: DM4310, ID 2
  joint_motor_init(&g_pitch_motor, GIMBAL_PITCH_MOTOR_ID, MIT_MODE);
  
  // 3. 使能电机
  // (延时确保 CAN 总线稳定)
  HAL_Delay(100); 
  enable_motor_mode(&hcan2, g_pitch_motor.para.id, MIT_MODE);
  HAL_Delay(10);
  
  // 4. 清空发送缓冲
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
      RunControlMode(); // 云台板不区分底盘模式，只管控制 Pitch 和转发遥控器
      break;
    default:
      RunEStopMode(); 
      break;
  }
}

static void UpdateControlMode(void) {
  if (rc_ptr == NULL) return;

  // 考核要求 2: 急停键
  if (rc_ptr->rc_l_switch() == remote_control::kSwitchStateDown) {
    g_control_mode = MODE_ESTOP;
    return;
  }
  
  // 根据右拨杆切换模式 (云台只负责转发)
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
  // (不要发送 CAN 指令)
}

static void RunEStopMode(void) {
  SetAllMotorsZero();
  // (发送 0 指令)
  g_chassis_cmd = {0}; 
  g_chassis_cmd.mode = (uint8_t)MODE_ESTOP;
}

static void RunControlMode(void) {
  // 1. 控制 Pitch 轴
  // 获取遥控器 Pitch 目标
  float target_pitch = rc_ptr->rc_rv() * RC_MAX_PITCH_RAD; // 右摇杆上下
  // 获取 IMU Pitch 反馈
  float fdb_pitch = imu_datas.euler_vals[1]; // PITCH=1
  // 计算 PID (DM4310 MIT 模式, Kp=0, Kd=0, T_ff=PID_out)
  float t_ff = Pid_CalcAngle(g_pitch_pid, target_pitch, fdb_pitch);
  // 发送电机指令
  mit_ctrl(&hcan2, g_pitch_motor.para.id, 0.0f, 0.0f, 0.0f, 0.0f, t_ff);

  // 2. 准备底盘指令 (云台坐标系)
  // 左摇杆 (LV, LH) -> 平移 (vx, vy)
  g_chassis_cmd.vx = rc_ptr->rc_lv() * RC_MAX_SPEED_X;
  g_chassis_cmd.vy = -rc_ptr->rc_lh() * RC_MAX_SPEED_Y; 
  // 右摇杆 (RH) -> 旋转 (wz)
  g_chassis_cmd.wz = -rc_ptr->rc_rh() * RC_MAX_SPEED_WZ;
  // 模式
  g_chassis_cmd.mode = (uint8_t)g_control_mode;
}


static void SetAllMotorsZero(void) {
  // DM4310 MIT 模式发 0 力矩
  mit_ctrl(&hcan2, g_pitch_motor.para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  
  if(g_pitch_pid) Pid_Reset(g_pitch_pid);
}

static void SendGimbalCanCommands(void) {
  // 1. 发送板间通信 (CAN1)
  // 将结构体打包成 8 字节
  // (float vx, float vy) -> 8 bytes
  // (float wz, uint8_t mode) -> 8 bytes (需要两次发送)
  
  // V1.3.1 的底盘代码只接收遥控器，不接收模式
  // 我们将打包 [float vx][float vy] 到 0x300
  // 并将 [float wz][uint8_t mode] 打包到 0x301
  
  // (ID 0x300: [vx_f] [vy_f])
  memcpy(g_chassis_tx_buf,     &g_chassis_cmd.vx, sizeof(float));
  memcpy(g_chassis_tx_buf + 4, &g_chassis_cmd.vy, sizeof(float));
  CAN_Send_Msg(&hcan1, g_chassis_tx_buf, CAN_ID_TX_GIMBAL_TO_CHASSIS, 8);
  
  // (ID 0x301: [wz_f] [mode_u8] [pad*3])
  memcpy(g_chassis_tx_buf,     &g_chassis_cmd.wz, sizeof(float));
  memcpy(g_chassis_tx_buf + 4, &g_chassis_cmd.mode, sizeof(uint8_t));
  CAN_Send_Msg(&hcan1, g_chassis_tx_buf, CAN_ID_TX_GIMBAL_TO_CHASSIS + 1, 8);

  // 2. 发送云台电机指令 (CAN2)
  // (已在 RunControlMode() 和 SetAllMotorsZero() 中通过 mit_ctrl() 发送)
}