/**
*******************************************************************************
* @file      :main_task.cpp (Chassis)
* @brief     : 底盘C板 - 主循环任务 (V1.6.3 舵向最短路径优化)
* @note      : [优化] 解决舵轮长时间旋转导致的角度变量溢出问题
* @note      : [优化] 引入 CalculateShortestPath 实现舵向电机就近转动
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
#include "arm_math.h"

/* Private macro -------------------------------------------------------------*/
#ifndef M_TWOPI
#define M_TWOPI (2.0f * 3.14159265358979323846f)
#endif

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// [BUG 修复] volatile 变量定义在唯一的 .cpp 文件中
volatile uint32_t tick = 0;
const float kCtrlPeriod = 0.001f; // 1ms

// -- [V1.5.0] 由 CAN 回调 (HW_can.cpp) 填充
volatile RobotControlMode g_control_mode = MODE_SAFETY_CALIB; 
volatile float g_rc_cmd_vx = 0.0f;
volatile float g_rc_cmd_vy = 0.0f;
volatile float g_target_gimbal_yaw_rad = 0.0f; 
volatile float g_current_gimbal_yaw_rad = 0.0f; 

// -- [V1.6.1] 底盘 IMU 累积角度 --
float g_current_chassis_yaw_cumulative = 0.0f; // 底盘当前绝对 Yaw (连续值)
static float last_raw_chassis_yaw = 0.0f;
// static int32_t chassis_yaw_round_count = 0;

float input;
float target_speed;

// [BUG 修复] imu_datas 定义移至 imu_task.cpp

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

float vx_cmd = 0.0f; // 最终底盘坐标系 vx
float vy_cmd = 0.0f; // 最终底盘坐标系 vy
float wz_cmd = 0.0f; // 最终底盘旋转 wz
float target_speed_ref;

PidParams steer_pid_params = PID_STEER_PARAMS;
PidParams wheel_pid_params = PID_WHEEL_PARAMS;
PidParams follow_pid_params = PID_FOLLOW_PARAMS;
PidParams yaw_pid_params = PID_CHASSIS_YAW_PARAMS;


/* Private function prototypes -----------------------------------------------*/
static void RobotInit(void);
static void RobotTask(void);
static void RunSafetyMode(void);
static void RunControlMode(void);
static void RunEStopMode(void);
static void SetAllMotorsZero(void);
static void SendChassisCanCommands(void);
static void RunSwerveControl(float vx, float vy, float wz);
static void ControlChassisYaw(void); 
// [V1.6.1 新增]
// static void UpdateCumulativeChassisYaw(void);
static void CoordinateTransform(float gimbal_vx, float gimbal_vy, float* chassis_vx, float* chassis_vy);

// [新增] 计算最短路径误差
static float CalculateShortestAngleError(float target_rad, float current_rad);


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

/**
 * @brief [BUG 修复] 重命名为 Loop, 此函数由 C 的 HAL_TIM_PeriodElapsedCallback 调用
 */
void MainTask_Loop(void) {
  tick = tick +1;
  
  if(tick <= IMU_CALIB_TIME) {
      ImuCalibrate(); 
      if (tick == IMU_CALIB_TIME) {
          ImuFinalizeCalibration(); 
          
          // [V1.6.1 修正] 校准完成后，初始化底盘的累积Yaw
          last_raw_chassis_yaw = imu_datas.euler_vals[YAW];
          g_current_chassis_yaw_cumulative = last_raw_chassis_yaw; 
      }
      RunSafetyMode();
  }
  else {
      ImuUpdate(); // 获取底盘板自己的IMU数据
      
      // [V1.6.1 新增] 在 RobotTask 之前更新底盘的连续角度
      g_current_chassis_yaw_cumulative = imu_datas.euler_vals[YAW];
      
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

  g_follow_pid = Pid_Create(&follow_pid_params);
  g_chassis_yaw_pid = Pid_Create(&yaw_pid_params); 

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
  // [V1.6.4] 切换到 MIT 模式
  joint_motor_init(&g_yaw_motor, CHASSIS_YAW_MOTOR_ID, MIT_MODE);

  // 4. 初始化运动学解算器
  g_swerve_drive = SwerveDrive_Create(
      SWERVE_WHEELBASE_X / 2.0f,
      SWERVE_WHEELBASE_Y / 2.0f,
      SWERVE_WHEEL_RADIUS
  );

  // 5. 使能 Yaw 电机
  HAL_Delay(100); 
  // [V1.6.4] 使能 MIT 模式
  enable_motor_mode(&hcan2, g_yaw_motor.para.id, MIT_MODE);
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
    case MODE_FOLLOW:
    case MODE_SPIN:
      RunControlMode();
      break;
    default:
      RunEStopMode(); 
      break;
  }
}

/**
 * @brief [V1.6.4 修正] 三种模式的底盘控制逻辑
 */
static void RunControlMode(void) {
    
    // 1. 坐标转换
    // 接收到的 (g_rc_cmd_vx, g_rc_cmd_vy) 是云台系的
    // 转换到 (vx_cmd, vy_cmd) 底盘系
    CoordinateTransform(g_rc_cmd_vx, g_rc_cmd_vy, &vx_cmd, &vy_cmd);
    
    // 2. 根据模式计算底盘旋转速度 (wz_cmd)
    //    g_control_mode 是由 CAN 中断写入的 volatile 变量
    switch(g_control_mode) {
        
        case MODE_SEPARATE:
            // 分离模式: "底盘不参与旋转"
            wz_cmd = 0.0f;
            break;
            
        case MODE_FOLLOW:
            // 跟随模式: "底盘是跟着云台一同旋转的"
            // 底盘旋转 (wz_cmd) 的目标是让底盘的绝对角度 (g_current_chassis_yaw_cumulative)
            // 去追随云台的 *目标* 绝对角度 (g_target_gimbal_yaw_rad)
            Pid_SetParams(g_follow_pid,&follow_pid_params);
            wz_cmd = Pid_CalcAngle(g_follow_pid,
                                   0.0f,                   // 目标角度 (0.0 rad)
                                   g_yaw_motor.para.pos);  // 反馈角度 (来自Yaw电机)
            break;
            
        case MODE_SPIN:
            // 小陀螺模式: "要求底盘...自行旋转"
            // 底盘使用预设的 SPIN_MODE_W_SPEED (来自 system_user.hpp)
            wz_cmd = SPIN_MODE_W_SPEED;
            
            // [V1.5.0 协同逻辑]
            // ControlChassisYaw() 正在自动反转电机
            // 以保持云台在世界系中静止
            break;
        
        default:
            wz_cmd = 0.0f;
            break;
    }
    
    // 3. 将最终的底盘系指令 (vx, vy, wz) 设置给舵轮解算器
    RunSwerveControl(vx_cmd, vy_cmd, wz_cmd);
}

static void RunSafetyMode(void) {
  SetAllMotorsZero();
}

static void RunEStopMode(void) {
  SetAllMotorsZero();
}

/**
 * @brief Yaw 轴电机速度控制 (二次曲线映射)
 * @note  根据摇杆输入 (-1.0 ~ 1.0) 计算目标速度
 */
static void ControlChassisYaw(void)
{
    // // 1. 获取输入 (假设范围 -1.0 ~ 1.0)
    // // [警告] 请确保 g_target_gimbal_yaw_rad 是归一化的摇杆值！
    // input = g_target_gimbal_yaw_rad;
    // // 2. 限制输入范围 (安全防呆)
    // if (input > 1.0f) input = 1.0f;
    // if (input < -1.0f) input = -1.0f;
    // // 3. 定义最大旋转速度 (rad/s)
    // const float MAX_YAW_SPEED = 3.14159f * 2.0f; // 示例: 2转/秒
    // // 4. 计算二次曲线因子 (保留符号)
    // // fabsf(input) * input 既保留了原来的正负，又实现了平方效果
    // float curve_factor = fabsf(input) * input;
    // // 5. 计算目标速度
    // target_speed = MAX_YAW_SPEED * curve_factor;
    // target_speed *= 0.8f; // 按要求缩放至 80%

    float target_yaw = g_target_gimbal_yaw_rad;
    float current_yaw = g_current_gimbal_yaw_rad;
    float yaw_error = CalculateShortestAngleError(target_yaw, current_yaw);
    Pid_SetParams(g_chassis_yaw_pid,&yaw_pid_params);
    target_speed_ref = Pid_Calc(g_chassis_yaw_pid, yaw_error, 0.0f);
    const float MAX_YAW_SPEED = 20.0f;
    if (target_speed_ref > MAX_YAW_SPEED) target_speed_ref = MAX_YAW_SPEED;
    if (target_speed_ref < -MAX_YAW_SPEED) target_speed_ref = -MAX_YAW_SPEED;
    mit_ctrl(&hcan2, g_yaw_motor.para.id, 0.0f, target_speed_ref, 0.0f, 2.0f, 0.0f);
}


/**
 * @brief [优化] 计算角度的最短路径误差
 * @param target_rad 目标角度 (rad)
 * @param current_rad 当前角度 (rad, 0~2PI)
 * @return 误差值，范围在 [-PI, PI]
 */
static float CalculateShortestAngleError(float target_rad, float current_rad) {
    float error = target_rad - current_rad;
    // 归一化到 -PI ~ PI
    while (error > M_PI) {
        error -= M_TWOPI;
    }
    while (error < -M_PI) {
        error += M_TWOPI;
    }
    return error;
}

/**
 * @brief (V1.6.3) 运行舵轮解算和8个PID环 - 包含最短路径逻辑
 */
static void RunSwerveControl(float vx, float vy, float wz)
{
  if (g_swerve_drive == NULL) return;
  
  // [优化] 使用 GetAngleRad (0-2PI) 获取当前物理角度，避免溢出
  float current_angles[4] = {
      GM6020_GetAngleRad(g_steer_motors[SWERVE_FR_MODULE]),
      GM6020_GetAngleRad(g_steer_motors[SWERVE_FL_MODULE]),
      GM6020_GetAngleRad(g_steer_motors[SWERVE_BL_MODULE]),
      GM6020_GetAngleRad(g_steer_motors[SWERVE_BR_MODULE])
  };

  // 解算目标角度和速度
  SwerveDrive_Calculate(g_swerve_drive, vx, vy, wz, current_angles);

  for (int i = 0; i < 4; ++i) {
    SwerveModuleState target_state = SwerveDrive_GetModuleState(g_swerve_drive, i);
    
    // --- 1. 航向电机 (GM6020) 最短路径控制 ---
    // 获取当前角度 (0 ~ 2PI)
    float steer_fdb = GM6020_GetAngleRad(g_steer_motors[i]);
    
    // 计算最短误差 (-PI ~ PI)
    float steer_error = CalculateShortestAngleError(target_state.angle_rad, steer_fdb);
    
    // [高级优化] 矢量反转逻辑 (Swerve Optimization)
    // 如果误差绝对值大于 90度 (PI/2)，则无需转大弯，而是反转轮子方向，并调整目标角度
    // 这里只实现基础的最短误差输入给 PID
    // 注意：如果要使用 PID 计算，现在的 Error 已经是差值了，所以：
    // 目标设为 steer_error，反馈设为 0；或者 目标设为 steer_fdb + steer_error，反馈设为 steer_fdb
    // 最简单的方式是直接对误差进行 PID 计算
    Pid_SetParams(g_steer_pids[i],&steer_pid_params);
    float steer_output = Pid_Calc(g_steer_pids[i], steer_error, 0.0f);
    
    GM6020_SetInputCurrent(g_steer_motors[i], (int16_t)steer_output); 

    // --- 2. 轮速电机 (M3508) ---
    // [警告] M3508 用于驱动车轮，必须是速度闭环！
    // 不能使用 GetMotorAngleRad (位置)，否则车轮会锁死无法连续滚动。
    // 这里保持 GetOutputVelRadS (速度反馈)。速度值不会无限累积，不会溢出。
    float wheel_fdb = M3508_GetOutputVelRadS(g_wheel_motors[i]);
    Pid_SetParams(g_wheel_pids[i], &wheel_pid_params);
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
  // 2. Yaw 电机 (使用 mit_ctrl 模拟速度模式，发送速度0)
  mit_ctrl(&hcan2, g_yaw_motor.para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  
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

// /**
//  * @brief [V1.6.1] 处理底盘 IMU 的 Yaw 轴角度突变 (用于 MODE_FOLLOW)
//  */
// static void UpdateCumulativeChassisYaw(void) {
//     // 1. 获取底盘 IMU 的原始 Yaw
//     float current_raw_yaw = imu_datas.euler_vals[YAW];
//     float delta = current_raw_yaw - last_raw_chassis_yaw;

//     // 2. 检测过零突变 (阈值设为 1.5 PI)
//     if (delta < -1.5f * M_PI) { // 使用 arm_math.h 中的 M_PI
//         chassis_yaw_round_count++; // 正向跨越 (179 -> -179)
//     } else if (delta > 1.5f * M_PI) {
//         chassis_yaw_round_count--; // 负向跨越 (-179 -> 179)
//     }

//     // 3. 计算底盘的累积角度
//     g_current_chassis_yaw_cumulative = current_raw_yaw + chassis_yaw_round_count * 2.0f * M_PI;
    
//     // 4. 保存当前值
//     last_raw_chassis_yaw = current_raw_yaw;
// }

/**
 * @brief [V1.6.2 修正] 坐标转换
 * @desc  将云台坐标系下的 (vx, vy) 转换到底盘坐标系下
 */
static void CoordinateTransform(float gimbal_vx, float gimbal_vy, float* chassis_vx, float* chassis_vy) {
    // [V1.6.2 修正]
    // 根据用户测试反馈, 不再使用 (云台IMU - 底盘IMU) 的差值,
    // 而是直接使用 Yaw 轴电机 (DM4310) 的编码器反馈角度。
    // g_yaw_motor.para.position_f 是 Yaw 电机反馈的弧度值。
    
    // [!!] 警告: 此方法要求 Yaw 电机在 RobotInit() 时
    // 已被正确校准到 0.0 rad (即云台指向底盘正前方)。
    float delta_yaw_rad = g_yaw_motor.para.pos;
    
    // 2. 旋转矩阵
    float cos_delta = arm_cos_f32(delta_yaw_rad);
    float sin_delta = arm_sin_f32(delta_yaw_rad);
    
    // 3. (Gimbal Frame -> Chassis Frame)
    *chassis_vx = gimbal_vx * cos_delta - gimbal_vy * sin_delta;
    *chassis_vy = gimbal_vx * sin_delta + gimbal_vy * cos_delta;
}