/**
 *******************************************************************************
 * @file      : swerve_drive.hpp
 * @brief     : 舵轮底盘运动学解算库 (Swerve Drive Kinematics)
 * @history   :
 * Version     Date            Author          Note
 * V1.0.0      2025-11-15      Gemini          1. 创建，C/C++ 兼容
 *******************************************************************************
 * @attention :
 * 1. 坐标系: X轴-底盘前，Y轴-底盘左，Z轴-底盘上 (右手系)
 * 2. 轮子编号: 0-FR(前右), 1-FL(前左), 2-BL(后左), 3-BR(后右)
 *******************************************************************************
 */

#ifndef _SWERVE_DRIVE_HPP_
#define _SWERVE_DRIVE_HPP_

#include <stdint.h>
#include <math.h> // C 兼容

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// -----------------------------------------------------------------
// C/C++ 共享定义
// -----------------------------------------------------------------

/**
 * @brief 单个舵轮模块的目标状态 (C/C++ 兼容)
 */
typedef struct {
    float angle_rad;  // 航向电机目标角度 (rad)
    float speed_rads; // 驱动电机目标速度 (rad/s)
} SwerveModuleState;

// -----------------------------------------------------------------
// C++ 专属定义
// -----------------------------------------------------------------
#ifdef __cplusplus

/**
 * @brief SwerveDrive C++ 类定义
 */
class SwerveDrive {
   public:
    /**
     * @brief 构造函数
     * @param wheelbase_x 底盘X轴轮距 (前后轮距) 的一半 (m)
     * @param wheelbase_y 底盘Y轴轮距 (左右轮距) 的一半 (m)
     * @param wheel_radius 轮子半径 (m)
     */
    SwerveDrive(float wheelbase_x, float wheelbase_y, float wheel_radius);
    ~SwerveDrive() = default;

    /**
     * @brief 逆运动学解算 (底盘速度 -> 四轮目标)
     * @param vx 底盘坐标系 X 轴速度 (m/s)
     * @param vy 底盘坐标系 Y 轴速度 (m/s)
     * @param wz 底盘坐标系 Z 轴角速度 (rad/s)
     * @param current_angles 各航向电机当前角度 (rad) [4]
     */
    void calculate(const float vx, const float vy, const float wz, const float current_angles[4]);

    /**
     * @brief 获取指定模块的计算后状态
     * @param index 轮子索引 (0-FR, 1-FL, 2-BL, 3-BR)
     * @return SwerveModuleState 目标状态
     */
    SwerveModuleState getModuleState(uint8_t index);

   private:
    float L_; // X轴轮距一半
    float W_; // Y轴轮距一半
    float R_; // 轮子半径
    SwerveModuleState states_[4]; // 0-FR, 1-FL, 2-BL, 3-BR
};

// 开始 C 语言链接声明
extern "C" {
#endif

// -----------------------------------------------------------------
// C 语言 API 接口
// -----------------------------------------------------------------
typedef void* SwerveDriveHandle;

SwerveDriveHandle SwerveDrive_Create(float wheelbase_x, float wheelbase_y, float wheel_radius);
void SwerveDrive_Destroy(SwerveDriveHandle handle);

/**
 * @brief C接口: 逆运动学解算 (底盘速度 -> 四轮目标)
 * @param handle 实例句柄
 * @param vx 底盘坐标系 X 轴速度 (m/s)
 * @param vy 底盘坐标系 Y 轴速度 (m/s)
 * @param wz 底盘坐标系 Z 轴角速度 (rad/s)
 * @param current_angles 各航向电机当前角度 (rad) [4]
 */
void SwerveDrive_Calculate(SwerveDriveHandle handle, const float vx, const float vy, const float wz, const float current_angles[4]);

SwerveModuleState SwerveDrive_GetModuleState(SwerveDriveHandle handle, uint8_t index);

#ifdef __cplusplus
} // 结束 extern "C"
#endif

#endif /* _SWERVE_DRIVE_HPP_ */