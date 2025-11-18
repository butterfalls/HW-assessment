/**
 *******************************************************************************
 * @file      : swerve_drive.cpp
 * @brief     : 舵轮底盘运动学解算库实现
 *******************************************************************************
 */

#include "swerve_drive.hpp"
#include <cstddef> // for NULL
#include <math.h>

// -----------------------------------------------------------------
// C++ 类 (SwerveDrive) 的实现
// -----------------------------------------------------------------

SwerveDrive::SwerveDrive(float wheelbase_x, float wheelbase_y, float wheel_radius)
    : L_(wheelbase_x), W_(wheelbase_y), R_(wheel_radius) {
    for (int i = 0; i < 4; ++i) {
        states_[i] = {0.0f, 0.0f};
    }
}

/**
 * @brief 核心解算函数
 */
void SwerveDrive::calculate(const float vx, const float vy, const float wz, const float current_angles[4]) {
    // 舵轮底盘逆运动学解算
    // 1. 计算每个轮子在底盘坐标系下的速度矢量
    // 轮子 0 (FR): {+L, -W}
    float A_0 = vx + wz * W_;
    float B_0 = vy + wz * L_;
    // 轮子 1 (FL): {+L, +W}
    float A_1 = vx - wz * W_;
    float B_1 = vy + wz * L_;
    // 轮子 2 (BL): {-L, +W}
    float A_2 = vx - wz * W_;
    float B_2 = vy - wz * L_;
    // 轮子 3 (BR): {-L, -W}
    float A_3 = vx + wz * W_;
    float B_3 = vy - wz * L_;

    float A[] = {A_0, A_1, A_2, A_3};
    float B[] = {B_0, B_1, B_2, B_3};

    float max_speed = 0.0f;

    for (int i = 0; i < 4; ++i) {
        // 2. 计算目标速度和角度
        float target_speed = sqrtf(A[i] * A[i] + B[i] * B[i]);
        float target_angle = atan2f(B[i], A[i]); // 目标角度

        // 3. 角度优化 (关键)
        // 我们希望航向电机转动最小的角度
        float current_angle = current_angles[i];
        float delta_angle = target_angle - current_angle;

        // 规范化到 [-pi, pi]
        while (delta_angle > M_PI) delta_angle -= 2.0f * M_PI;
        while (delta_angle < -M_PI) delta_angle += 2.0f * M_PI;

        if (fabsf(delta_angle) > (M_PI / 2.0f)) {
            // 如果转动超过90度，不如反转轮子速度，转动 (180 - delta) 度
            target_speed = -target_speed;
            target_angle += M_PI;
            // 重新规范化 target_angle
            if (target_angle > M_PI) target_angle -= 2.0f * M_PI;
        }
        
        // 确保目标角度仍然在 [-pi, pi] 范围内
        while (target_angle > M_PI) target_angle -= 2.0f * M_PI;
        while (target_angle < -M_PI) target_angle += 2.0f * M_PI;

        states_[i].angle_rad = target_angle;
        states_[i].speed_rads = target_speed / R_; // (m/s) / (m) = rad/s

        if (fabsf(states_[i].speed_rads) > max_speed) {
            max_speed = fabsf(states_[i].speed_rads);
        }
    }

    // 4. 速度归一化 (如果任何一个轮子超速)
    // 注意：这里需要一个最大速度限制，我们暂时假设为 1.0 (100%)
    // 实际应用中应设为 M3508 的最大轮速 (rad/s)
    const float MAX_WHEEL_SPEED_RADS = 30.0f; // 假设最大 30 rad/s
    if (max_speed > MAX_WHEEL_SPEED_RADS) {
        float scale = MAX_WHEEL_SPEED_RADS / max_speed;
        for (int i = 0; i < 4; ++i) {
            states_[i].speed_rads *= scale;
        }
    }
}

SwerveModuleState SwerveDrive::getModuleState(uint8_t index) {
    if (index < 4) {
        return states_[index];
    }
    return {0.0f, 0.0f};
}

// -----------------------------------------------------------------
// C 语言 API 函数的实现
// -----------------------------------------------------------------

SwerveDriveHandle SwerveDrive_Create(float wheelbase_x, float wheelbase_y, float wheel_radius) {
    return (SwerveDriveHandle)(new SwerveDrive(wheelbase_x, wheelbase_y, wheel_radius));
}

void SwerveDrive_Destroy(SwerveDriveHandle handle) {
    if (handle != NULL) {
        delete (SwerveDrive*)handle;
    }
}

void SwerveDrive_Calculate(SwerveDriveHandle handle, const float vx, const float vy, const float wz, const float current_angles[4]) {
    if (handle != NULL) {
        ((SwerveDrive*)handle)->calculate(vx, vy, wz, current_angles);
    }
}

SwerveModuleState SwerveDrive_GetModuleState(SwerveDriveHandle handle, uint8_t index) {
    if (handle != NULL) {
        return ((SwerveDrive*)handle)->getModuleState(index);
    }
    return {0.0f, 0.0f};
}