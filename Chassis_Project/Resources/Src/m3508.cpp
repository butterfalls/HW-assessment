/**
 *******************************************************************************
 * @file      : m3508.cpp
 * @brief     : M3508 (C620 电调) 电机库实现
 *******************************************************************************
 */

#include "m3508.hpp"
#include <cstddef> // for NULL

// -----------------------------------------------------------------
// C++ 类 (M3508) 的实现
// -----------------------------------------------------------------

M3508::M3508(uint32_t id) : id_(id) {
    input_current_ = 0;
    angle_raw_ = 0;
    last_angle_raw_ = 0;
    vel_rpm_ = 0;
    actual_current_ = 0;
    temp_ = 0;
    total_round_ = 0;
    total_angle_rad_ = 0.0f;
}

/**
 * @brief 从CAN总线数据中解码电机反馈
 * @param data 长度为8的CAN Rx数据缓冲区 (C620反馈帧)
 * data[0..1]: 机械角度 (0-8191)
 * data[2..3]: 转速 (RPM)
 * data[4..5]: 实际扭矩电流
 * data[6]:     温度
 */
void M3508::decode(uint8_t *data) {
    last_angle_raw_ = angle_raw_;
    angle_raw_ = (uint16_t)(data[0] << 8 | data[1]);
    vel_rpm_ = (int16_t)(data[2] << 8 | data[3]);
    actual_current_ = (int16_t)(data[4] << 8 | data[5]);
    temp_ = data[6];

    // --- 连续角度（多圈角度）计算 ---
    int16_t delta = (int16_t)(angle_raw_ - last_angle_raw_);
    // 处理编码器环绕
    if (delta > (M3508_ENCODER_MAX / 2.0f)) { 
        // 从 8191 环绕到 0
        total_round_--;
    } else if (delta < -(M3508_ENCODER_MAX / 2.0f)) { 
        // 从 0 环绕到 8191
        total_round_++;
    }
    
    // 计算总角度 (rad)
    total_angle_rad_ = (total_round_ * 2.0f * M_PI) + (angle_raw_ * M3508_RAD_PER_TICK);
}

void M3508::setInputCurrent(int16_t input_current) {
    // C620 的电流/电压范围是 -16384 到 16384
    if (input_current > 16384) input_current = 16384;
    if (input_current < -16384) input_current = -16384;
    input_current_ = input_current;
}

int16_t M3508::getInputCurrent(void) {
    return input_current_;
}

uint32_t M3508::getId(void) {
    return id_;
}

uint16_t M3508::getRawAngle(void) {
    return angle_raw_;
}

float M3508::getMotorAngleRad(void) {
    // 返回 [0, 2*PI] 范围内的电机转子角度
    return (float)angle_raw_ * M3508_RAD_PER_TICK;
}

float M3508::getOutputTotalAngleRad(void) {
    // 返回减速后的输出轴累计角度
    return total_angle_rad_ / M3508_REDUCTION_RATIO;
}

int16_t M3508::getVelRPM(void) {
    return vel_rpm_;
}

float M3508::getOutputVelRadS(void) {
    // 将 RPM 转换为 rad/s 并考虑减速比
    return (float)vel_rpm_ * M3508_RPM_TO_RADS / M3508_REDUCTION_RATIO;
}

int16_t M3508::getActualCurrent(void) {
    return actual_current_;
}

uint8_t M3508::getTemp(void) {
    return temp_;
}

// -----------------------------------------------------------------
// C 语言 API 函数的实现
// -----------------------------------------------------------------

M3508Handle M3508_Create(uint32_t id) {
    return (M3508Handle)(new M3508(id));
}

void M3508_Destroy(M3508Handle handle) {
    if (handle != NULL) {
        delete (M3508*)handle;
    }
}

void M3508_Decode(M3508Handle handle, uint8_t *data) {
    if (handle != NULL && data != NULL) {
        ((M3508*)handle)->decode(data);
    }
}

void M3508_SetInputCurrent(M3508Handle handle, int16_t input_current) {
    if (handle != NULL) {
        ((M3508*)handle)->setInputCurrent(input_current);
    }
}

int16_t M3508_GetInputCurrent(M3508Handle handle) {
    if (handle == NULL) return 0;
    return ((M3508*)handle)->getInputCurrent();
}

uint32_t M3508_GetId(M3508Handle handle) {
    if (handle == NULL) return 0;
    return ((M3508*)handle)->getId();
}

uint16_t M3508_GetRawAngle(M3508Handle handle) {
    if (handle == NULL) return 0;
    return ((M3508*)handle)->getRawAngle();
}

float M3508_GetMotorAngleRad(M3508Handle handle) {
    if (handle == NULL) return 0.0f;
    return ((M3508*)handle)->getMotorAngleRad();
}

float M3508_GetOutputTotalAngleRad(M3508Handle handle) {
    if (handle == NULL) return 0.0f;
    return ((M3508*)handle)->getOutputTotalAngleRad();
}

int16_t M3508_GetVelRPM(M3508Handle handle) {
    if (handle == NULL) return 0;
    return ((M3508*)handle)->getVelRPM();
}

float M3508_GetOutputVelRadS(M3508Handle handle) {
    if (handle == NULL) return 0.0f;
    return ((M3508*)handle)->getOutputVelRadS();
}

int16_t M3508_GetActualCurrent(M3508Handle handle) {
    if (handle == NULL) return 0;
    return ((M3508*)handle)->getActualCurrent();
}

uint8_t M3508_GetTemp(M3508Handle handle) {
    if (handle == NULL) return 0;
    return ((M3508*)handle)->getTemp();
}