/**
 *******************************************************************************
 * @file      : gm6020.cpp
 * @brief     : GM6020 电机库实现
 * @history   :
 * Version     Date            Author          Note
 * V1.5.0      2025-11-16      Gemini          1. [遵照要求] 从电压模式切换到电流模式
 *******************************************************************************
 */
#include "gm6020.hpp"
#include <cstddef> // for NULL
#include <math.h> 

// -----------------------------------------------------------------
// C++ 类 (GM6020) 的实现
// -----------------------------------------------------------------
GM6020::GM6020(uint32_t id) : id_(id) {
    input_current_ = 0; // [遵照要求]
    angle_raw_ = 0;
    vel_rpm_ = 0;
    current_ = 0;
    temp_ = 0;
    total_angle_rad_ = 0.0f;
    last_angle_rad_ = 0.0f;
}

void GM6020::decode(uint8_t *data) {
    angle_raw_ = (uint16_t)(data[0] << 8 | data[1]);
    vel_rpm_ = (int16_t)(data[2] << 8 | data[3]);
    current_ = (int16_t)(data[4] << 8 | data[5]);
    temp_ = data[6];

    // --- 计算连续角度 ---
    float current_angle_rad = (float)angle_raw_ * GM6020_RAD_PER_TICK;
    
    if (total_angle_rad_ == 0.0f && last_angle_rad_ == 0.0f) {
        last_angle_rad_ = current_angle_rad;
    }

    float delta_angle = current_angle_rad - last_angle_rad_;
    if (delta_angle > M_PI) { 
        delta_angle -= 2.0f * M_PI;
    } else if (delta_angle < -M_PI) { 
        delta_angle += 2.0f * M_PI;
    }
    
    total_angle_rad_ += delta_angle;
    last_angle_rad_ = current_angle_rad;
}

// [遵照要求] 修改为电流模式
void GM6020::setInputCurrent(int16_t input_current) {
    // GM6020 电流范围: -16384 ~ +16384
    if (input_current > GM6020_CURRENT_MAX) input_current = GM6020_CURRENT_MAX;
    if (input_current < -GM6020_CURRENT_MAX) input_current = -GM6020_CURRENT_MAX;
    input_current_ = input_current;
}

int16_t GM6020::getInputCurrent(void) {
    return input_current_;
}

uint32_t GM6020::getId(void) {
    return id_;
}

uint16_t GM6020::getRawAngle(void) {
    return angle_raw_;
}

float GM6020::getAngleRad(void) {
    return (float)angle_raw_ * GM6020_RAD_PER_TICK;
}

float GM6020::getOutputAngleRad(void) {
    return total_angle_rad_;
}

int16_t GM6020::getVelRPM(void) {
    return vel_rpm_;
}

int16_t GM6020::getCurrent(void) {
    return current_;
}

uint8_t GM6020::getTemp(void) {
    return temp_;
}

// -----------------------------------------------------------------
// C 语言 API 函数的实现
// -----------------------------------------------------------------
GM6020Handle GM6020_Create(uint32_t id) {
    return (GM6020Handle)(new GM6020(id));
}

void GM6020_Destroy(GM6020Handle handle) {
    if (handle != NULL) {
        delete (GM6020*)handle;
    }
}

void GM6020_Decode(GM6020Handle handle, uint8_t *data) {
    if (handle != NULL && data != NULL) {
        ((GM6020*)handle)->decode(data);
    }
}

// [遵照要求]
void GM6020_SetInputCurrent(GM6020Handle handle, int16_t input_current) {
    if (handle != NULL) {
        ((GM6020*)handle)->setInputCurrent(input_current);
    }
}

int16_t GM6020_GetInputCurrent(GM6020Handle handle) {
    if (handle == NULL) return 0;
    return ((GM6020*)handle)->getInputCurrent();
}
// [遵照要求] 结束

uint32_t GM6020_GetId(GM6020Handle handle) {
    if (handle == NULL) return 0;
    return ((GM6020*)handle)->getId();
}

uint16_t GM6020_GetRawAngle(GM6020Handle handle) {
    if (handle == NULL) return 0;
    return ((GM6020*)handle)->getRawAngle();
}

float GM6020_GetAngleRad(GM6020Handle handle) {
    if (handle == NULL) return 0.0f;
    return ((GM6020*)handle)->getAngleRad();
}

float GM6020_GetOutputAngleRad(GM6020Handle handle) {
    if (handle == NULL) return 0.0f;
    return ((GM6020*)handle)->getOutputAngleRad();
}

int16_t GM6020_GetVelRPM(GM6020Handle handle) {
    if (handle == NULL) return 0;
    return ((GM6020*)handle)->getVelRPM();
}

int16_t GM6020_GetCurrent(GM6020Handle handle) {
    if (handle == NULL) return 0;
    return ((GM6020*)handle)->getCurrent();
}

uint8_t GM6020_GetTemp(GM6020Handle handle) {
    if (handle == NULL) return 0;
    return ((GM6020*)handle)->getTemp();
}