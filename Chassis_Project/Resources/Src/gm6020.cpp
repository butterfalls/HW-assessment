#include "gm6020.hpp"
#include <cstddef> // for NULL

// -----------------------------------------------------------------
// C++ 类 (GM6020) 的实现
// -----------------------------------------------------------------
GM6020::GM6020(uint32_t id) : id_(id) {
    input_ = 0;
    angle_raw_ = 0;
    vel_rpm_ = 0;
    current_ = 0;
    temp_ = 0;
}

void GM6020::decode(uint8_t *data) {
    angle_raw_ = (uint16_t)(data[0] << 8 | data[1]);
    vel_rpm_ = (int16_t)(data[2] << 8 | data[3]);
    current_ = (int16_t)(data[4] << 8 | data[5]);
    temp_ = data[6];
}

void GM6020::setInput(float input) {
    input_ = input;
}

float GM6020::getInput(void) {
    return input_;
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

void GM6020_SetInput(GM6020Handle handle, float input) {
    if (handle != NULL) {
        ((GM6020*)handle)->setInput(input);
    }
}

float GM6020_GetInput(GM6020Handle handle) {
    if (handle == NULL) return 0.0f;
    return ((GM6020*)handle)->getInput();
}

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