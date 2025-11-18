#include "pid.hpp"
#include <cstddef> // for NULL
#include <cmath>   // 包含 cmath 以使用 fabs

// -----------------------------------------------------------------
// C++ 类 (Pid) 的实现
// -----------------------------------------------------------------

Pid::Pid(PidParams &params) {
    setParams(params);
    // 默认采样时间 1ms (与 TIM6 1kHz 保持一致)
    // 若需要动态 dt，请调用 Pid_SetDt 或 setDt
    setDt(0.001f);
    reset();
}

void Pid::setParams(PidParams &params) {
    params_ = params;
}

void Pid::setDt(float dt) {
    // 防止异常 dt；若传入不合理则保持原值
    if (dt > 0.0f && dt <= 1.0f) {
        dt_ = dt;
    }
}

PidParams Pid::getParams(void) {
    return params_;
}

void Pid::reset(void) {
    data_.error = 0;
    data_.last_error = 0;
    data_.integral = 0;
    data_.derivative = 0;
    data_.output = 0;
}

float Pid::clamp(float value, float min, float max) {
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

/**
 * @brief 计算PID (速度环, rad/s)
 */
float Pid::pidCalc(const float ref, const float fdb) {
    data_.error = ref - fdb;
    const float dt = (dt_ > 1e-6f) ? dt_ : 1.0f; // 兜底，避免除零
    
    // --- [修改] 积分死区, 适用于 rad/s ---
    // 0.5 rad/s (约 28 deg/s) 以下的误差不累积积分
    const float integral_deadband = 0.5f; 

    if (fabsf(data_.error) > integral_deadband) {
        data_.integral += data_.error * dt; // 融合 dt
        data_.integral = clamp(data_.integral, -params_.max_integral, params_.max_integral);
    }
    // --- [修改结束] ---
    
    data_.derivative = (data_.error - data_.last_error) / dt; // 融合 dt
    
    data_.output = params_.Kp * data_.error +
                   params_.Ki * data_.integral +
                   params_.Kd * data_.derivative;
                   
    data_.output = clamp(data_.output, -params_.max_out, params_.max_out);
    
    data_.last_error = data_.error;
    
    return data_.output;
}

/**
 * @brief 计算PID（角度），已添加积分死区
 */
float Pid::pidCalc_Angle(const float ref, const float fdb) {
    data_.error = ref - fdb;
    const float dt = (dt_ > 1e-6f) ? dt_ : 1.0f; // 兜底

    // 角度环绕逻辑 (只对误差进行环绕)
    while (data_.error > M_PI) {
        data_.error -= 2.0f * M_PI;
    }
    while (data_.error < -M_PI) {
        data_.error += 2.0f * M_PI;
    }

    // --- [修改] 积分死区 ---
    // 0.1 弧度 (约 5.7 度)
    const float integral_deadband = 0.1f; 

    if (fabsf(data_.error) > integral_deadband) {
        data_.integral += data_.error * dt; // 融合 dt
        data_.integral = clamp(data_.integral, -params_.max_integral, params_.max_integral);
    }
    // --- [修改结束] ---
    
    data_.derivative = (data_.error - data_.last_error) / dt; // 融合 dt
    
    data_.output = params_.Kp * data_.error +
                   params_.Ki * data_.integral +
                   params_.Kd * data_.derivative;
                   
    data_.output = clamp(data_.output, -params_.max_out, params_.max_out);
    
    data_.last_error = data_.error;
    
    return data_.output;
}

// -----------------------------------------------------------------
// C 语言 API 函数的实现 (保持不变)
// -----------------------------------------------------------------
PidHandle Pid_Create(PidParams *params) {
    if (params == NULL) {
        return NULL;
    }
    return (PidHandle)(new Pid(*params));
}

void Pid_Destroy(PidHandle handle) {
    if (handle != NULL) {
        delete (Pid*)handle;
    }
}

float Pid_Calc(PidHandle handle, float ref, float fdb) {
    if (handle == NULL) {
        return 0.0f;
    }
    return ((Pid*)handle)->pidCalc(ref, fdb);
}

float Pid_CalcAngle(PidHandle handle, float ref, float fdb) {
    if (handle == NULL) {
        return 0.0f;
    }
    return ((Pid*)handle)->pidCalc_Angle(ref, fdb);
}

void Pid_Reset(PidHandle handle) {
    if (handle != NULL) {
        ((Pid*)handle)->reset();
    }
}

void Pid_SetParams(PidHandle handle, PidParams *params) {
    if (handle != NULL && params != NULL) {
        ((Pid*)handle)->setParams(*params);
    }
}

void Pid_SetDt(PidHandle handle, float dt) {
    if (handle != NULL) {
        ((Pid*)handle)->setDt(dt);
    }
}