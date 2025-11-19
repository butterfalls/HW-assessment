#include "pid.hpp"
#include <cstddef> // for NULL
#include <cmath>   // 包含 cmath 以使用 fabs

// -----------------------------------------------------------------
// C++ 类 (Pid) 的实现
// -----------------------------------------------------------------

Pid::Pid(PidParams &params) {
    setParams(params);
    // 参数兜底
    if (params_.dt <= 0.0f) params_.dt = 0.001f;
    if (params_.d_filter_gain <= 0.0f) params_.d_filter_gain = 1.0f;
    reset();
}

void Pid::setParams(PidParams &params) {
    params_ = params;
}

void Pid::setDt(float dt) {
    if (dt > 0.0f) {
        params_.dt = dt;
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
    data_.last_fdb = 0;
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

    // 1) 死区处理
    if (fabsf(data_.error) < params_.deadband) {
        data_.error = 0.0f;
        //data_.output = 0.0f; // 可选：进入死区直接输出0
    }

    // 2) 积分处理: 梯形积分 + 积分分离
    if (fabsf(data_.error) < params_.integral_range) {
        float step_i = (data_.error + data_.last_error) * 0.5f * params_.dt;
        data_.integral += step_i;
        data_.integral = clamp(data_.integral, -params_.max_integral, params_.max_integral);
    } else {
        // 误差过大时，不仅不积，有些策略甚至会清空积分，或者保持不变
        // data_.integral = 0.0f; // 视情况打开
    }

    // 3) 微分先行 + 低通滤波
    float derivative_raw = -(fdb - data_.last_fdb) / ((params_.dt > 1e-6f) ? params_.dt : 1.0f);
    data_.derivative = params_.d_filter_gain * derivative_raw +
                       (1.0f - params_.d_filter_gain) * data_.derivative;

    // 4) 输出组合与限幅
    data_.output = params_.Kp * data_.error +
                   params_.Ki * data_.integral +
                   params_.Kd * data_.derivative;
    data_.output = clamp(data_.output, -params_.max_out, params_.max_out);

    // 5) 更新历史
    data_.last_error = data_.error;
    data_.last_fdb = fdb;

    return data_.output;
}

/**
 * @brief 计算PID（角度），已添加积分死区
 */
float Pid::pidCalc_Angle(const float ref, const float fdb) {
    float error = ref - fdb;
    while (error > M_PI)  error -= 2.0f * M_PI;
    while (error < -M_PI) error += 2.0f * M_PI;
    return pidCalc(fdb + error, fdb);
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
