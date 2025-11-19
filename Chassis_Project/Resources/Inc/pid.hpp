#ifndef _PID_HPP_
#define _PID_HPP_

#include <stdint.h>
#include <math.h> // 使用 C 兼容的 math.h

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// -----------------------------------------------------------------
// C/C++ 共享定义
// -----------------------------------------------------------------

/**
 * @brief PID 参数结构体 (C/C++ 兼容)
 */
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    
    float max_out;          // 最大输出限幅
    float max_integral;     // 积分限幅
    
    float deadband;         // 误差死区
    float integral_range;   // 积分分离阈值
    
    float d_filter_gain;    // 微分滤波系数 [0..1]
    float dt;               // 控制周期 (秒)
} PidParams;

// -----------------------------------------------------------------
// C++ 专属定义
// -----------------------------------------------------------------
#ifdef __cplusplus
#include <cmath> // C++ 可以使用 cmath

/**
 * @brief PID 内部数据 (仅 C++ 使用)
 */
struct PidData {
    float error;
    float last_error;
    float integral;
    float derivative;   // 滤波后的微分
    float last_fdb;     // 上一次反馈 (Derivative on Measurement)
    float output;
};

/**
 * @brief Pid C++ 类定义
 */
class Pid {
   public:
    Pid(PidParams &params);
    ~Pid() = default;

    void setParams(PidParams &params);
    void setDt(float dt); // 设置采样时间 (秒) -> 写入 params_.dt
    PidParams getParams(void);
    
    /**
     * @brief 线性PID计算 (例如：速度环)
     */
    float pidCalc(const float ref, const float fdb);
    
    /**
     * @brief 角度PID计算 (例如：航向环)
     * @note  处理 -PI 到 +PI 的环绕, 并包含积分死区
     */
    float pidCalc_Angle(const float ref, const float fdb);
    
    void reset(void);

   private:
    PidParams params_;
    PidData data_;
    float clamp(float value, float min, float max);
};

// 开始 C 语言链接声明 (告诉 C++ 编译器)
extern "C" {
#endif

// -----------------------------------------------------------------
// C 语言 API 接口 (C 和 C++ 均可见)
// -----------------------------------------------------------------

typedef void* PidHandle;

PidHandle Pid_Create(PidParams *params);
void Pid_Destroy(PidHandle handle);
float Pid_Calc(PidHandle handle, float ref, float fdb);
float Pid_CalcAngle(PidHandle handle, float ref, float fdb);
void Pid_Reset(PidHandle handle);
void Pid_SetParams(PidHandle handle, PidParams *params);


#ifdef __cplusplus
} // 结束 extern "C"
#endif

#endif /* _PID_HPP_ */