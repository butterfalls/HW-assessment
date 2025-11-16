/**
 *******************************************************************************
 * @file      : gm6020.hpp
 * @brief     : GM6020 电机库
 * @history   :
 * Version     Date            Author          Note
 * V1.5.0      2025-11-16      Gemini          1. [遵照要求] 从电压模式切换到电流模式
 *******************************************************************************
 */
#ifndef _GM6020_HPP_
#define _GM6020_HPP_

#include <stdint.h>
#include <math.h> // C 兼容

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// -----------------------------------------------------------------
// C/C++ 共享定义
// -----------------------------------------------------------------
#define GM6020_ENCODER_MAX 8191.0f
#define GM6020_RAD_PER_TICK (2.0f * M_PI / GM6020_ENCODER_MAX)

// [遵照要求] GM6020 电流模式最大值 (对应 -3A ~ +3A)
#define GM6020_CURRENT_MAX 16384

// -----------------------------------------------------------------
// C++ 专属定义
// -----------------------------------------------------------------
#ifdef __cplusplus

class GM6020 {
   public:
    GM6020(uint32_t id);
    ~GM6020() = default;

    void decode(uint8_t *data);
    
    /**
     * @brief [遵照要求] 设置电机目标控制值 (电流)
     * @param input_current 目标电流值 (-16384 to 16384)
     */
    void setInputCurrent(int16_t input_current);
    
    /**
     * @brief [遵照要求] 获取设置的目标控制值 (电流)
     */
    int16_t getInputCurrent(void);

    uint32_t getId(void);
    uint16_t getRawAngle(void);
    float getAngleRad(void);
    float getOutputAngleRad(void);
    int16_t getVelRPM(void);
    int16_t getCurrent(void);
    uint8_t getTemp(void);

   private:
    uint32_t id_;
    int16_t input_current_; // [遵照要求] 从 voltage 改为 current
    
    // 反馈数据
    uint16_t angle_raw_;
    int16_t vel_rpm_;
    int16_t current_;
    uint8_t temp_;
    
    // 累计角度
    float total_angle_rad_;
    float last_angle_rad_;
};

// 开始 C 语言链接声明
extern "C" {
#endif

// -----------------------------------------------------------------
// C 语言 API 接口
// -----------------------------------------------------------------
typedef void* GM6020Handle;

GM6020Handle GM6020_Create(uint32_t id);
void GM6020_Destroy(GM6020Handle handle);
void GM6020_Decode(GM6020Handle handle, uint8_t *data);
void GM6020_SetInputCurrent(GM6020Handle handle, int16_t input_current); // [遵照要求]
int16_t GM6020_GetInputCurrent(GM6020Handle handle); // [遵照要求]
uint32_t GM6020_GetId(GM6020Handle handle);
uint16_t GM6020_GetRawAngle(GM6020Handle handle);
float GM6020_GetAngleRad(GM6020Handle handle);
float GM6020_GetOutputAngleRad(GM6020Handle handle); 
int16_t GM6020_GetVelRPM(GM6020Handle handle);
int16_t GM6020_GetCurrent(GM6020Handle handle);
uint8_t GM6020_GetTemp(GM6020Handle handle);


#ifdef __cplusplus
} // 结束 extern "C"
#endif

#endif /* _GM6020_HPP_ */