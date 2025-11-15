/**
 *******************************************************************************
 * @file      : m3508.hpp
 * @brief     : M3508 (C620 电调) 电机库
 * @history   :
 * Version     Date            Author          Note
 * V1.0.0      2025-11-15      Gemini          1. 创建，基于GM6020库的风格
 * V1.1.0      2025-11-15      Gemini          2. 增加多圈角度累计
 *******************************************************************************
 * @attention :
 * 此库基于 M3508 电机 + C620 电调的 CAN 通信协议。
 * C620 反馈报文 ID: 0x200 + Motor ID (1-8)
 * C620 控制报文 ID: 0x200 (控制 1-4), 0x1FF (控制 5-8)
 *******************************************************************************
 */

#ifndef _M3508_HPP_
#define _M3508_HPP_

#include <stdint.h>
#include <math.h> // C 兼容

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// -----------------------------------------------------------------
// C/C++ 共享定义
// -----------------------------------------------------------------

// 编码器最大值 (0-8191)
#define M3508_ENCODER_MAX 8191.0f
// 减速比 (来自 M3508 PDF)
#define M3508_REDUCTION_RATIO (3591.0f / 187.0f) // 约 19.2
// 编码器原始值 转换为 弧度 (电机转子)
#define M3508_RAD_PER_TICK (2.0f * M_PI / M3508_ENCODER_MAX)
// RPM 转换为 弧度/秒
#define M3508_RPM_TO_RADS (2.0f * M_PI / 60.0f)

// -----------------------------------------------------------------
// C++ 专属定义
// -----------------------------------------------------------------
#ifdef __cplusplus

/**
 * @brief M3508 C++ 类定义
 */
class M3508 {
   public:
    /**
     * @brief M3508 构造函数
     * @param id 电机ID (C620 电调上的ID, 范围 1-8)
     */
    M3508(uint32_t id);
    ~M3508() = default;

    /**
     * @brief 从CAN总线数据中解码电机反馈
     * @param data 长度为8的CAN Rx数据缓冲区
     */
    void decode(uint8_t *data);

    /**
     * @brief 设置电机目标控制值 (电流)
     * @param input_current 目标电流值 (-16384 to 16384)
     */
    void setInputCurrent(int16_t input_current);

    /**
     * @brief 获取设置的目标控制值
     */
    int16_t getInputCurrent(void);

    /**
     * @brief 获取电机ID
     */
    uint32_t getId(void);

    /**
     * @brief 获取编码器原始值 (0-8191)
     */
    uint16_t getRawAngle(void);

    /**
     * @brief 获取电机转子角度 (弧度, 0-2PI)
     */
    float getMotorAngleRad(void);

    /**
     * @brief 获取减速后的输出轴累计角度 (弧度, 多圈)
     * @note 用于位置闭环
     */
    float getOutputTotalAngleRad(void);

    /**
     * @brief 获取电机转子速度 (RPM)
     */
    int16_t getVelRPM(void);

    /**
     * @brief 获取减速后的输出轴速度 (rad/s)
     */
    float getOutputVelRadS(void);

    /**
     * @brief 获取实际扭矩电流
     */
    int16_t getActualCurrent(void);

    /**
     * @brief 获取电机温度
     */
    uint8_t getTemp(void);

   private:
    uint32_t id_;
    int16_t input_current_; // C620 发送的是 int16_t

    // 反馈数据
    uint16_t angle_raw_;
    uint16_t last_angle_raw_;
    int16_t vel_rpm_;
    int16_t actual_current_;
    uint8_t temp_;
    
    int32_t total_round_; // 累计圈数
    float total_angle_rad_; // 累计总角度 (rad)
};

// 开始 C 语言链接声明
extern "C" {
#endif

// -----------------------------------------------------------------
// C 语言 API 接口
// -----------------------------------------------------------------
typedef void* M3508Handle;

M3508Handle M3508_Create(uint32_t id);
void M3508_Destroy(M3508Handle handle);

void M3508_Decode(M3508Handle handle, uint8_t *data);
void M3508_SetInputCurrent(M3508Handle handle, int16_t input_current);
int16_t M3508_GetInputCurrent(M3508Handle handle);

uint32_t M3508_GetId(M3508Handle handle);
uint16_t M3508_GetRawAngle(M3508Handle handle);
float M3508_GetMotorAngleRad(M3508Handle handle);
float M3508_GetOutputTotalAngleRad(M3508Handle handle);
int16_t M3508_GetVelRPM(M3508Handle handle);
float M3508_GetOutputVelRadS(M3508Handle handle);
int16_t M3508_GetActualCurrent(M3508Handle handle);
uint8_t M3508_GetTemp(M3508Handle handle);


#ifdef __cplusplus
} // 结束 extern "C"
#endif

#endif /* _M3508_HPP_ */