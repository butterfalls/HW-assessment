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

// -----------------------------------------------------------------
// C++ 专属定义
// -----------------------------------------------------------------
#ifdef __cplusplus

/**
 * @brief GM6020 C++ 类定义
 */
class GM6020 {
   public:
    GM6020(uint32_t id);
    ~GM6020() = default;

    /**
     * @brief 解码来自 CAN 的反馈数据
     */
    void decode(uint8_t *data);
    
    /**
     * @brief 设置电机目标控制值 (电压)
     * @param input_voltage 目标电压值 (-25000 to 25000)
     */
    void setInput(int16_t input_voltage);
    
    /**
     * @brief 获取设置的目标控制值 (电压)
     */
    int16_t getInput(void);

    uint32_t getId(void);
    uint16_t getRawAngle(void);
    
    /**
     * @brief 获取单圈电机角度 (弧度, 0 ~ 2*PI)
     */
    float getAngleRad(void);

    /**
     * @brief 获取累计电机角度 (弧度, 用于PID)
     */
    float getOutputAngleRad(void);

    int16_t getVelRPM(void);
    int16_t getCurrent(void);
    uint8_t getTemp(void);

   private:
    uint32_t id_;
    int16_t input_voltage_; // [修改] 从 float 改为 int16_t
    
    // 反馈数据
    uint16_t angle_raw_;
    int16_t vel_rpm_;
    int16_t current_;
    uint8_t temp_;
    
    // [新增] 累计角度
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
void GM6020_SetInput(GM6020Handle handle, int16_t input_voltage); // [修改]
int16_t GM6020_GetInput(GM6020Handle handle); // [修改]
uint32_t GM6020_GetId(GM6020Handle handle);
uint16_t GM6020_GetRawAngle(GM6020Handle handle);
float GM6020_GetAngleRad(GM6020Handle handle);
float GM6020_GetOutputAngleRad(GM6020Handle handle); // [新增]
int16_t GM6020_GetVelRPM(GM6020Handle handle);
int16_t GM6020_GetCurrent(GM6020Handle handle);
uint8_t GM6020_GetTemp(GM6020Handle handle);


#ifdef __cplusplus
} // 结束 extern "C"
#endif

#endif /* _GM6020_HPP_ */