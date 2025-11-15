/**
 *******************************************************************************
 * @file      : board_comms.hpp
 * @brief     : 定义云台C板和底盘C板之间的CAN通信协议
 * @history   :
 * Version     Date            Author          Note
 * V1.0.0      2025-11-15      Gemini          1. 创建
 *******************************************************************************
 * @attention :
 * 1. 云台 -> 底盘: 0x150 (发送底盘期望速度)
 * 2. 底盘 -> 云台: 0x151 (反馈底盘状态，可选)
 *******************************************************************************
 */

#ifndef _BOARD_COMMS_HPP_
#define _BOARD_COMMS_HPP_

#include <stdint.h>

// CAN ID 定义 (使用CAN1)
#define CAN_ID_GIMBAL_TO_CHASSIS 0x150
#define CAN_ID_CHASSIS_TO_GIMBAL 0x151

/**
 * @brief 云台 -> 底盘 的CAN数据包
 * @note  发送底盘期望速度 (基于云台坐标系) 和 模式
 */
typedef struct {
    // 期望速度
    float vx; // m/s, X轴 (云台前)
    float vy; // m/s, Y轴 (云台左)
    float wz; // rad/s, Z轴 (自转)
    
    // 控制模式 (来自考核)
    // 0: 分离模式
    // 1: 跟随模式
    // 2: 小陀螺模式
    // 255: 急停 (E-Stop)
    uint8_t control_mode; 
    
    // 安全/校验
    uint8_t checksum; // 备用
    
} GimbalToChassis_t;


/**
 * @brief 底盘 -> 云台 的CAN数据包 (可选)
 * @note  反馈底盘信息，例如IMU, 里程计等
 */
typedef struct {
    float real_yaw_angle; // 底盘IMU的Yaw角 (如果需要)
    uint8_t status;
    uint8_t checksum; // 备用
} ChassisToGimbal_t;


/**
 * @brief 将 GimbalToChassis_t 结构体打包到 uint8_t[8]
 * @param msg_ptr 指向 GimbalToChassis_t 结构体
 * @param tx_data 指向8字节的CAN发送缓冲区
 */
static inline void pack_gimbal_to_chassis_msg(const GimbalToChassis_t* msg_ptr, uint8_t tx_data[8]) {
    // 使用 int16_t 来发送浮点数，提高精度
    // vx, vy: -10.0f ~ 10.0f m/s -> -20000 ~ 20000
    // wz: -10.0f ~ 10.0f rad/s -> -10000 ~ 10000
    
    int16_t vx_int = (int16_t)(msg_ptr->vx * 2000.0f);
    int16_t vy_int = (int16_t)(msg_ptr->vy * 2000.0f);
    int16_t wz_int = (int16_t)(msg_ptr->wz * 1000.0f);
    
    tx_data[0] = (vx_int >> 8) & 0xFF;
    tx_data[1] = (vx_int) & 0xFF;
    tx_data[2] = (vy_int >> 8) & 0xFF;
    tx_data[3] = (vy_int) & 0xFF;
    tx_data[4] = (wz_int >> 8) & 0xFF;
    tx_data[5] = (wz_int) & 0xFF;
    tx_data[6] = msg_ptr->control_mode;
    tx_data[7] = msg_ptr->checksum; // 备用
}

/**
 * @brief 将 uint8_t[8] 解包到 GimbalToChassis_t 结构体
 * @param rx_data 指向8字节的CAN接收缓冲区
 * @param msg_ptr 指向 GimbalToChassis_t 结构体
 */
static inline void unpack_gimbal_to_chassis_msg(const uint8_t rx_data[8], GimbalToChassis_t* msg_ptr) {
    int16_t vx_int = (int16_t)(rx_data[0] << 8 | rx_data[1]);
    int16_t vy_int = (int16_t)(rx_data[2] << 8 | rx_data[3]);
    int16_t wz_int = (int16_t)(rx_data[4] << 8 | rx_data[5]);
    
    msg_ptr->vx = (float)vx_int / 2000.0f;
    msg_ptr->vy = (float)vy_int / 2000.0f;
    msg_ptr->wz = (float)wz_int / 1000.0f;
    msg_ptr->control_mode = rx_data[6];
    msg_ptr->checksum = rx_data[7];
}


// (ChassisToGimbal 的打包/解包函数可以按需添加)

#endif /* _BOARD_COMMS_HPP_ */