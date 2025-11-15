/**
*******************************************************************************
* @file      :imu_task.hpp
* @brief     : IMU 任务头文件
* @history   :
* Version     Date            Author          Note
* V1.0.0      2024-09-08      Jinletian       1. Create this file.
* V1.1.0      2025-11-15      Gemini          1. 添加零漂校准功能
*******************************************************************************
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HOMEWORK_TASKS_IMU_TASK_HPP_
#define HOMEWORK_TASKS_IMU_TASK_HPP_

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "system_user.hpp" // 引入 system_user.hpp

/* Exported macro ------------------------------------------------------------*/
// 零漂校准时间 (ms, 对应 tick)
#define IMU_CALIB_TIME 1000 

/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

extern float euler_angles[3]; // [roll, pitch, yaw] (来自 Mahony)
extern float imu_gyro_bias[3]; // [gx, gy, gz] 零漂
extern volatile bool imu_is_calibrated; // 校准完成标志

/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief       IMU 初始化
 * @note        初始化 BMI088 和 Mahony 滤波器
 */
void ImuInit();

/**
 * @brief       IMU 更新 (在 1kHz 循环中调用)
 * @note        执行零漂校准或更新姿态
 */
void ImuUpdate();

/**
 * @brief       IMU 零漂校准 (在启动时调用)
 */
void ImuCalibrate();

/**
 * @brief       IMU 零漂校准数据处理
 * @note        计算平均零漂值
 */
void ImuFinalizeCalibration();


#endif /* HOMEWORK_TASKS_IMU_TASK_HPP_ */