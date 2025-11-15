/**
*******************************************************************************
 * @file      :imu_task.cpp
* @brief     : IMU数据处理任务
* @history   :
* Version     Date            Author          Note
* V1.3.3      2025-11-15      Gemini          1. 完全仿照用户要求的 Quat2Euler
*******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "imu_task.hpp"

#include "imu.hpp"     // 包含 BMI088.hpp
#include "mahony.hpp"  // 包含 Mahony.hpp
#include "spi.h"
#include "tick.hpp"    // 用于延时
#include "string.h"    // 用于 memset
#include "math.h"      // 用于 asinf
#include "arm_math.h"  // 包含 arm_atan2_f32

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
hello_world::imu::BMI088 *bmi088_ptr = nullptr;
hello_world::ahrs::Mahony *mahony_ptr = nullptr;

float acc_data[3], gyro_data[3], temp;
float quat[4];
float euler_angles[3]; // [roll, pitch, yaw]

// 零漂校准相关
float imu_gyro_bias[3] = {0.0f, 0.0f, 0.0f};
static float gyro_calib_sum[3] = {0.0f, 0.0f, 0.0f};
static uint32_t calib_count = 0;
volatile bool imu_is_calibrated = false;

/* External variables --------------------------------------------------------*/
ImuDatas_t imu_datas = {0};

/* Private function prototypes -----------------------------------------------*/

static void Quat2Euler(float *q, float *euler);

/* -------------------------------------------------------------------------- */
/* [!!] 以下所有函数 (除 Quat2Euler 外) 均保持 V1.3.1 不变             */
/* -------------------------------------------------------------------------- */

/**
 * @brief       IMU 初始化
 */
void ImuInit() {
  hello_world::imu::BMI088HWConfig default_params = {
      .hspi = &hspi1,
      .acc_cs_port = GPIOA,
      .acc_cs_pin = GPIO_PIN_4,
      .gyro_cs_port = GPIOB,
      .gyro_cs_pin = GPIO_PIN_0,
  };
  float rot_mat_flatten[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f,
                              0.0f, 0.0f, 0.0f, 1.0f};

  float mahony_kp = 0.5f; 
  float mahony_ki = 0.0f;
  float mahony_init_quat[4] = {1.0f, 0.0f, 0.0f, 0.0f}; 
  float sample_freq = 1000.0f; 
  
  bmi088_ptr = new hello_world::imu::BMI088(default_params, rot_mat_flatten);
  
  while (bmi088_ptr->imuInit(false) != hello_world::imu::kBMI088ErrStateNoErr)
  {
      hello_world::tick::DelayUs(100000); // 延时 100ms 重试
  }

  mahony_ptr = new hello_world::ahrs::Mahony(mahony_init_quat, sample_freq,
                                             mahony_kp, mahony_ki);
  
  memset(imu_gyro_bias, 0, sizeof(imu_gyro_bias));
  memset(gyro_calib_sum, 0, sizeof(gyro_calib_sum));
  calib_count = 0;
  imu_is_calibrated = false;
}

/**
 * @brief       IMU 零漂校准 (在启动时调用)
 */
void ImuCalibrate() {
    if (bmi088_ptr == nullptr || imu_is_calibrated) return;
    
    bmi088_ptr->getData(acc_data, gyro_data, &temp);
    
    gyro_calib_sum[0] += gyro_data[0];
    gyro_calib_sum[1] += gyro_data[1];
    gyro_calib_sum[2] += gyro_data[2];
    calib_count++;
}

/**
 * @brief       IMU 零漂校准数据处理
 */
void ImuFinalizeCalibration() {
    if (bmi088_ptr == nullptr || imu_is_calibrated) return;

    if (calib_count > 0) {
        imu_gyro_bias[0] = gyro_calib_sum[0] / calib_count;
        imu_gyro_bias[1] = gyro_calib_sum[1] / calib_count;
        imu_gyro_bias[2] = gyro_calib_sum[2] / calib_count;
    }
    imu_is_calibrated = true;
}


/**
 * @brief       IMU 更新 (在 1kHz 循环中调用)
 */
void ImuUpdate() {
    if (bmi088_ptr == nullptr || mahony_ptr == nullptr) return;

    // 1. 获取 IMU 原始数据
    bmi088_ptr->getData(acc_data, gyro_data, &temp);

    // 2. 减去零漂
    gyro_data[0] -= imu_gyro_bias[0];
    gyro_data[1] -= imu_gyro_bias[1];
    gyro_data[2] -= imu_gyro_bias[2];

    // 3. 更新 Mahony 滤波器
    mahony_ptr->update(acc_data, gyro_data);

    // 4. 获取四元数和欧拉角
    mahony_ptr->getQuat(quat);
    Quat2Euler(quat, euler_angles); // euler_angles[0]=roll, [1]=pitch, [2]=yaw

    // 5. 更新全局 imu_datas 结构体
    imu_datas.acc_vals[0] = acc_data[0];
    imu_datas.acc_vals[1] = acc_data[1];
    imu_datas.acc_vals[2] = acc_data[2];
    imu_datas.gyro_vals[0] = gyro_data[0]; 
    imu_datas.gyro_vals[1] = gyro_data[1];
    imu_datas.gyro_vals[2] = gyro_data[2];
    imu_datas.euler_vals[0] = euler_angles[0]; // Roll
    imu_datas.euler_vals[1] = euler_angles[1]; // Pitch
    imu_datas.euler_vals[2] = euler_angles[2]; // Yaw
}

/* -------------------------------------------------------------------------- */
/* [!!] 以下函数已按您的要求，完全仿照 V0.9.0 进行重写              */
/* -------------------------------------------------------------------------- */

/**
 * @brief       四元数转欧拉角 (Z-Y-X 顺序)
 * @param        q: 四元数，[w, x, y, z] (q[0] = w)
 * @param        euler: 欧拉角，[roll, pitch, yaw]，单位：rad
 * @note        [按您要求] V1.3.3 版本 - 完全仿照 V0.9.0 函数结构
 * @note        [按您要求] V1.3.3 版本 - 添加了 sinp 保护
 * @note        [BUG 修复] V1.3.3 版本 - 修正了 Roll/Yaw 索引
 */
static void Quat2Euler(float *quat, float *euler) {

  // [您要求的标注] euler[0] (Roll)
  // 对应 system_user.hpp 中的 #define ROLL 0
  arm_atan2_f32(quat[0] * quat[1] + quat[2] * quat[3],
                quat[0] * quat[0] + quat[3] * quat[3] - 0.5f,
                euler + 0); // euler + 0 指向 euler[0]
  
  // [按您要求] euler[1] (Pitch)
  // 添加 sinp 保护
  float sinp = 2.0f * (quat[0] * quat[2] - quat[3] * quat[1]);
  if (sinp > 1.0f)
    sinp = 1.0f;
  else if (sinp < -1.0f)
    sinp = -1.0f;
  euler[1] = asinf(sinp);

  // [您要求的标注] euler[2] (Yaw)
  // 对应 system_user.hpp 中的 #define YAW 2
  arm_atan2_f32(quat[0] * quat[3] + quat[1] * quat[2],
                quat[0] * quat[0] + quat[1] * quat[1] - 0.5f,
                euler + 2); // euler + 2 指向 euler[2]
}