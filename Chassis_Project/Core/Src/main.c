/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "iwdg.h"
#include "pid.hpp"       // 包含 C 兼容头文件
#include "gm6020.hpp"    // 包含 C 兼容头文件
#include "main_task.hpp" // declare MainTask_Loop()
#include "system_user.hpp" // PID macros, vehicle geometry
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Control_Init(void);
void Control_CAN_Tx(void);
void Control_Loop(void);
void run_position_control_loop(void);
float clamp_current(float current);
// 从 C++ 模块读取计算出的目标角度 (rad)
extern volatile float g_computed_steer_angle[4];
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash  interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */                                                                              
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_TIM6_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_IWDG_Init();
  MX_CAN2_Init();
  MX_TIM10_Init();
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  Control_Init();
  MainInit();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) {
    // 1. 运行控制任务 (PID计算)
    MainTask_Loop();
    Control_Loop();

    // 2. 发送CAN控制报文
    Control_CAN_Tx();
  }
}

/* 控制初始化/循环/CAN发送实现 (仿照用户样例) */

/* Private variables ---------------------------------------------------------*/
static float g_target_pos_rad = 0.0f;
static float g_feedback_pos_rad = 0.0f;

static PidHandle g_pid_controller_1;
GM6020Handle g_motors[4]; // 4个电机

/* 每个舵轮的角度偏移修正 (rad)，用于将检测零点对齐到机械零点 */
static const float g_motor_angle_offsets[4] = {
  -0.52f,  /* g_motors[0] 加 0.52 */
  +1.08f, /* g_motors[1] 减 1.08 */
  -0.08f,  /* g_motors[2] 加 0.08 */
  -1.74f   /* g_motors[3] 加 1.74 */
};

// --- CAN 发送缓冲区 ---
static uint8_t g_can_tx_msg_1ff[8]; // 用于ID 1-4 (共享缓冲区，供 0x1FE/0x2FE 等复用)

/* 共享 CAN TX header / mailbox，便于将 GM6020 的 0x1FE 报文与其它发送逻辑合并 */
static CAN_TxHeaderTypeDef g_tx_header_1fe;
static uint32_t g_tx_mailbox_1fe;

// --- PID 参数 (C 结构体) ---
static PidParams g_position_pid_params = {
  .Kp = 2200.0f,
  .Ki = 200.0f,
  .Kd = 200.0f,
  .max_out = 16384.0f,
  .max_integral = 5000.0f,
  .deadband = 0.002f,
  .integral_range = 0.5f,
  .d_filter_gain = 0.8f,
  .dt = 0.001f
};

/* 舵向 PID: 为 4 个舵轮创建独立的 PID 实例，使用 g_position_pid_params 作为参数 */
static PidHandle g_steer_pids_c[4] = {0, 0, 0, 0};

/* 暴露给调试器的舵向 PID 输出 (控制电流)，每个轮一个值 (rad->current) */
volatile float g_steer_output_c[4] = {0.0f, 0.0f, 0.0f, 0.0f};

/* Control functions (copied/adapted) */
void Control_Init(void)
{
  int i;
  // 1. 创建所有电机对象
  for (i = 0; i < 4; i++) {
    g_motors[i] = GM6020_Create(i + 1);
  }

  // 2. 根据任务创建PID控制器 (默认位置任务)
  g_pid_controller_1 = Pid_Create(&g_position_pid_params);

  // 额外：为舵向电机创建 4 个 PID（使用 g_position_pid_params）
  for (i = 0; i < 4; i++) {
    g_steer_pids_c[i] = Pid_Create(&g_position_pid_params);
  }

  // 3. 配置CAN过滤器
  CAN_FilterTypeDef filter_config;
  filter_config.FilterBank = 0;
  filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
  filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
  filter_config.FilterIdHigh = 0x0000;
  filter_config.FilterIdLow = 0x0000;
  filter_config.FilterMaskIdHigh = 0x0000;
  filter_config.FilterMaskIdLow = 0x0000;
  filter_config.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter_config.FilterActivation = ENABLE;
  filter_config.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &filter_config) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
    Error_Handler();
  }

  /* Ensure CAN2 is started and FIFO1 notification is enabled so HAL_CAN_RxFifo1MsgPendingCallback is invoked */
  if (HAL_CAN_Start(&hcan2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) {
    Error_Handler();
  }

}

/* 用户任务：速度闭环 (已移除) */

/* 用户任务：位置闭环 */
void run_position_control_loop(void)
{
  static uint32_t counter_ms = 0;
  static uint8_t state = 0;
  counter_ms++;

  if (counter_ms % 2000 == 0) {
    state = (state + 1) % 6;
    switch (state) {
      case 0: g_target_pos_rad = -5.0f * M_PI / 6.0f; break;
      case 1: g_target_pos_rad = 5.0f * M_PI / 6.0f; break;
      case 2: g_target_pos_rad = M_PI / 3.0f; break;
      case 3: g_target_pos_rad = 2.0f * M_PI / 3.0f; break;
      case 4: g_target_pos_rad = M_PI / 4.0f; break;
      case 5: g_target_pos_rad = -M_PI; break;
    }
  }

  if (g_motors[0]) {
    g_feedback_pos_rad = GM6020_GetAngleRad(g_motors[0]);
    /* Apply mechanical zero offset for motor 0 */
    g_feedback_pos_rad += g_motor_angle_offsets[0];
    /* normalize to [0, 2PI) */
    while (g_feedback_pos_rad < 0.0f) g_feedback_pos_rad += 2.0f * M_PI;
    while (g_feedback_pos_rad >= 2.0f * M_PI) g_feedback_pos_rad -= 2.0f * M_PI;
    float output_current = Pid_CalcAngle(g_pid_controller_1, g_target_pos_rad, g_feedback_pos_rad);
    GM6020_SetInput(g_motors[0], output_current);
  }
}

/* Control loop called from TIM6 */
void Control_Loop(void)
{
  // 默认运行位置控制任务
  Pid_SetParams(g_pid_controller_1, &g_position_pid_params);
  // run_position_control_loop();
}

float clamp_current(float current) {
  if (current > 16384.0f) return 16384.0f;
  if (current < -16384.0f) return -16384.0f;
  return current;
}


/* CAN TX: pack GM6020 SetInput values into 0x1FE / 0x2FE frames */
void Control_CAN_Tx(void)
{
  int i;

  /* 使用文件范围共享的 header/mailbox，便于与其它发送路径共用同一组变量 */
  g_tx_header_1fe.StdId = 0x1FE;
  g_tx_header_1fe.IDE = CAN_ID_STD;
  g_tx_header_1fe.RTR = CAN_RTR_DATA;
  g_tx_header_1fe.DLC = 8;
  g_tx_header_1fe.TransmitGlobalTime = DISABLE;

  for (i = 0; i < 4; i++) {
    /* 1) 从 C++ 模块读取分解出的目标角度 (rad) */
    float target_angle = g_computed_steer_angle[i];
    /* 2) 读取当前物理角度 (rad) 并应用检测零点偏移修正 */
    float current_angle = GM6020_GetAngleRad(g_motors[i]);
    /* apply per-motor mechanical zero offset */
    current_angle += g_motor_angle_offsets[i];
    /* normalize to [0, 2PI) to keep angle in expected range */
    while (current_angle < 0.0f) current_angle += 2.0f * M_PI;
    while (current_angle >= 2.0f * M_PI) current_angle -= 2.0f * M_PI;
    /* 3) 运行角度 PID，输出控制量（电流） */
    if (g_steer_pids_c[i]) {
      Pid_SetParams(g_steer_pids_c[i], &g_position_pid_params);
    float steer_output = Pid_CalcAngle(g_steer_pids_c[i], target_angle, current_angle);
    /* 保存到全局可观察变量，便于调试 */
    g_steer_output_c[i] = target_angle;
    /* 把 PID 输出写回 motor input，后续打包发送 */
    GM6020_SetInput(g_motors[i], steer_output);
    }
    /* 4) 打包为 int16 并写入 CAN 缓冲区 */
    int16_t voltage = (int16_t)clamp_current(GM6020_GetInput(g_motors[i]));
    g_can_tx_msg_1ff[i * 2] = (voltage >> 8) & 0xFF;
    g_can_tx_msg_1ff[i * 2 + 1] = (voltage & 0xFF);
  }

  HAL_CAN_AddTxMessage(&hcan1, &g_tx_header_1fe, g_can_tx_msg_1ff, &g_tx_mailbox_1fe);
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
