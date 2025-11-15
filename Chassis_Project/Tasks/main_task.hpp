/**
*******************************************************************************
* @file      ：main_task.hpp (Chassis)
* @brief     : 底盘C板 - 主任务头文件 (C/C++ 桥接)
*******************************************************************************
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_TASK_HPP__
#define __MAIN_TASK_HPP__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "main.h" 
#include "usart.h" // 包含 huart

/* Exported function prototypes ----------------------------------------------*/

void MainInit(void);
void MainTask_Loop(void);

// [修改] 底盘板不再需要 UART 回调
// void MainTask_UART_Callback(UART_HandleTypeDef *huart, uint16_t Size);


#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __MAIN_TASK_HPP__ */