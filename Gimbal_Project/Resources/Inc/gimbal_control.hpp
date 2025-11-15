/**
 *******************************************************************************
 * @file      : gimbal_control.hpp
 * @brief     : 云台C板的核心控制逻辑
 * @history   :
 * Version     Date            Author          Note
 * V1.0.0      2025-11-15      Gemini          1. 创建
 *******************************************************************************
 * @attention :
 * 1. 负责RC解析, 模式切换, 云台电机(DM4310)控制
 * 2. 负责向底盘C板发送CAN指令
 *******************************************************************************
 */
#ifndef _GIMBAL_CONTROL_HPP_
#define _GIMBAL_CONTROL_HPP_

#include "system_user.hpp"
#include "DT7.hpp"
#include "dm4310_drv.hpp"
#include "pid.hpp"

// 遥控器通道定义
#define RC_CHAN_RIGHT_X 0
#define RC_CHAN_RIGHT_Y 1
#define RC_