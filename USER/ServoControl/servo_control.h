/**
 ******************************************************************************
 * @file    servo_control.h
 * @brief   舵机控制模块 - PWM舵机控制
 *
 * 功能说明：
 * - 双路舵机PWM控制
 * - 舵机角度设置
 *
 * 模块接口：
 * - Servo_Init(): 初始化模块
 * - Servo_Update(): 预留接口（当前为开环控制，无需Update）
 * - Servo_Send(): 预留接口
 * - Servo_ProcessSetAngle(): 设置舵机角度
 ******************************************************************************
 */

#ifndef __SERVO_CONTROL_H
#define __SERVO_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ==================== 舵机ID定义 ==================== */
#define SERVO_ID_1  0
#define SERVO_ID_2  1

#define SERVO_COUNT 2

/* ==================== 核心接口 ==================== */

/**
 * @brief 初始化舵机控制模块
 */
void Servo_Init(void);

/**
 * @brief 更新舵机状态（预留接口）
 *
 * 功能：
 * - 舵机目前是开环控制，无需Update
 * - 预留给未来闭环控制使用
 *
 * 注意：预留接口，当前为空函数
 */
void Servo_Update(void);

/**
 * @brief 发送舵机状态（预留接口）
 *
 * 注意：预留接口，当前为空函数
 */
void Servo_Send(void);

/* ==================== 控制指令处理 ==================== */

/**
 * @brief 处理舵机角度控制指令
 * @param servo_id 舵机ID (0-1)
 * @param angle 角度值 (0-180度)
 *
 * 功能：
 * - 设置舵机角度
 * - 输出PWM
 *
 * 注意：由Comm模块在接收到舵机控制指令时调用
 */
void Servo_ProcessSetAngle(uint8_t servo_id, int16_t angle);

/* ==================== 兼容性接口（临时） ==================== */

/**
 * @brief 兼容性接口：设置舵机角度
 * @deprecated 请使用 Servo_ProcessSetAngle() 替代
 * @note 保留此函数以保持向后兼容
 */
void Servo_SetAngle(uint8_t servo_id, uint8_t angle);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_CONTROL_H */
