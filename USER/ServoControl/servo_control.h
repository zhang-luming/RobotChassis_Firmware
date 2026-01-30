/**
 ******************************************************************************
 * @file    servo_control.h
 * @brief   舵机控制模块 - PWM舵机控制
 *
 * 功能说明：
 * - 双路舵机PWM控制
 * - 舵机角度设置
 * - 预留平滑移动、状态查询等扩展接口
 *
 * 核心接口：
 * - Servo_Init(): 初始化模块
 * - Servo_ProcessSetAngle(): 设置舵机角度（立即执行）
 ******************************************************************************
 */

#ifndef __SERVO_CONTROL_H
#define __SERVO_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ==================== 舵机配置 ==================== */
#define SERVO_ID_1  0
#define SERVO_ID_2  1
#define SERVO_COUNT 2

/* ==================== 核心接口 ==================== */

/**
 * @brief 初始化舵机控制模块
 */
void Servo_Init(void);

/**
 * @brief 处理舵机角度控制指令
 * @param servo_id 舵机ID (0-1)
 * @param angle 角度值 (0-180度)
 *
 * 功能：立即设置舵机到目标角度
 */
void Servo_ProcessSetAngle(uint8_t servo_id, int16_t angle);

/* ==================== 扩展接口（预留） ==================== */

/**
 * @brief 平滑移动舵机到目标角度（预留接口）
 * @param servo_id 舵机ID
 * @param angle 目标角度 (0-180)
 * @param speed 移动速度 (度/秒)
 *
 * 功能：以指定速度平滑移动到目标位置
 * 状态：未实现，预留接口
 */
void Servo_MoveTo(uint8_t servo_id, int16_t angle, uint16_t speed);

/**
 * @brief 获取舵机当前角度（预留接口）
 * @param servo_id 舵机ID
 * @return 当前角度 (0-180)
 *
 * 功能：返回舵机当前角度
 * 状态：未实现，预留接口
 */
int16_t Servo_GetAngle(uint8_t servo_id);

/**
 * @brief 使能/禁用舵机（预留接口）
 * @param servo_id 舵机ID
 * @param enable 1-使能, 0-禁用
 *
 * 功能：禁用时停止PWM输出，节省电源
 * 状态：未实现，预留接口
 */
void Servo_SetEnable(uint8_t servo_id, uint8_t enable);

/**
 * @brief 舵机复位到中位（预留接口）
 * @param servo_id 舵机ID
 *
 * 功能：快速移动到90度中位
 * 状态：未实现，预留接口
 */
void Servo_ResetToCenter(uint8_t servo_id);

/**
 * @brief 设置角度校准偏移（预留接口）
 * @param servo_id 舵机ID
 * @param offset 校准偏移量 (度)
 *
 * 功能：修正机械装配误差
 * 状态：未实现，预留接口
 */
void Servo_SetCalibration(uint8_t servo_id, int16_t offset);

/**
 * @brief 多舵机同步控制（预留接口）
 * @param angles 角度数组
 * @param count 舵机数量
 *
 * 功能：同时设置多个舵机角度
 * 状态：未实现，预留接口
 */
void Servo_SetMultiple(int16_t *angles, uint8_t count);

/**
 * @brief 更新舵机状态（预留接口）
 *
 * 功能：用于平滑移动等需要定时更新的功能
 * 状态：未实现，预留接口
 */
void Servo_Update(void);

/**
 * @brief 发送舵机状态（预留接口）
 *
 * 功能：上报舵机当前状态
 * 状态：未实现，预留接口
 */
void Servo_Send(void);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_CONTROL_H */
