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

/* ==================== 函数接口 ==================== */

/**
 * @brief 初始化舵机控制模块
 */
void Servo_Init(void);

/**
 * @brief 设置舵机角度
 * @param servo_id 舵机ID (0-1)
 * @param angle 角度 (0-180度)
 */
void Servo_SetAngle(uint8_t servo_id, uint8_t angle);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_CONTROL_H */
