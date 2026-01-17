#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ==================== 电机ID定义 ==================== */
#define MOTOR_ID_A  0
#define MOTOR_ID_B  1
#define MOTOR_ID_C  2
#define MOTOR_ID_D  3

#define MOTOR_COUNT 4

/* ==================== 函数接口 ==================== */

/**
 * @brief 初始化电机控制模块
 */
void Motor_Init(void);

/**
 * @brief 设置电机目标速度
 * @param motor_id 电机ID (0-3)
 * @param speed 目标速度
 */
void Motor_SetTargetSpeed(uint8_t motor_id, int16_t speed);

/**
 * @brief 电机PID控制
 * @param motor_id 电机ID (0-3)
 * @param ideal_speed 理想速度
 * @param actual_speed 实际速度
 * @return PWM输出值
 */
int16_t Motor_PIDControl(uint8_t motor_id, int16_t ideal_speed, int16_t actual_speed);

/**
 * @brief 设置电机PWM输出
 * @param motor_id 电机ID (0-3)
 * @param pwm PWM值
 */
void Motor_SetSpeed(uint8_t motor_id, int16_t pwm);

/**
 * @brief 读取编码器相对变化值并更新累加值
 */
void Motor_UpdateEncoderDelta(void);

/**
 * @brief 获取编码器累加值
 * @param encoder_values 编码器数组[4]
 */
void Motor_GetEncoder(int16_t *encoder_values);

/**
 * @brief 重置编码器计数器到中间值
 */
void Motor_ResetEncoder(void);

/**
 * @brief 获取编码器目标速度数组
 * @return 目标速度数组指针
 */
int16_t* Motor_GetTargetSpeedArray(void);

/**
 * @brief 获取编码器相对变化值数组
 * @return 相对变化值数组指针
 */
int16_t* Motor_GetEncoderDeltaArray(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H */
