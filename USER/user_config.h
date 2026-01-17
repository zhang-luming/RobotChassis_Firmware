#ifndef __USER_CONFIG_H
#define __USER_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

/* ==================== 物理常数 ==================== */
#define G 9.8f

/* ==================== PID参数配置 ==================== */
#define PID_SCALE  0.01f

extern int16_t PID_P;  /* 比例系数 */
extern int16_t PID_I;  /* 积分系数 */
extern int16_t PID_D;  /* 微分系数 */

/* ==================== 编码器配置 ==================== */
#define ENCODER_MIDDLE 32767

/* ==================== 电机PWM配置 ==================== */
#define MOTOR_PWM_MAX 1439   /* PWM最大值 (对应TIM8 Period = 1439) */
#define MOTOR_PWM_FREQ 25000 /* PWM频率: 25kHz */

/* ==================== 舵机配置 ==================== */
#define SERVO_MIN_PULSE 500    /* 最小脉宽 0.5ms */
#define SERVO_MAX_PULSE 2500   /* 最大脉宽 2.5ms */
#define SERVO_MIN_ANGLE 0      /* 最小角度 0度 */
#define SERVO_MAX_ANGLE 180    /* 最大角度 180度 */

#ifdef __cplusplus
}
#endif

#endif /* __USER_CONFIG_H */
