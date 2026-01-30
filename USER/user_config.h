#ifndef __USER_CONFIG_H
#define __USER_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== PID参数配置 ==================== */
#define PID_SCALE 0.01f

/* ==================== 编码器配置 ==================== */
#define ENCODER_MIDDLE 32767
/* 注意：速度单位使用CPS（Counts Per Second），不依赖具体编码器PPR配置 */

/* ==================== 电机PWM配置 ==================== */
#define MOTOR_PWM_MAX 1439 /* PWM最大值 (对应TIM8 Period = 1439) */

/* ==================== 舵机配置 ==================== */
#define SERVO_MIN_PULSE 500  /* 最小脉宽 0.5ms */
#define SERVO_MAX_PULSE 2500 /* 最大脉宽 2.5ms */
#define SERVO_MIN_ANGLE 0    /* 最小角度 0度 */
#define SERVO_MAX_ANGLE 180  /* 最大角度 180度 */

/* ==================== 调试配置 ==================== */
#define DEBUG_ENABLE                    /* 启用调试输出（取消注释以启用） */
#define DEBUG_LEVEL DEBUG_LEVEL_INFO    /* 调试级别：INFO/WARN/ERROR/VERBOSE */

/* 调试级别说明：
 * DEBUG_LEVEL_INFO     - 只输出重要信息（系统状态、错误等）
 * DEBUG_LEVEL_WARN     - 输出警告和错误
 * DEBUG_LEVEL_ERROR    - 只输出错误
 * DEBUG_LEVEL_VERBOSE  - 输出所有调试信息（包括频繁打印的统计数据）
 */

#ifdef __cplusplus
}
#endif

#endif /* __USER_CONFIG_H */
