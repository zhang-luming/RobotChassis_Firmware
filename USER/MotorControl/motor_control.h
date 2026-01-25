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

/* ==================== 核心接口 ==================== */

/**
 * @brief 初始化电机控制模块
 */
void Motor_Init(void);

/**
 * @brief 读取编码器位置并上报（在IMU中断中调用）
 *
 * 功能：
 * - 读取4路编码器的当前累加位置
 * - 通过串口上报给上位机
 *
 * 注意：
 * - 此函数在IMU中断（100Hz）中调用
 * - 与IMU数据保持时间同步
 * - 只读取和上报，不执行任何控制逻辑
 */
void Motor_ReadAndReport(void);

/**
 * @brief 执行电机控制
 *
 * 功能：
 * - 计算时间差（自适应调用周期）
 * - 读取编码器增量（用于PID）
 * - 执行PID控制
 * - 输出PWM
 *
 */
void Motor_UpdateControl(void);

/* ==================== 控制指令处理 ==================== */

/**
 * @brief 处理电机速度控制帧（从通信协议调用）
 * @param frame 帧数据指针
 * @param frame_len 帧长度
 *
 * 帧格式：[FC][0x06][速度A高][速度A低][速度B高][速度B低][速度C高][速度C低][速度D高][速度D低][Checksum][DF]
 *
 * 功能：
 * - 从通信帧中解析4个电机的速度
 * - 单位：centi-CPS（CPS/100）
 * - 转换为编码器增量并存储
 */
void Motor_ProcessSpeedFrame(uint8_t *frame, uint16_t frame_len);

/**
 * @brief 处理PID参数设置帧（从通信协议调用）
 * @param frame 帧数据指针
 * @param frame_len 帧长度
 *
 * 帧格式：[FC][0x07][Kp高][Kp低][Ki高][Ki低][Kd高][Kd低][Checksum][DF]
 *
 * 功能：
 * - 从通信帧中解析PID参数
 * - 为所有4个电机设置相同的PID参数
 */
void Motor_ProcessPIDFrame(uint8_t *frame, uint16_t frame_len);

/**
 * @brief 处理PID参数设置指令
 * @param motor_id 电机ID
 * @param kp P参数（放大100倍）
 * @param ki I参数（放大100倍）
 * @param kd D参数（放大100倍）
 */
void Motor_ProcessSetPID(uint8_t motor_id, int16_t kp, int16_t ki, int16_t kd);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H */
