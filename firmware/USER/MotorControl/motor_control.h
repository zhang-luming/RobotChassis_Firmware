/**
 ******************************************************************************
 * @file    motor_control.h
 * @brief   电机控制模块 - 四路电机PID控制（统一采样点架构）
 *
 * 统一采样点架构：
 * - Motor_Update(): 统一更新函数（采样+上报+控制）
 * - Motor_SampleEncoders(): 采样编码器
 * - Motor_ReadAndReport(): 上报编码器位置
 * - Motor_UpdateControl(): 执行PID控制
 ******************************************************************************
 */

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
 * @brief 统一更新函数（推荐使用）
 *
 * 功能：
 * - 采样编码器
 * - 上报编码器位置
 * - 执行PID控制
 *
 * 优势：
 * - 一次采样，数据完全一致
 * - 简化调用代码
 * - 避免竞态条件
 *
 * 注意：
 * - 此函数在IMU中断（100Hz）中调用
 */
void Motor_Update(void);

/**
 * @brief 统一采样编码器
 *
 * 功能：
 * - 同时读取4路编码器的当前计数值
 * - 计算各编码器的增量（速度）
 *
 * 注意：
 * - 此函数在IMU中断（100Hz）中调用
 * - 采样结果存储在 g_sample_deltas[] 中
 */
void Motor_SampleEncoders(void);

/**
 * @brief 读取编码器位置并上报
 *
 * 功能：
 * - 直接读取当前编码器计数值（16位）
 * - 上报给上位机
 *
 * 说明：
 * - 利用16位编码器的自然溢出（65535→0）
 * - 上位机通过计算两次上报的增量得到速度
 * - 节省内存：8字节（4×int16）vs 16字节（4×int32拆分）
 *
 * 注意：
 * - 必须先调用 Motor_SampleEncoders()
 * - 使用 DMA 非阻塞发送
 */
void Motor_ReadAndReport(void);

/**
 * @brief 执行电机控制
 *
 * 功能：
 * - 计算时间差（自适应调用周期）
 * - 使用采样的增量执行PID控制
 * - 输出PWM
 *
 * 注意：
 * - 必须先调用 Motor_SampleEncoders()
 */
void Motor_UpdateControl(void);

/* ==================== 控制指令处理 ==================== */

/**
 * @brief 处理电机速度控制帧（从通信协议调用）
 * @param frame 帧数据指针
 * @param frame_len 帧长度
 *
 * 帧格式：[FC][0x06][速度A高][速度A低][速度B高][速度B低][速度C高][速度C低][速度D高][速度D低][Checksum][DF]
 * 索引：   0    1      2        3       4      5       6      7       8      9      10       11
 *
 * 功能：
 * - 从帧中解析4个电机的速度（小端序int16_t）
 * - 单位：CPS (Counts Per Second，每秒编码器脉冲数)
 * - 转换为编码器增量（counts/控制周期）并存储
 */
void Motor_ProcessSpeedFrame(uint8_t *frame, uint16_t frame_len);

/**
 * @brief 处理PID参数设置帧（从通信协议调用）
 * @param frame 帧数据指针
 * @param frame_len 帧长度
 *
 * 帧格式：[FC][0x05][Kp低][Kp高][Ki低][Ki高][Kd低][Kd高][时间戳8B][校验][DF]
 * 索引：  0    1     2    3     4    5     6    7     8-15    16   17
 *
 * 字节序：小端序（与速度控制帧一致）
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
