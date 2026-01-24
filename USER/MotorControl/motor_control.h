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
 * @brief 执行电机控制（在TIM6中断中调用）
 *
 * 功能：
 * - 计算时间差（自适应调用周期）
 * - 读取编码器增量（用于PID）
 * - 执行PID控制
 * - 输出PWM
 *
 * 注意：
 * - 此函数可在较低频率下执行（如25Hz）
 * - 包含所有控制相关的计算
 * - 不涉及数据上报
 */
void Motor_UpdateControl(void);

/* ==================== 控制指令处理 ==================== */

/**
 * @brief 处理电机速度控制指令
 * @param motor_id 电机ID (0-3)
 * @param speed 目标速度（单位：centi-CPS - CPS/100）
 *
 * 功能：
 * - 设置电机目标速度
 * - 保存到内部状态
 *
 * 注意：
 * - 使用centi-CPS（CPS/100）单位以支持更大的速度范围（int16_t范围±32767）
 * - 实际CPS = 协议值 × 100
 * - 与编码器硬件配置（PPR、减速比）无关
 * - 上位机需要根据实际硬件配置转换：
 *   协议值 = RPM × (PPR × 4 × 减速比) / 60 / 100
 *   或：RPM = 协议值 × 100 / (PPR × 4 × 减速比) × 60
 *
 * 示例（PPR=500，4倍频=2000，减速比=30:1）：
 * - 目标10 RPM → 协议值 = 10 × 2000 × 30 / 60 / 100 = 100
 * - 目标50 RPM → 协议值 = 50 × 2000 × 30 / 60 / 100 = 500
 * - 目标100 RPM → 协议值 = 100 × 2000 × 30 / 60 / 100 = 1000
 *
 * 数据范围：
 * - 协议值范围：-32768 到 32767（int16_t）
 * - 对应CPS范围：-3276800 到 3276700 CPS
 * - 足够覆盖大多数应用场景
 */
void Motor_ProcessSetSpeed(uint8_t motor_id, int16_t speed);

/**
 * @brief 处理PID参数设置指令
 * @param motor_id 电机ID
 * @param kp P参数（放大100倍）
 * @param ki I参数（放大100倍）
 * @param kd D参数（放大100倍）
 *
 * 注意：由Comm模块在接收到PID配置指令时调用
 */
void Motor_ProcessSetPID(uint8_t motor_id, int16_t kp, int16_t ki, int16_t kd);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H */
