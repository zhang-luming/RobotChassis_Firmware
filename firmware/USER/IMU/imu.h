/**
 ******************************************************************************
 * @file    imu.h
 * @brief   IMU模块 - MPU6050 DMP姿态解算
 *
 * 功能说明：
 * - MPU6050 DMP姿态解算（6轴传感器融合）
 * - 欧拉角、陀螺仪、加速度数据采集和上报
 * - MPU INT中断处理
 *
 * 数据来源（双通道保留用于对比）：
 * - 欧拉角：DMP四元数解算（6轴传感器融合）
 * - 陀螺仪：
 *   - 寄存器原始数据（未去除零偏）
 *   - DMP校准数据（零偏已自动消除，由DMP_FEATURE_GYRO_CAL提供）
 * - 加速度：
 *   - 寄存器原始数据
 *   - DMP原始数据
 *
 * 数据发送配置：
 * - 在 imu.c 中通过 IMU_USE_DMP_CALIBRATED_DATA 宏选择发送数据源
 * - =1: 发送DMP校准数据（推荐，陀螺仪零偏已消除）
 * - =0: 发送寄存器原始数据
 *
 * 硬件连接：
 * - I2C：PB12(SCL), PB13(SDA) - GPIO模拟I2C
 * - INT：PC12 - 外部中断（下降沿触发）
 ******************************************************************************
 */

#ifndef __IMU_H__
#define __IMU_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ==================== 核心接口 ==================== */

/**
 * @brief 初始化IMU模块
 * @return 0-成功, 其他-失败
 *
 * 初始化MPU6050硬件、加载DMP固件、配置传感器采样率
 */
uint8_t IMU_Init(void);

/**
 * @brief 启用/禁用IMU中断处理
 * @param enable 1-启用, 0-禁用
 */
void IMU_SetEnabled(uint8_t enable);

/**
 * @brief MPU INT中断处理函数（100Hz）
 *
 * 从FIFO读取DMP数据并上报，在EXTI15_10_IRQHandler()中调用
 */
void IMU_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_H__ */
