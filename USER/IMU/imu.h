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
 * 数据来源：
 * - 所有数据均来自DMP优化输出（FIFO）
 * - 经过传感器融合和偏差校准
 *
 * 硬件连接：
 * - I2C：PB12(SCL), PB13(SDA) - GPIO模拟I2C
 * - INT：PC12 - 外部中断（下降沿触发）
 *
 * 模块接口：
 * - IMU_Init(): 初始化IMU模块
 * - IMU_IRQHandler(): MPU INT中断处理函数
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
 * 功能：
 * - 初始化MPU6050硬件
 * - 加载并配置DMP固件
 * - 配置传感器采样率和FIFO输出率
 * - 启用陀螺仪自动校准
 */
uint8_t IMU_Init(void);

/**
 * @brief 启用/禁用IMU中断处理
 * @param enable 1-启用中断处理, 0-禁用中断处理
 *
 * 说明：
 * - 用于控制是否响应MPU INT中断
 * - 可在初始化期间禁用，主循环开始后再启用
 */
void IMU_SetEnabled(uint8_t enable);

/**
 * @brief MPU INT中断处理函数
 *
 * 调用位置：
 * - 在EXTI15_10_IRQHandler()中调用
 *
 * 功能：
 * - 从FIFO读取DMP优化数据
 * - 更新内部数据变量
 * - 通过串口发送IMU数据
 *
 * 注意：
 * - 此函数在中断上下文中执行
 * - DMP数据更新率：100Hz（10ms周期）
 */
void IMU_IRQHandler(void);

/* ==================== 数据获取接口 ==================== */

/**
 * @brief 获取欧拉角
 * @param euler_angle 欧拉角数组[俯仰,横滚,航向]，单位: 0.01°
 * @note 数据来源：DMP四元数转换
 */
void IMU_GetEulerAngle(int16_t *euler_angle);

/**
 * @brief 获取陀螺仪数据
 * @param gyro 陀螺仪数组[x,y,z]，单位: 0.01弧度/s
 * @note 数据来源：DMP FIFO（已去除零偏，经DMP校准）
 */
void IMU_GetGyro(int16_t *gyro);

/**
 * @brief 获取加速度数据
 * @param acc 加速度数组[x,y,z]，单位: 0.01G
 * @note 数据来源：DMP FIFO（原始加速度数据）
 */
void IMU_GetAccel(int16_t *acc);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_H__ */
