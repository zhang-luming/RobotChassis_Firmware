/**
 ******************************************************************************
 * @file    imu_data.h
 * @brief   IMU数据模块 - 姿态测量
 *
 * 功能说明：
 * - MPU6050 DMP姿态解算
 * - 欧拉角、陀螺仪、加速度数据采集
 * - 数据上报
 *
 * 模块接口：
 * - IMU_Init(): 初始化模块
 * - IMU_Update(): 每次调用更新传感器数据
 * - IMU_Send(): 发送所有IMU数据
 ******************************************************************************
 */

#ifndef __IMU_DATA_H
#define __IMU_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ==================== 核心接口 ==================== */

/**
 * @brief 初始化IMU模块
 * @return 0-成功, 其他-失败
 */
uint8_t IMU_Init(void);

/**
 * @brief 更新IMU数据（每10ms调用）
 *
 * 功能：
 * - 从MPU6050读取传感器数据
 * - DMP姿态解算
 * - 数据转换和存储
 *
 * 注意：需要在主循环中每次调用
 */
void IMU_Update(void);

/**
 * @brief 发送IMU数据
 *
 * 功能：
 * - 发送欧拉角
 * - 发送陀螺仪
 * - 发送加速度
 *
 * 注意：直接发送，不判断时间
 */
void IMU_Send(void);

/* ==================== 兼容性接口（临时） ==================== */

/**
 * @brief 获取欧拉角（放大100倍）
 * @deprecated 请使用 IMU_Send() 替代
 * @note 保留此函数以保持向后兼容
 */
void IMU_GetEulerAngle(int16_t *euler_angle);

/**
 * @brief 获取陀螺仪数据（放大100倍）
 * @deprecated 请使用 IMU_Send() 替代
 * @note 保留此函数以保持向后兼容
 */
void IMU_GetGyro(int16_t *gyro);

/**
 * @brief 获取加速度数据（放大100倍）
 * @deprecated 请使用 IMU_Send() 替代
 * @note 保留此函数以保持向后兼容
 */
void IMU_GetAccel(int16_t *acc);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_DATA_H */
