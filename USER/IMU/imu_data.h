#ifndef __IMU_DATA_H
#define __IMU_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ==================== 函数接口 ==================== */

/**
 * @brief 初始化IMU模块
 * @return 0-成功, 其他-失败
 */
uint8_t IMU_Init(void);

/**
 * @brief 更新IMU数据
 * @note 在主循环中定期调用
 */
void IMU_Update(void);

/**
 * @brief 获取欧拉角（放大100倍）
 * @param euler_angle 欧拉角数组[俯仰,横滚,航向]
 */
void IMU_GetEulerAngle(int16_t *euler_angle);

/**
 * @brief 获取陀螺仪数据（放大100倍）
 * @param gyro 陀螺仪数组[x,y,z]，单位: 0.01弧度/s
 */
void IMU_GetGyro(int16_t *gyro);

/**
 * @brief 获取加速度数据（放大100倍）
 * @param acc 加速度数组[x,y,z]，单位: 0.01G
 */
void IMU_GetAccel(int16_t *acc);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_DATA_H */
