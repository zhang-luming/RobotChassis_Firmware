/**
 ******************************************************************************
 * @file    imu_data.c
 * @brief   IMU数据模块 - 姿态测量
 *
 * 功能说明：
 * - MPU6050 DMP姿态解算
 * - 欧拉角、陀螺仪、加速度数据采集
 * - 数据上报
 ******************************************************************************
 */

#include "imu_data.h"
#include "comm_protocol.h"
#include "user_config.h"
#include "mpu6050.h"
#include "inv_mpu.h"

/* ==================== 私有变量 ==================== */

/* IMU欧拉角原始值 */
static float pitch, roll, yaw;

/* IMU原始数据 */
static short aacx, aacy, aacz;    /* 加速度传感器原始数据 */
static short gyrox, gyroy, gyroz; /* 陀螺仪原始数据 */

/* IMU数据处理数组（数据放大100倍以保留精度）
 * 使用时需要除以100得到真实值
 */
static int16_t g_gyro[3];         /* 陀螺仪[x,y,z]，单位: 0.01弧度/s */
static int16_t g_acc[3];          /* 加速度[x,y,z]，单位: 0.01G */
static int16_t g_euler_angle[3];  /* 欧拉角[俯仰,横滚,航向]，数据已放大100倍 */

/* ==================== 公共接口实现 ==================== */

/**
 * @brief 初始化IMU模块
 */
uint8_t IMU_Init(void) {
    /* 初始化原始数据 */
    pitch = roll = yaw = 0;
    aacx = aacy = aacz = 0;
    gyrox = gyroy = gyroz = 0;
    g_gyro[0] = g_gyro[1] = g_gyro[2] = 0;
    g_acc[0] = g_acc[1] = g_acc[2] = 0;
    g_euler_angle[0] = g_euler_angle[1] = g_euler_angle[2] = 0;

    /* MPU6050 IMU初始化 */
    MPU_Init();

    /* MPU DMP初始化 */
    const uint8_t result = mpu_dmp_init();

    return result;
}

/**
 * @brief 更新IMU数据（每10ms调用）
 *
 * 功能：
 * 1. 获取DMP计算后的欧拉角数据
 * 2. 获取陀螺仪和加速度计原始数据
 * 3. 进行数据转换和单位换算
 * 4. 存储到内部变量
 */
void IMU_Update(void) {
    /* 获取DMP计算后的欧拉角数据（单位：度） */
    while (mpu_dmp_get_data(&pitch, &roll, &yaw) != 0) {
        /* 等待DMP数据有效 */
    }

    /* 获取陀螺仪原始数据（ADC原始计数值） */
    MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);

    /* 获取加速度计原始数据（ADC原始计数值） */
    MPU_Get_Accelerometer(&aacx, &aacy, &aacz);

    /*
     * 陀螺仪数据转换：
     * 量程：±2000 dps
     * 原始值范围：-32768 ~ 32767
     * 转换目标：弧度/s，并放大100倍保留精度
     *
     * 推导：
     * 2000 dps / 32767 ≈ 0.061 dps/LSB(灵敏度)
     * 0.061 * (π / 180) ≈ 0.001064 rad/s/LSB
     */
    g_gyro[0] = (float)(gyrox * 0.001064f) * 100; /* X轴，单位：0.01 rad/s */
    g_gyro[1] = (float)(gyroy * 0.001064f) * 100; /* Y轴，单位：0.01 rad/s */
    g_gyro[2] = (float)(gyroz * 0.001064f) * 100; /* Z轴，单位：0.01 rad/s */

    /*
     * 加速度计数据转换：
     * 量程：±2G
     * 原始值范围：-32768 ~ 32767（16位有符号）
     * 灵敏度：16384 LSB / G
     * 转换目标：m/s²，并放大100倍保留精度
     */
    g_acc[0] = (float)(aacx / 32767.0f) * 2 * G * 100; /* X轴，单位：0.01 m/s² */
    g_acc[1] = (float)(aacy / 32767.0f) * 2 * G * 100; /* Y轴，单位：0.01 m/s² */
    g_acc[2] = (float)(aacz / 32767.0f) * 2 * G * 100; /* Z轴，单位：0.01 m/s² */

    /* 欧拉角数据放大100倍（原始单位：度） */
    g_euler_angle[0] = pitch * 100; /* 俯仰角，单位：0.01° */
    g_euler_angle[1] = roll  * 100; /* 横滚角，单位：0.01° */
    g_euler_angle[2] = yaw   * 100; /* 航向角，单位：0.01° */
}

/**
 * @brief 发送IMU数据
 *
 * 直接发送所有IMU数据到上位机，不判断时间
 */
void IMU_Send(void) {
    /* 发送欧拉角 */
    Comm_SendEulerAngle(g_euler_angle);

    /* 发送陀螺仪 */
    Comm_SendGyro(g_gyro);

    /* 发送加速度 */
    Comm_SendAccel(g_acc);
}

/* ==================== 兼容性接口（临时） ==================== */

/**
 * @brief 兼容性接口：获取欧拉角
 * @deprecated 请使用 IMU_Send() 替代
 */
void IMU_GetEulerAngle(int16_t *euler) {
    if (euler != NULL) {
        euler[0] = g_euler_angle[0];
        euler[1] = g_euler_angle[1];
        euler[2] = g_euler_angle[2];
    }
}

/**
 * @brief 兼容性接口：获取陀螺仪数据
 * @deprecated 请使用 IMU_Send() 替代
 */
void IMU_GetGyro(int16_t *gyro_data) {
    if (gyro_data != NULL) {
        gyro_data[0] = g_gyro[0];
        gyro_data[1] = g_gyro[1];
        gyro_data[2] = g_gyro[2];
    }
}

/**
 * @brief 兼容性接口：获取加速度数据
 * @deprecated 请使用 IMU_Send() 替代
 */
void IMU_GetAccel(int16_t *acc_data) {
    if (acc_data != NULL) {
        acc_data[0] = g_acc[0];
        acc_data[1] = g_acc[1];
        acc_data[2] = g_acc[2];
    }
}
