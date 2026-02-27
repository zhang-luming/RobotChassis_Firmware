/**
 ******************************************************************************
 * @file    imu.c
 * @brief   IMU模块 - MPU6050 DMP姿态解算实现
 *
 * 功能说明：
 * - MPU6050 DMP姿态解算
 * - 欧拉角、陀螺仪、加速度数据采集
 * - MPU INT中断处理
 * - 数据上报
 *
 * 优化说明：
 * - 使用整数运算代替浮点运算
 * - 预计算转换系数，减少计算量
 * - 减少中断处理时间
 ******************************************************************************
 */

#include "imu.h"
#include "comm_protocol.h"
#include "user_config.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "timer.h"
#include "System/debug.h"
#include "MotorControl/motor_control.h"

/* ==================== 私有常量定义 ==================== */

/*
 * 转换系数（预计算，避免浮点运算）
 *
 * 陀螺仪转换：
 *   量程±2000dps，灵敏度0.061 dps/LSB
 *   弧度转换：0.061 * π / 180 = 0.001064 rad/s/LSB
 *   放大100倍：0.001064 * 100 = 0.1064
 *   整数运算：raw * 1064 / 10000 = raw * 266 / 2500
 *
 * 加速度计转换：
 *   量程±2G，灵敏度16384 LSB/G
 *   m/s²转换：raw / 32767 * 2 * 9.8 * 100
 *   简化：raw * 1960 / 32767 ≈ raw * 60 / 1000
 */
#define GYRO_SCALE_NUM    266      /* 陀螺仪缩放分子 */
#define GYRO_SCALE_DEN    2500     /* 陀螺仪缩放分母 */
#define ACC_SCALE_NUM     60       /* 加速度缩放分子 */
#define ACC_SCALE_DEN     1000     /* 加速度缩放分母 */

/* ==================== 私有变量 ==================== */

/* IMU数据（DMP优化输出） */
static float s_pitch, s_roll, s_yaw;        /* 欧拉角原始值（度） */
static int16_t s_gyro[3];                   /* 陀螺仪[x,y,z]，单位: 0.01弧度/s */
static int16_t s_acc[3];                    /* 加速度[x,y,z]，单位: 0.01G */
static int16_t s_euler_angle[3];            /* 欧拉角[俯仰,横滚,航向]，单位: 0.01° */

/* 数据有效标志 */
static uint8_t s_data_valid = 0;            /* 数据有效标志 */

/* 中断处理控制 */
static uint8_t s_irq_enabled = 0;           /* 中断处理使能标志 */
static uint32_t s_irq_count = 0;            /* 中断计数器 */
static uint64_t s_last_irq_timestamp_us = 0; /* 上次中断时间戳 */

/* ==================== 私有函数声明 ==================== */

/**
 * @brief 更新并发布IMU数据
 *
 * 功能：
 * - 从DMP FIFO读取姿态数据
 * - 从寄存器读取传感器原始数据并转换
 * - 通过串口发送所有数据
 *
 * 注意：
 * - 此函数在中断上下文中执行
 * - 使用整数运算优化性能
 * - 一次性发送所有数据，减少串口调用次数
 */
static void IMU_UpdateAndPublish(void);

/* ==================== 公共接口实现 ==================== */

/**
 * @brief 初始化IMU模块
 */
uint8_t IMU_Init(void) {
    /* 初始化数据 */
    s_pitch = s_roll = s_yaw = 0;
    s_gyro[0] = s_gyro[1] = s_gyro[2] = 0;
    s_acc[0] = s_acc[1] = s_acc[2] = 0;
    s_euler_angle[0] = s_euler_angle[1] = s_euler_angle[2] = 0;
    s_data_valid = 0;
    s_irq_enabled = 0;
    s_irq_count = 0;
    s_last_irq_timestamp_us = 0;

    /* MPU6050 IMU初始化 */
    MPU_Init();

    /* MPU DMP初始化 */
    const uint8_t result = mpu_dmp_init();

    if (result == 0) {
        DEBUG_INFO("IMU模块初始化完成（初始禁用中断）\r\n");
    }

    return result;
}

/**
 * @brief 启用/禁用IMU中断处理
 */
void IMU_SetEnabled(uint8_t enable) {
    s_irq_enabled = enable;
    if (enable) {
        DEBUG_INFO("IMU中断处理已启用\r\n");
    }
}

/**
 * @brief MPU INT中断处理函数
 */
void IMU_IRQHandler(void) {
    /* 快速退出：未启用 */
    if (!s_irq_enabled) {
        return;
    }

    /* 获取时间戳和计数 */
    uint64_t current_timestamp = Time_GetUs();
    s_irq_count++;

#ifdef DEBUG_ENABLE
    /* 每100次中断打印一次状态 */
    if ((s_irq_count % 100) == 0) {
        uint64_t interval = current_timestamp - s_last_irq_timestamp_us;
        DEBUG_INFO("[IMU] IRQ#%lu Int:%lluus\r\n", s_irq_count, interval);
    }
#endif
    s_last_irq_timestamp_us = current_timestamp;

    /* 读取并上报IMU数据 */
    IMU_UpdateAndPublish();

    /* 读取并上报编码器位置 */
    Motor_ReadAndReport();
}

/**
 * @brief GPIO外部中断回调（HAL库弱函数）
 * @param GPIO_Pin 触发中断的GPIO引脚
 *
 * 处理的中断：
 * - PC12 (MPU_INT): MPU6050 DMP数据就绪
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == MPU_INT_Pin) {
        IMU_IRQHandler();
    }
}

/* ==================== 私有函数实现 ==================== */

/**
 * @brief 更新并发布IMU数据（优化版本 - 使用FIFO中的DMP校准后数据）
 */
static void IMU_UpdateAndPublish(void) {
    short dmp_gyro[3], dmp_acc[3];
    int32_t temp;
    uint8_t i;

    /* ========== 1. 从FIFO读取DMP数据（姿态+传感器） ========== */
    if (mpu_dmp_get_data(&s_pitch, &s_roll, &s_yaw,
                         dmp_gyro, dmp_acc) != 0) {
        return;  /* DMP数据未就绪，直接返回 */
    }

    /* ========== 2. 转换姿态角 ========== */
    s_euler_angle[0] = (int16_t)(s_pitch * 100);
    s_euler_angle[1] = (int16_t)(s_roll  * 100);
    s_euler_angle[2] = (int16_t)(s_yaw   * 100);

    /* ========== 3. 转换传感器数据（使用来自FIFO的数据） ========== */
    /* 陀螺仪：DMP已校准，直接使用相同的转换公式 */
    for (i = 0; i < 3; i++) {
        temp = ((int32_t)dmp_gyro[i] * GYRO_SCALE_NUM) / GYRO_SCALE_DEN;
        s_gyro[i] = (int16_t)temp;
    }

    /* 加速度：来自FIFO的原始数据 */
    for (i = 0; i < 3; i++) {
        temp = ((int32_t)dmp_acc[i] * ACC_SCALE_NUM) / ACC_SCALE_DEN;
        s_acc[i] = (int16_t)temp;
    }

    s_data_valid = 1;

    /* ========== 4. 发送所有数据 ========== */
    Comm_SendDataFrame(FUNC_EULER_ANGLE, s_euler_angle, 3);
    Comm_SendDataFrame(FUNC_GYRO, s_gyro, 3);
    Comm_SendDataFrame(FUNC_ACCEL, s_acc, 3);
}

/* ==================== 数据获取接口实现 ==================== */

/**
 * @brief 获取欧拉角
 */
void IMU_GetEulerAngle(int16_t *euler_angle) {
    if (euler_angle != NULL && s_data_valid) {
        euler_angle[0] = s_euler_angle[0];
        euler_angle[1] = s_euler_angle[1];
        euler_angle[2] = s_euler_angle[2];
    }
}

/**
 * @brief 获取陀螺仪数据
 */
void IMU_GetGyro(int16_t *gyro) {
    if (gyro != NULL && s_data_valid) {
        gyro[0] = s_gyro[0];
        gyro[1] = s_gyro[1];
        gyro[2] = s_gyro[2];
    }
}

/**
 * @brief 获取加速度数据
 */
void IMU_GetAccel(int16_t *acc) {
    if (acc != NULL && s_data_valid) {
        acc[0] = s_acc[0];
        acc[1] = s_acc[1];
        acc[2] = s_acc[2];
    }
}
