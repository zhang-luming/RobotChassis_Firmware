#include "imu_data.h"
#include "user_config.h"
#include "MPU6050.h"
#include "inv_mpu.h"
#include "stdio.h"

/* ==================== 私有变量 ==================== */

/* IMU欧拉角原始值 */
static float pitch, roll, yaw;

/* IMU原始数据 */
static short aacx, aacy, aacz;    /* 加速度传感器原始数据 */
static short gyrox, gyroy, gyroz; /* 陀螺仪原始数据 */

/* IMU数据处理数组（数据放大100倍以保留精度）
 * 使用时需要除以100得到真实值
 */
static int16_t gyro[3];        /* 陀螺仪[x,y,z]，单位: 0.01弧度/s */
static int16_t acc[3];         /* 加速度[x,y,z]，单位: 0.01G */
static int16_t euler_angle[3]; /* 欧拉角[俯仰,横滚,航向]，数据已放大100倍 */

/* ==================== 函数实现 ==================== */

/**
 * @brief 初始化IMU模块
 */
uint8_t IMU_Init(void)
{
    /* 初始化原始数据 */
    pitch = roll = yaw = 0;
    aacx = aacy = aacz = 0;
    gyrox = gyroy = gyroz = 0;
    gyro[0] = gyro[1] = gyro[2] = 0;
    acc[0] = acc[1] = acc[2] = 0;
    euler_angle[0] = euler_angle[1] = euler_angle[2] = 0;

    /* MPU6050 IMU初始化 */
    MPU_Init();

    /* MPU DMP初始化 */
    const uint8_t result = mpu_dmp_init();

    return result;
}
/**
 * @brief 更新IMU数据
 */
void IMU_Update(void)
{
    /* 获取DMP计算后的欧拉角数据（单位：度） */
    while (mpu_dmp_get_data(&pitch, &roll, &yaw) != 0)
    {
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
    gyro[0] = (float)(gyrox * 0.001064f) * 100; /* X轴，单位：0.01 rad/s */
    gyro[1] = (float)(gyroy * 0.001064f) * 100; /* Y轴，单位：0.01 rad/s */
    gyro[2] = (float)(gyroz * 0.001064f) * 100; /* Z轴，单位：0.01 rad/s */

    /*
     * 加速度计数据转换：
     * 量程：±2G
     * 原始值范围：-32768 ~ 32767（16位有符号）
     * 灵敏度：16384 LSB / G
     * 转换目标：m/s²，并放大100倍保留精度
     */
    acc[0] = (float)(aacx / 32767.0f) * 2 * G * 100; /* X轴，单位：0.01 m/s² */
    acc[1] = (float)(aacy / 32767.0f) * 2 * G * 100; /* Y轴，单位：0.01 m/s² */
    acc[2] = (float)(aacz / 32767.0f) * 2 * G * 100; /* Z轴，单位：0.01 m/s² */

    /* 欧拉角数据放大100倍（原始单位：度） */
    euler_angle[0] = pitch * 100; /* 俯仰角，单位：0.01° */
    euler_angle[1] = roll  * 100; /* 横滚角，单位：0.01° */
    euler_angle[2] = yaw   * 100; /* 航向角，单位：0.01° */
}

/**
 * @brief 获取欧拉角（放大100倍）
 */
void IMU_GetEulerAngle(int16_t *euler)
{
    if(euler != NULL)
    {
        euler[0] = euler_angle[0];
        euler[1] = euler_angle[1];
        euler[2] = euler_angle[2];
    }
}

/**
 * @brief 获取陀螺仪数据（放大100倍）
 */
void IMU_GetGyro(int16_t *gyro_data)
{
    if(gyro_data != NULL)
    {
        gyro_data[0] = gyro[0];
        gyro_data[1] = gyro[1];
        gyro_data[2] = gyro[2];
    }
}

/**
 * @brief 获取加速度数据（放大100倍）
 */
void IMU_GetAccel(int16_t *acc_data)
{
    if(acc_data != NULL)
    {
        acc_data[0] = acc[0];
        acc_data[1] = acc[1];
        acc_data[2] = acc[2];
    }
}
