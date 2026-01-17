#ifndef __COMM_PROTOCOL_H
#define __COMM_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ==================== 协议定义 ==================== */
#define PROTOCOL_HEADER   0xFC  /* 帧头 */
#define PROTOCOL_TAIL     0xDF  /* 帧尾 */

/* 功能码定义 */
#define FUNC_BATTERY_VOLTAGE  0x01  /* 电池电压 */
#define FUNC_ENCODER          0x02  /* 编码器 */
#define FUNC_GYRO             0x03  /* 陀螺仪 */
#define FUNC_ACCEL            0x04  /* 加速度 */
#define FUNC_EULER_ANGLE      0x05  /* 欧拉角 */
#define FUNC_MOTOR_SPEED      0x06  /* 电机目标速度 */
#define FUNC_PID_PARAM        0x07  /* PID参数设置 */
#define FUNC_SERVO_CONTROL    0x08  /* 舵机控制 */

#define PROTOCOL_DATA_LEN     10    /* 数据长度 */
#define RX_BUFFER_SIZE        256   /* 接收缓冲区大小 */

/* ==================== 类型定义 ==================== */

/* 串口接收状态 */
typedef struct {
    uint8_t buffer[RX_BUFFER_SIZE];  /* 接收缓冲区 */
    uint8_t index;                   /* 当前接收索引 */
    uint8_t byte;                    /* 单字节接收缓冲 */
    uint8_t complete;                /* 接收完成标志 */
} CommRx_t;

/* 串口发送缓冲 */
extern uint8_t UART_SEND_BUF[20];

/* ==================== 函数接口 ==================== */

/**
 * @brief 初始化通信模块
 */
void Comm_Init(void);

/**
 * @brief 处理接收到的控制数据
 * @note 在主循环中调用
 */
void Comm_ProcessControlData(void);

/**
 * @brief 发送欧拉角数据
 * @param euler_angle 欧拉角数组[俯仰,横滚,航向]，数据已放大100倍
 */
void Comm_SendEulerAngle(int16_t *euler_angle);

/**
 * @brief 发送陀螺仪数据
 * @param gyro 陀螺仪数组[x,y,z]，单位: 0.01弧度/s
 */
void Comm_SendGyro(int16_t *gyro);

/**
 * @brief 发送加速度数据
 * @param acc 加速度数组[x,y,z]，单位: 0.01G
 */
void Comm_SendAccel(int16_t *acc);

/**
 * @brief 发送编码器数据
 * @param encoder 编码器数组[A,B,C,D]
 */
void Comm_SendEncoder(int16_t *encoder);

/**
 * @brief 发送电池电压
 * @param voltage 电池电压（mV）
 */
void Comm_SendBatteryVoltage(uint16_t voltage);

/**
 * @brief 异或校验
 * @param a 数组地址
 * @param len 长度
 * @return 校验值
 */
uint8_t Comm_XORCheck(uint8_t *a, uint8_t len);

/**
 * @brief 串口发送数组函数
 * @param USART_COM 串口端口
 * @param buf 发送缓冲区
 * @param len 发送长度
 */
void Comm_SendBuf(USART_TypeDef *USART_COM, uint8_t *buf, uint16_t len);

/**
 * @brief 串口接收中断回调
 * @param huart UART句柄
 * @note 在HAL_UART_RxCpltCallback中调用
 */
void Comm_RxCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* __COMM_PROTOCOL_H */
