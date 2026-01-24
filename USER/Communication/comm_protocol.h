/**
 ******************************************************************************
 * @file    comm_protocol.h
 * @brief   通信协议模块 - 串口通信
 *
 * 功能说明：
 * - 接收控制指令并分发到各模块
 * - 提供数据发送接口
 *
 * 模块接口：
 * - Comm_Init(): 初始化模块
 * - Comm_Update(): 处理接收数据并分发指令
 * - Comm_Send*(): 提供发送接口供其他模块调用
 ******************************************************************************
 */

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

#define RX_BUFFER_SIZE        256   /* 接收缓冲区大小 */

/* ==================== 类型定义 ==================== */

/* 串口接收状态 */
typedef struct {
    uint8_t buffer[RX_BUFFER_SIZE];  /* 接收缓冲区 */
    uint8_t index;                   /* 当前接收索引 */
    uint8_t byte;                    /* 单字节接收缓冲 */
    uint8_t complete;                /* 接收完成标志 */
} CommRx_t;

/* 串口发送缓冲 - 增大到128字节以支持更长的数据帧 */
extern uint8_t UART_SEND_BUF[128];

/* ==================== 核心接口 ==================== */

/**
 * @brief 初始化通信模块
 */
void Comm_Init(void);

/**
 * @brief 更新通信模块
 *
 * 功能：
 * - 处理接收到的数据
 * - 解析协议帧
 * - 分发控制指令到各模块
 *
 * 注意：需要在主循环中每次调用
 */
void Comm_Update(void);

/* ==================== 工具函数 ==================== */

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
 * @brief 公共接口：发送协议数据帧
 * @param func_code 功能码
 * @param data 数据指针（int16_t数组）
 * @param data_len 数据长度（字节数，支持任意长度）
 *
 * 功能：
 * - 自动打包协议帧：[0xFC][FuncCode][Data...][Checksum][0xDF]
 * - 支持动态数据长度（不限制为10字节）
 * - 校验位计算包括帧头、功能码和数据段
 * - 各模块可调用此接口发送数据
 *
 * 示例：
 * - Comm_SendDataFrame(FUNC_EULER_ANGLE, euler, 6);   // 3个int16_t
 * - Comm_SendDataFrame(FUNC_GYRO, gyro, 6);           // 3个int16_t
 * - Comm_SendDataFrame(FUNC_ENCODER, encoders, 8);    // 4个int16_t
 * - Comm_SendDataFrame(FUNC_BATTERY_VOLTAGE, &voltage, 2);  // 1个int16_t
 */
void Comm_SendDataFrame(uint8_t func_code, int16_t *data, uint8_t data_len);

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
