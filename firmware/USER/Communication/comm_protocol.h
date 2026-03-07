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
 * - Comm_Receive(): 处理接收数据并分发指令
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
#define FUNC_IMU              0x03  /* IMU合并数据（欧拉角+陀螺仪+加速度） */
#define FUNC_MOTOR_SPEED      0x04  /* 电机目标速度 */
#define FUNC_PID_PARAM        0x05  /* PID参数设置 */
#define FUNC_SERVO_CONTROL    0x06  /* 舵机控制 */
#define FUNC_PTP_SYNC         0x10  /* PTP时间同步 */

#define RX_BUFFER_SIZE        256   /* 接收缓冲区大小 */

/* ==================== PTP时间同步协议 ==================== */

/*
 * PTP时间同步协议（基于PTP思想的简化实现）
 *
 * 帧格式（所有多字节值均为小端序 Little-Endian）：
 *
 * 请求帧（PC → MCU）：5字节
 *   [0xFC][0x10][0x01][checksum][0xDF]
 *    帧头  功能码  请求   校验和   帧尾
 *
 * 响应帧（MCU → PC）：20字节
 *   [0xFC][0x10][t2(8B)][tx_timestamp(t3, 8B)][checksum][0xDF]
 *    帧头  功能码  t2时间戳  t3时间戳(自动添加)      校验和  帧尾
 *
 * 说明：
 * - 0x01：PTP同步请求消息类型
 * - t2：MCU接收请求完成的时间戳（64位微秒，拆分为4个int16_t）
 * - t3：MCU发送响应开始的时间戳（64位微秒，Comm_SendDataFrame自动添加）
 * - 上位机计算：offset = ((t2+g_offset - t1) - (t4 - (t3+g_offset))) / 2
 */

#define PTP_SYNC_REQUEST      0x01  /* PTP同步请求消息类型 */

/* ==================== 旧协议定义（保留兼容性） ==================== */

/* ==================== 类型定义 ==================== */


/* 串口发送缓冲 - 增大到128字节以支持更长的数据帧 */
extern uint8_t UART_SEND_BUF[128];

/* ==================== DMA发送队列 ==================== */
/*
 * DMA发送队列（双缓冲区机制）
 * 队列配置：DMA_TX_QUEUE_SIZE=4帧，每帧最大64字节
 */

#define DMA_TX_QUEUE_SIZE 4  /* 队列深度：4帧 */
#define DMA_TX_FRAME_SIZE 64  /* 每帧最大长度 */

/* DMA发送帧结构 */
typedef struct {
    uint8_t data[DMA_TX_FRAME_SIZE];  /* 帧数据 */
    uint8_t len;                      /* 帧长度 */
} DmaTxFrame_t;

/* ==================== 核心接口 ==================== */

/**
 * @brief 初始化通信模块
 */
void Comm_Init(void);

/**
 * @brief 接收并处理通信数据
 *
 * 处理接收缓冲区中的数据、解析协议帧、分发控制指令到各模块
 * 需要在主循环中每次调用
 */
void Comm_Receive(void);

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
 * @brief 发送协议数据帧（阻塞方式）
 * @param func_code 功能码
 * @param data 数据指针（int16_t数组）
 * @param data_len 数据长度（int16_t个数）
 *
 * 自动打包协议帧：[0xFC][FuncCode][Data...][TxTimestamp][Checksum][0xDF]
 *
 * @示例
 * - Comm_SendDataFrame(FUNC_IMU, imu_data, 9);          // 9个int16_t
 * - Comm_SendDataFrame(FUNC_ENCODER, encoders, 8);      // 8个int16_t
 * - Comm_SendDataFrame(FUNC_BATTERY_VOLTAGE, &voltage, 1);  // 1个int16_t
 */
void Comm_SendDataFrame(uint8_t func_code, int16_t *data, uint8_t data_len);

/**
 * @brief DMA方式发送协议数据帧（非阻塞，使用发送队列）
 * @param func_code 功能码
 * @param data 数据指针（int16_t数组）
 * @param data_len 数据长度（int16_t个数）
 * @return HAL_OK=成功入队，HAL_ERROR=队列已满
 *
 * 使用DMA传输+发送队列，CPU立即返回。适用于周期性大数据量发送（IMU、编码器等）
 *
 * @示例
 *   // 在IMU中断中调用
 *   Comm_SendDataFrameDMA(FUNC_IMU, imu_data, 9);
 *   Comm_SendDataFrameDMA(FUNC_ENCODER, encoder_data, 8);
 */
HAL_StatusTypeDef Comm_SendDataFrameDMA(uint8_t func_code, int16_t *data, uint8_t data_len);

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
