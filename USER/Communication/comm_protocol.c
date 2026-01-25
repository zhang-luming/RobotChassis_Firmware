/**
 ******************************************************************************
 * @file    comm_protocol.c
 * @brief   通信协议模块 - 串口通信
 *
 * 功能说明：
 * - 接收控制指令并分发到各模块
 * - 提供数据发送接口
 ******************************************************************************
 */

#include "comm_protocol.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"
#include "user_config.h"
#include "tim.h"
#include "motor_control.h"
#include "servo_control.h"
#include "System/debug.h"

/* ==================== 私有变量 ==================== */

/* 接收缓冲区 - 循环缓冲区设计 */
#define RX_BUFFER_SIZE 256
static uint8_t g_rx_buffer[RX_BUFFER_SIZE];
static volatile uint16_t g_rx_write_idx = 0;  /* 写指针（中断中） */
static volatile uint16_t g_rx_read_idx = 0;   /* 读指针（主循环中） */

/* 单字节接收缓冲（用于HAL库） */
static uint8_t g_rx_byte = 0;

/* 串口发送缓冲 */
uint8_t UART_SEND_BUF[128];

/* 调试：上一次打印的字节数 */
static uint16_t g_last_printed_bytes = 0;

/* ==================== 公共接口实现 ==================== */

/**
 * @brief 初始化通信模块
 */
void Comm_Init(void) {
    memset((void*)g_rx_buffer, 0, RX_BUFFER_SIZE);
    g_rx_write_idx = 0;
    g_rx_read_idx = 0;
    g_last_printed_bytes = 0;

    /* 启动串口接收中断 */
    HAL_UART_Receive_IT(&huart2, &g_rx_byte, 1);
}

/**
 * @brief 更新通信模块（每10ms调用）
 *
 * 功能：
 * - 读取并打印接收缓冲区中的新数据
 */
void Comm_Update(void) {
    /* 检查是否有新数据 */
    if (g_rx_write_idx != g_rx_read_idx) {

        uint16_t new_bytes = (g_rx_write_idx >= g_rx_read_idx) ?
                             (g_rx_write_idx - g_rx_read_idx) :
                             (RX_BUFFER_SIZE - g_rx_read_idx + g_rx_write_idx);

        if (new_bytes > 0) {
            printf("[Comm_Update] 接收到 %u 字节: ", new_bytes);

            /* 打印新接收的字节 */
            for (uint16_t i = 0; i < new_bytes; i++) {
                uint16_t idx = (g_rx_read_idx + i) % RX_BUFFER_SIZE;
                printf("%02X ", g_rx_buffer[idx]);
            }
            printf("\r\n");

            /* 更新读指针 */
            g_rx_read_idx = (g_rx_read_idx + new_bytes) % RX_BUFFER_SIZE;
        }
    }
}

/**
 * @brief 串口接收中断回调
 * @note 在中断中调用，只负责接收数据到缓冲区
 */
void Comm_RxCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        /* 将接收到的字节写入循环缓冲区 */
        uint16_t next_idx = (g_rx_write_idx + 1) % RX_BUFFER_SIZE;

        /* 缓冲区未满时才写入 */
        if (next_idx != g_rx_read_idx) {
            g_rx_buffer[g_rx_write_idx] = g_rx_byte;
            g_rx_write_idx = next_idx;
        }
        /* 缓冲区满时丢弃新字节 */
    }
}

/**
 * @brief UART接收完成回调（HAL库弱函数）
 * @param huart UART句柄
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        /* 调用接收回调 */
        Comm_RxCallback(huart);
    }

    /* 重新启动接收中断 */
    HAL_UART_Receive_IT(&huart2, &g_rx_byte, 1);
}

/* ==================== 发送函数实现 ==================== */

/**
 * @brief 异或校验
 */
uint8_t Comm_XORCheck(uint8_t *a, uint8_t len) {
    uint8_t XOR = 0;
    uint8_t i = 0;
    while (i < len) {
        XOR ^= a[i];
        i++;
    }
    return XOR;
}

/**
 * @brief 串口发送数组函数
 */
void Comm_SendBuf(USART_TypeDef *USART_COM, uint8_t *buf, uint16_t len) {
    while (len--) {
        while ((USART_COM->SR & 0X40) == 0);  /* 等待发送缓冲区空 */
        USART_COM->DR = (uint8_t)(*buf++);
        while ((USART_COM->SR & 0X40) == 0);  /* 等待发送完成 */
    }
}

/**
 * @brief 公共接口：发送协议数据帧
 */
void Comm_SendDataFrame(uint8_t func_code, int16_t *data, uint8_t data_len) {
    /* 计算帧总长度 */
    uint8_t frame_len = 2 + data_len + 2;

    /* 构建协议帧 */
    UART_SEND_BUF[0] = PROTOCOL_HEADER;
    UART_SEND_BUF[1] = func_code;

    /* 复制数据（int16_t转uint8_t，大端序） */
    uint8_t data_idx = 2;
    for (uint8_t i = 0; i < data_len; i += 2) {
        int16_t value = data[i / 2];
        UART_SEND_BUF[data_idx++] = (value >> 8) & 0xFF;
        UART_SEND_BUF[data_idx++] = value & 0xFF;
    }

    /* 计算校验和 */
    UART_SEND_BUF[data_idx] = Comm_XORCheck(UART_SEND_BUF, data_idx);
    UART_SEND_BUF[data_idx + 1] = PROTOCOL_TAIL;

    /* 发送 */
    Comm_SendBuf(USART2, UART_SEND_BUF, frame_len);
}
