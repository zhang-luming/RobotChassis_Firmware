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

/* 串口接收状态 */
static CommRx_t g_comm_rx = {
    .buffer = {0},
    .index = 0,
    .byte = 0,
    .complete = 0
};

/* 串口发送缓冲 - 增大到128字节 */
uint8_t UART_SEND_BUF[128];

/* 调试统计 */
static uint32_t g_rx_byte_count = 0;     /* 接收字节计数 */
static uint32_t g_rx_frame_count = 0;    /* 接收完整帧计数 */
static uint32_t g_rx_checksum_error = 0; /* 校验错误计数 */

/* ==================== 公共接口实现 ==================== */

/**
 * @brief 初始化通信模块
 */
void Comm_Init(void) {
    memset(g_comm_rx.buffer, 0, sizeof(g_comm_rx.buffer));
    g_comm_rx.index = 0;
    g_comm_rx.complete = 0;

    /* 启动串口接收中断 */
    HAL_UART_Receive_IT(&huart2, &g_comm_rx.byte, 1);
}

/**
 * @brief 更新通信模块（每10ms调用）
 *
 * 功能：
 * - 处理接收到的数据
 * - 解析协议帧
 * - 分发控制指令到各模块
 */
void Comm_Update(void) {
    if (g_comm_rx.complete == 1) {
        uint8_t func_code = g_comm_rx.buffer[1];  /* 功能码 */

        /* 打印完整帧数据（十六进制）用于调试 */
        DEBUG_INFO("[Comm] 收到帧: FC=0x%02X, Len=%d", func_code, g_comm_rx.index);
        DEBUG_INFO("[Comm] 帧数据: ");
        for (uint8_t i = 0; i < g_comm_rx.index && i < 16; i++) {
            printf("%02X ", g_comm_rx.buffer[i]);
            if ((i + 1) % 8 == 0) printf("\r\n          ");
        }
        printf("\r\n");

        switch (func_code) {
            case FUNC_SERVO_CONTROL:  /* 舵机控制 0x08 */
            {
                DEBUG_INFO("[Comm] 舵机控制: s1=%d, s2=%d",
                          g_comm_rx.buffer[2], g_comm_rx.buffer[3]);
                /* 调用舵机模块的Process函数 */
                Servo_ProcessSetAngle(0, g_comm_rx.buffer[2]);  /* 舵机1 */
                Servo_ProcessSetAngle(1, g_comm_rx.buffer[3]);  /* 舵机2 */
                break;
            }

            case FUNC_MOTOR_SPEED:  /* 电机目标速度 0x06 */
            {
                int16_t speed[4];
                speed[0] = (int16_t)((g_comm_rx.buffer[2] << 8) | g_comm_rx.buffer[3]);
                speed[1] = (int16_t)((g_comm_rx.buffer[4] << 8) | g_comm_rx.buffer[5]);
                speed[2] = (int16_t)((g_comm_rx.buffer[6] << 8) | g_comm_rx.buffer[7]);
                speed[3] = (int16_t)((g_comm_rx.buffer[8] << 8) | g_comm_rx.buffer[9]);

                DEBUG_INFO("[Comm] 电机速度: A=%d, B=%d, C=%d, D=%d",
                          speed[0], speed[1], speed[2], speed[3]);
                /* 调用电机模块的Process函数 */
                Motor_ProcessSetSpeed(0, speed[0]);
                Motor_ProcessSetSpeed(1, speed[1]);
                Motor_ProcessSetSpeed(2, speed[2]);
                Motor_ProcessSetSpeed(3, speed[3]);
                break;
            }

            case FUNC_PID_PARAM:  /* PID参数 0x07 */
            {
                int16_t kp = (int16_t)((g_comm_rx.buffer[2] << 8) | g_comm_rx.buffer[3]);
                int16_t ki = (int16_t)((g_comm_rx.buffer[4] << 8) | g_comm_rx.buffer[5]);
                int16_t kd = (int16_t)((g_comm_rx.buffer[6] << 8) | g_comm_rx.buffer[7]);

                DEBUG_INFO("[Comm] PID参数: Kp=%d, Ki=%d, Kd=%d", kp, ki, kd);
                /* 调用电机模块的Process函数 */
                /* 为4个电机设置相同的PID参数 */
                for (uint8_t i = 0; i < 4; i++) {
                    Motor_ProcessSetPID(i, kp, ki, kd);
                }
                break;
            }

            default:
                DEBUG_WARN("[Comm] 未知功能码: 0x%02X", func_code);
                break;
        }

        /* 处理完成后重置接收状态 */
        g_comm_rx.complete = 0;
        g_comm_rx.index = 0;
        g_rx_frame_count++;
    }
}

/**
 * @brief 串口接收中断回调
 * @param huart UART句柄
 * @note 在中断中调用，避免使用printf等耗时操作
 */
void Comm_RxCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        g_rx_byte_count++;

        /* 最小化处理，不打印日志 */
        if (g_comm_rx.index >= 255) {  /* 溢出判断 */
            g_comm_rx.index = 0;
            memset(g_comm_rx.buffer, 0x00, sizeof(g_comm_rx.buffer));
        } else if (g_comm_rx.index == 0 && g_comm_rx.complete == 0) {  /* 接收首位 */
            if (g_comm_rx.byte == PROTOCOL_HEADER) {
                g_comm_rx.buffer[g_comm_rx.index] = g_comm_rx.byte;
                g_comm_rx.index++;
            } else {
                g_comm_rx.index = 0;
            }
        } else if (g_comm_rx.index == 1) {  /* 接收功能位 */
            // 接受所有有效的功能码 (0x01-0x08)
            if (g_comm_rx.byte >= 0x01 && g_comm_rx.byte <= 0x08) {
                g_comm_rx.buffer[g_comm_rx.index] = g_comm_rx.byte;
                g_comm_rx.index++;
            } else {
                g_comm_rx.index = 0;
            }
        } else if (g_comm_rx.index > 1) {  /* 接收数据、校验和帧尾 */
            g_comm_rx.buffer[g_comm_rx.index] = g_comm_rx.byte;
            g_comm_rx.index++;

            /* 检查是否接收到帧尾 */
            if (g_comm_rx.byte == PROTOCOL_TAIL) {
                /* 计算校验和（不包括帧尾） */
                uint8_t checksum_len = g_comm_rx.index - 1;  // 减去帧尾
                uint8_t calculated_checksum = Comm_XORCheck(g_comm_rx.buffer, checksum_len);
                uint8_t received_checksum = g_comm_rx.buffer[checksum_len];

                if (calculated_checksum == received_checksum) {
                    g_comm_rx.complete = 1;
                    /* 注意：不在这里重置index和增加frame_count，由Comm_Update()统一处理 */
                } else {
                    /* 校验错误，丢弃帧 */
                    g_rx_checksum_error++;
                    g_comm_rx.index = 0;
                }
            }
        }
    }

    /* 重新启动接收中断 - 放在最后，确保执行 */
    HAL_UART_Receive_IT(&huart2, &g_comm_rx.byte, 1);
}

/**
 * @brief UART接收完成回调（HAL库弱函数）
 * @param huart UART句柄
 *
 * 处理的串口：
 * - USART2: 控制指令接收
 *
 * 注意：
 * - USART1 仅用于调试输出，不处理接收数据
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    /* 仅处理 USART2 的接收中断 */
    if (huart->Instance == USART2) {
        /* ========== 调试：直接打印接收到的字节 ========== */
        printf("[UART2_IRQ] RX: 0x%02X ('%c')\r\n", g_comm_rx.byte,
               (g_comm_rx.byte >= 32 && g_comm_rx.byte <= 126) ? g_comm_rx.byte : '.');

        /* 暂时注释掉正常处理流程，只测试接收功能 */
        // Comm_RxCallback(huart);
    }
    /* USART1 不处理接收数据 */

    /* 重新启动接收中断 */
    HAL_UART_Receive_IT(&huart2, &g_comm_rx.byte, 1);
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
 *
 * 支持动态数据长度，校验位计算包括帧头、功能码和数据段
 */
void Comm_SendDataFrame(uint8_t func_code, int16_t *data, uint8_t data_len) {
    /* 计算帧总长度: 2字节(帧头+功能码) + data_len + 1字节(校验) + 1字节(帧尾) */
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

    /* 计算校验和（包括帧头、功能码和数据段） */
    UART_SEND_BUF[data_idx] = Comm_XORCheck(UART_SEND_BUF, data_idx);
    UART_SEND_BUF[data_idx + 1] = PROTOCOL_TAIL;

    /* 发送（动态帧长） */
    Comm_SendBuf(USART2, UART_SEND_BUF, frame_len);
}

