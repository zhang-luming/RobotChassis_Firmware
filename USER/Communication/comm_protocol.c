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

/* ==================== 私有变量 ==================== */

/* 串口接收状态 */
static CommRx_t g_comm_rx = {
    .buffer = {0},
    .index = 0,
    .byte = 0,
    .complete = 0
};

/* 串口发送缓冲 */
uint8_t UART_SEND_BUF[20];

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

        switch (func_code) {
            case FUNC_SERVO_CONTROL:  /* 舵机控制 0x08 */
            {
                printf("舵机旋转:%d,%d\r\n", g_comm_rx.buffer[2], g_comm_rx.buffer[3]);
                printf("脉冲输出:%d,%d\r\n",
                       2000 * g_comm_rx.buffer[2] / 180 + 500,
                       2000 * g_comm_rx.buffer[3] / 180 + 500);

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

                printf("期望速度[centi-CPS]:%d,%d,%d,%d\r\n", speed[0], speed[1], speed[2], speed[3]);

                /* 调用电机模块的Process函数（内部会自动转换为编码器增量） */
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

                printf("设置PID:%d,%d,%d\r\n", kp, ki, kd);

                /* 调用电机模块的Process函数 */
                /* 为4个电机设置相同的PID参数 */
                for (uint8_t i = 0; i < 4; i++) {
                    Motor_ProcessSetPID(i, kp, ki, kd);
                }
                break;
            }

            default:
                break;
        }

        g_comm_rx.complete = 0;
    }
}

/**
 * @brief 串口接收中断回调
 * @param huart UART句柄
 */
void Comm_RxCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        if (g_comm_rx.index >= 255) {  /* 溢出判断 */
            g_comm_rx.index = 0;
            memset(g_comm_rx.buffer, 0x00, sizeof(g_comm_rx.buffer));
            HAL_UART_Transmit(&huart1, (uint8_t *)"串口2数据溢出\r\n", 17, 0xFFFF);
        } else if (g_comm_rx.index == 0 && g_comm_rx.complete == 0) {  /* 接收首位 */
            if (g_comm_rx.byte == 0xFC) {
                g_comm_rx.buffer[g_comm_rx.index] = g_comm_rx.byte;
                g_comm_rx.index++;
            } else {
                g_comm_rx.index = 0;
                printf("串口2接收错误：起始位\r\n");
            }
        } else if (g_comm_rx.index == 1) {  /* 接收功能位 */
            if (g_comm_rx.byte == 0x08 || g_comm_rx.byte == 0x06 || g_comm_rx.byte == 0x07) {
                g_comm_rx.buffer[g_comm_rx.index] = g_comm_rx.byte;
                g_comm_rx.index++;
            } else {
                g_comm_rx.index = 0;
                printf("串口2接收错误：功能位\r\n");
            }
        } else if (g_comm_rx.index > 1 && g_comm_rx.index < 10) {  /* 数据位 */
            g_comm_rx.buffer[g_comm_rx.index] = g_comm_rx.byte;
            g_comm_rx.index++;
        } else if (g_comm_rx.index == 10) {  /* 校验位 */
            if (Comm_XORCheck(g_comm_rx.buffer, 10) == g_comm_rx.byte) {
                g_comm_rx.buffer[g_comm_rx.index] = g_comm_rx.byte;
                g_comm_rx.index++;
            } else {
                g_comm_rx.index = 0;
                printf("串口2接收错误：校验位\r\n");
            }
        } else if (g_comm_rx.index == 11) {  /* 结束位 */
            if (g_comm_rx.byte == 0xDF) {
                g_comm_rx.buffer[g_comm_rx.index] = g_comm_rx.byte;
                g_comm_rx.complete = 1;
                printf("串口2接收完成\r\n");
                g_comm_rx.index = 0;
            } else {
                g_comm_rx.index = 0;
            }
        }
    }

    /* 重新启动接收中断 */
    HAL_UART_Receive_IT(&huart2, &g_comm_rx.byte, 1);
}

/* ==================== 兼容性接口（临时） ==================== */

/**
 * @brief 兼容性接口：处理接收到的控制数据
 * @deprecated 请使用 Comm_Update() 替代
 */
void Comm_ProcessControlData(void) {
    Comm_Update();
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
 * @brief 发送欧拉角数据
 */
void Comm_SendEulerAngle(int16_t *euler_angle) {
    UART_SEND_BUF[0] = PROTOCOL_HEADER;
    UART_SEND_BUF[1] = FUNC_EULER_ANGLE;
    UART_SEND_BUF[2] = euler_angle[0] >> 8;
    UART_SEND_BUF[3] = euler_angle[0] & 0x00FF;
    UART_SEND_BUF[4] = euler_angle[1] >> 8;
    UART_SEND_BUF[5] = euler_angle[1] & 0x00FF;
    UART_SEND_BUF[6] = euler_angle[2] >> 8;
    UART_SEND_BUF[7] = euler_angle[2] & 0x00FF;
    UART_SEND_BUF[8] = 0x00;
    UART_SEND_BUF[9] = 0x00;
    UART_SEND_BUF[10] = Comm_XORCheck(UART_SEND_BUF, 10);
    UART_SEND_BUF[11] = PROTOCOL_TAIL;
    Comm_SendBuf(USART2, UART_SEND_BUF, 12);
}

/**
 * @brief 发送陀螺仪数据
 */
void Comm_SendGyro(int16_t *gyro) {
    UART_SEND_BUF[0] = PROTOCOL_HEADER;
    UART_SEND_BUF[1] = FUNC_GYRO;
    UART_SEND_BUF[2] = gyro[0] >> 8;
    UART_SEND_BUF[3] = gyro[0] & 0x00FF;
    UART_SEND_BUF[4] = gyro[1] >> 8;
    UART_SEND_BUF[5] = gyro[1] & 0x00FF;
    UART_SEND_BUF[6] = gyro[2] >> 8;
    UART_SEND_BUF[7] = gyro[2] & 0x00FF;
    UART_SEND_BUF[8] = 0x00;
    UART_SEND_BUF[9] = 0x00;
    UART_SEND_BUF[10] = Comm_XORCheck(UART_SEND_BUF, 10);
    UART_SEND_BUF[11] = PROTOCOL_TAIL;
    Comm_SendBuf(USART2, UART_SEND_BUF, 12);
}

/**
 * @brief 发送加速度数据
 */
void Comm_SendAccel(int16_t *acc) {
    UART_SEND_BUF[0] = PROTOCOL_HEADER;
    UART_SEND_BUF[1] = FUNC_ACCEL;
    UART_SEND_BUF[2] = acc[0] >> 8;
    UART_SEND_BUF[3] = acc[0] & 0x00FF;
    UART_SEND_BUF[4] = acc[1] >> 8;
    UART_SEND_BUF[5] = acc[1] & 0x00FF;
    UART_SEND_BUF[6] = acc[2] >> 8;
    UART_SEND_BUF[7] = acc[2] & 0x00FF;
    UART_SEND_BUF[8] = 0x00;
    UART_SEND_BUF[9] = 0x00;
    UART_SEND_BUF[10] = Comm_XORCheck(UART_SEND_BUF, 10);
    UART_SEND_BUF[11] = PROTOCOL_TAIL;
    Comm_SendBuf(USART2, UART_SEND_BUF, 12);
}

/**
 * @brief 发送编码器数据
 */
void Comm_SendEncoder(int16_t *encoder) {
    UART_SEND_BUF[0] = PROTOCOL_HEADER;
    UART_SEND_BUF[1] = FUNC_ENCODER;
    UART_SEND_BUF[2] = encoder[0] >> 8;
    UART_SEND_BUF[3] = encoder[0] & 0x00FF;
    UART_SEND_BUF[4] = encoder[1] >> 8;
    UART_SEND_BUF[5] = encoder[1] & 0x00FF;
    UART_SEND_BUF[6] = encoder[2] >> 8;
    UART_SEND_BUF[7] = encoder[2] & 0x00FF;
    UART_SEND_BUF[8] = encoder[3] >> 8;
    UART_SEND_BUF[9] = encoder[3] & 0x00FF;
    UART_SEND_BUF[10] = Comm_XORCheck(UART_SEND_BUF, 10);
    UART_SEND_BUF[11] = PROTOCOL_TAIL;
    Comm_SendBuf(USART2, UART_SEND_BUF, 12);
}

/**
 * @brief 发送电池电压
 */
void Comm_SendBatteryVoltage(uint16_t voltage) {
    UART_SEND_BUF[0] = PROTOCOL_HEADER;
    UART_SEND_BUF[1] = FUNC_BATTERY_VOLTAGE;
    UART_SEND_BUF[2] = voltage >> 8;
    UART_SEND_BUF[3] = voltage & 0x00FF;
    UART_SEND_BUF[4] = 0;
    UART_SEND_BUF[5] = 0;
    UART_SEND_BUF[6] = 0;
    UART_SEND_BUF[7] = 0;
    UART_SEND_BUF[8] = 0;
    UART_SEND_BUF[9] = 0;
    UART_SEND_BUF[10] = Comm_XORCheck(UART_SEND_BUF, 10);
    UART_SEND_BUF[11] = PROTOCOL_TAIL;
    Comm_SendBuf(USART2, UART_SEND_BUF, 12);
}
