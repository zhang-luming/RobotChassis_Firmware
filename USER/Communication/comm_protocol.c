#include "comm_protocol.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"
#include "user_config.h"
#include "tim.h"

/* ==================== 私有变量 ==================== */

/* 串口接收状态 */
static CommRx_t comm_rx = {
    .buffer = {0},
    .index = 0,
    .byte = 0,
    .complete = 0
};

/* 串口发送缓冲 */
uint8_t UART_SEND_BUF[20];

/* ==================== 函数实现 ==================== */

/**
 * @brief 初始化通信模块
 */
void Comm_Init(void)
{
    memset(comm_rx.buffer, 0, sizeof(comm_rx.buffer));
    comm_rx.index = 0;
    comm_rx.complete = 0;

    /* 启动串口接收中断 */
    HAL_UART_Receive_IT(&huart2, &comm_rx.byte, 1);
}

/**
 * @brief 串口接收中断回调
 * @param huart UART句柄
 */
void Comm_RxCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        if(comm_rx.index >= 255)  /* 溢出判断 */
        {
            comm_rx.index = 0;
            memset(comm_rx.buffer, 0x00, sizeof(comm_rx.buffer));
            HAL_UART_Transmit(&huart1, (uint8_t *)"串口2数据溢出\r\n", 17, 0xFFFF);
        }
        else if(comm_rx.index == 0 && comm_rx.complete == 0)  /* 接收首位 */
        {
            if(comm_rx.byte == 0xFC)
            {
                comm_rx.buffer[comm_rx.index] = comm_rx.byte;
                comm_rx.index++;
            }
            else
            {
                comm_rx.index = 0;
                printf("串口2接收错误：起始位\r\n");
            }
        }
        else if(comm_rx.index == 1)  /* 接收功能位 */
        {
            if(comm_rx.byte == 0x08 || comm_rx.byte == 0x06 || comm_rx.byte == 0x07)
            {
                comm_rx.buffer[comm_rx.index] = comm_rx.byte;
                comm_rx.index++;
            }
            else
            {
                comm_rx.index = 0;
                printf("串口2接收错误：功能位\r\n");
            }
        }
        else if(comm_rx.index > 1 && comm_rx.index < 10)  /* 数据位 */
        {
            comm_rx.buffer[comm_rx.index] = comm_rx.byte;
            comm_rx.index++;
        }
        else if(comm_rx.index == 10)  /* 校验位 */
        {
            if(Comm_XORCheck(comm_rx.buffer, 10) == comm_rx.byte)
            {
                comm_rx.buffer[comm_rx.index] = comm_rx.byte;
                comm_rx.index++;
            }
            else
            {
                comm_rx.index = 0;
                printf("串口2接收错误：校验位\r\n");
            }
        }
        else if(comm_rx.index == 11)  /* 结束位 */
        {
            if(comm_rx.byte == 0xDF)
            {
                comm_rx.buffer[comm_rx.index] = comm_rx.byte;
                comm_rx.complete = 1;
                printf("串口2接收完成\r\n");
                comm_rx.index = 0;
            }
            else
            {
                comm_rx.index = 0;
            }
        }
    }

    /* 重新启动接收中断 */
    HAL_UART_Receive_IT(&huart2, &comm_rx.byte, 1);
}

/**
 * @brief 处理接收到的控制数据
 */
void Comm_ProcessControlData(void)
{
    if(comm_rx.complete == 1)
    {
        switch(comm_rx.buffer[1])  /* 功能位 */
        {
            case FUNC_SERVO_CONTROL:  /* 舵机控制 0x08 */
            {
                extern void Servo_SetAngle(uint8_t servo_id, uint8_t angle);

                printf("舵机旋转:%d,%d\r\n", comm_rx.buffer[2], comm_rx.buffer[3]);
                printf("脉冲输出:%d,%d\r\n",
                       2000*comm_rx.buffer[2]/180+500,
                       2000*comm_rx.buffer[3]/180+500);

                Servo_SetAngle(0, comm_rx.buffer[2]);  /* 舵机1 */
                Servo_SetAngle(1, comm_rx.buffer[3]);  /* 舵机2 */
                break;
            }
            case FUNC_MOTOR_SPEED:  /* 电机目标速度 0x06 */
            {
                extern void Motor_SetTargetSpeed(uint8_t motor_id, int16_t speed);

                Motor_SetTargetSpeed(0, (comm_rx.buffer[2]<<8) | comm_rx.buffer[3]);
                Motor_SetTargetSpeed(1, (comm_rx.buffer[4]<<8) | comm_rx.buffer[5]);
                Motor_SetTargetSpeed(2, (comm_rx.buffer[6]<<8) | comm_rx.buffer[7]);
                Motor_SetTargetSpeed(3, (comm_rx.buffer[8]<<8) | comm_rx.buffer[9]);

                printf("期望转速:%d,%d,%d,%d\r\n",
                       (comm_rx.buffer[2]<<8) | comm_rx.buffer[3],
                       (comm_rx.buffer[4]<<8) | comm_rx.buffer[5],
                       (comm_rx.buffer[6]<<8) | comm_rx.buffer[7],
                       (comm_rx.buffer[8]<<8) | comm_rx.buffer[9]);
                break;
            }
            case FUNC_PID_PARAM:  /* PID参数 0x07 */
            {
                PID_P = (int16_t)(comm_rx.buffer[2]<<8) | comm_rx.buffer[3];
                PID_I = (int16_t)(comm_rx.buffer[4]<<8) | comm_rx.buffer[5];
                PID_D = (int16_t)(comm_rx.buffer[6]<<8) | comm_rx.buffer[7];
                printf("设置PID:%d,%d,%d\r\n", PID_P, PID_I, PID_D);
                break;
            }
            default:
                break;
        }
        comm_rx.complete = 0;
    }
}

/**
 * @brief 异或校验
 */
uint8_t Comm_XORCheck(uint8_t *a, uint8_t len)
{
    uint8_t XOR = 0;
    uint8_t i = 0;
    while(i < len)
    {
        XOR ^= a[i];
        i++;
    }
    return XOR;
}

/**
 * @brief 串口发送数组函数
 */
void Comm_SendBuf(USART_TypeDef *USART_COM, uint8_t *buf, uint16_t len)
{
    while(len--)
    {
        while((USART_COM->SR & 0X40) == 0);  /* 等待发送缓冲区空 */
        USART_COM->DR = (uint8_t)(*buf++);
        while((USART_COM->SR & 0X40) == 0);  /* 等待发送完成 */
    }
}

/**
 * @brief 发送欧拉角数据
 */
void Comm_SendEulerAngle(int16_t *euler_angle)
{
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
void Comm_SendGyro(int16_t *gyro)
{
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
void Comm_SendAccel(int16_t *acc)
{
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
void Comm_SendEncoder(int16_t *encoder)
{
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
void Comm_SendBatteryVoltage(uint16_t voltage)
{
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
