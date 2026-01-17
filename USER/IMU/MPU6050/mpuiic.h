#ifndef __MPUIIC_H
#define __MPUIIC_H
#include "main.h"
#include "stm32f1xx.h"

// IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000) + 0x2000000 + ((addr & 0xFFFFF) << 5) + (bitnum << 2))
#define MEM_ADDR(addr) *((volatile unsigned long *)(addr))
#define BIT_ADDR(addr, bitnum) MEM_ADDR(BITBAND(addr, bitnum))

// IO方向设置
#define MPU_SDA_IN()          \
  {                           \
    GPIOB->CRH &= 0XFFFFFF0F; \
    GPIOB->CRH |= 8 << 4;     \
  }
#define MPU_SDA_OUT()         \
  {                           \
    GPIOB->CRH &= 0XFFFFFF0F; \
    GPIOB->CRH |= 3 << 4;     \
  }

// IO操作函数
#define MPU_IIC_SCL BIT_ADDR((GPIOB_BASE + 12), 8) // SCL
#define MPU_IIC_SDA BIT_ADDR((GPIOB_BASE + 12), 9) // SDA
#define MPU_READ_SDA BIT_ADDR((GPIOB_BASE + 8), 9) // 输入SDA

// IIC所有操作函数
void IIC_Delay(void);                         // MPU IIC延时函数
void MPU_IIC_Init(void);                      // 初始化IIC的IO口
void MPU_IIC_Start(void);                     // 发送IIC开始信号
void MPU_IIC_Stop(void);                      // 发送IIC停止信号
void MPU_IIC_Send_Byte(uint8_t txd);          // IIC发送一个字节
uint8_t MPU_IIC_Read_Byte(unsigned char ack); // IIC读取一个字节
uint8_t MPU_IIC_Wait_Ack(void);               // IIC等待ACK信号
void MPU_IIC_Ack(void);                       // IIC发送ACK信号
void MPU_IIC_NAck(void);                      // IIC不发送ACK信号

void IMPU_IC_Write_One_Byte(uint8_t daddr, uint8_t addr, uint8_t data);
uint8_t MPU_IIC_Read_One_Byte(uint8_t daddr, uint8_t addr);
#endif
