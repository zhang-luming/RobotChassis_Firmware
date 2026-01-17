#include "mpuiic.h"
#include "EXIT.h"

// MPU IIC 延时函数
void IIC_Delay(void)
{
  delay_us(2);
}

// 初始化IIC
void MPU_IIC_Init(void)
{
  GPIO_InitTypeDef GPIO_Initure;

  __HAL_RCC_GPIOB_CLK_ENABLE(); // GPIOB时钟开启

  GPIO_Initure.Pin = GPIO_PIN_8 | GPIO_PIN_9; // PB8、PB9
  GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;    // 推挽输出
  GPIO_Initure.Pull = GPIO_PULLUP;            // 上拉
  GPIO_Initure.Speed = GPIO_SPEED_HIGH;       // 高速模式

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET); // PB8,PB9 输出高电平

  HAL_GPIO_Init(GPIOB, &GPIO_Initure);
}
// IIC起始信号
void MPU_IIC_Start(void)
{
  MPU_SDA_OUT(); // sda线输出模式
  MPU_IIC_SDA = 1;
  MPU_IIC_SCL = 1;
  IIC_Delay();
  MPU_IIC_SDA = 0;
  IIC_Delay();
  MPU_IIC_SCL = 0;
}
// IIC停止信号
void MPU_IIC_Stop(void)
{
  MPU_SDA_OUT();
  MPU_IIC_SCL = 0;
  MPU_IIC_SDA = 0;
  IIC_Delay();
  MPU_IIC_SCL = 1;
  MPU_IIC_SDA = 1;
  IIC_Delay();
}

/*等待应答 1失败 0成功*/
uint8_t MPU_IIC_Wait_Ack(void)
{
  uint8_t ucErrTime = 0;
  MPU_SDA_IN(); // SDA设置为输入模式
  MPU_IIC_SDA = 1;
  IIC_Delay();
  MPU_IIC_SCL = 1;
  IIC_Delay();
  while (MPU_READ_SDA)
  {
    ucErrTime++;
    if (ucErrTime > 250)
    {
      MPU_IIC_Stop();
      return 1;
    }
  }
  MPU_IIC_SCL = 0;
  return 0;
}

// 产生ACK应答
void MPU_IIC_Ack(void)
{
  MPU_IIC_SCL = 0;
  MPU_SDA_OUT();
  MPU_IIC_SDA = 0;
  IIC_Delay();
  MPU_IIC_SCL = 1;
  IIC_Delay();
  MPU_IIC_SCL = 0;
}
// 不产生ACK应答
void MPU_IIC_NAck(void)
{
  MPU_IIC_SCL = 0;
  MPU_SDA_OUT();
  MPU_IIC_SDA = 1;
  IIC_Delay();
  MPU_IIC_SCL = 1;
  IIC_Delay();
  MPU_IIC_SCL = 0;
}
// IIC发送一个字节
// 返回从机有无应答
// 1，有应答
// 0，无应答
void MPU_IIC_Send_Byte(uint8_t txd)
{
  uint8_t t;
  MPU_SDA_OUT();
  MPU_IIC_SCL = 0; // 拉低时钟开始数据传输
  for (t = 0; t < 8; t++)
  {
    MPU_IIC_SDA = (txd & 0x80) >> 7;
    txd <<= 1;
    MPU_IIC_SCL = 1;
    IIC_Delay();
    MPU_IIC_SCL = 0;
    IIC_Delay();
  }
}
// 读1个字节，ack=1时，发送ACK，ack=0，发送nACK
uint8_t MPU_IIC_Read_Byte(unsigned char ack)
{
  unsigned char i, receive = 0;
  MPU_SDA_IN(); // SDA设置为输入
  for (i = 0; i < 8; i++)
  {
    MPU_IIC_SCL = 0;
    IIC_Delay();
    MPU_IIC_SCL = 1;
    receive <<= 1;
    if (MPU_READ_SDA)
      receive++;
    IIC_Delay();
  }
  if (!ack)
    MPU_IIC_NAck(); // 发送nACK
  else
    MPU_IIC_Ack(); // 发送ACK
  return receive;
}
