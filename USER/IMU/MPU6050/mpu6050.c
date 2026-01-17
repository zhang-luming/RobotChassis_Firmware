#include "mpu6050.h"
#include "usart.h"
#include "stdio.h"

/*初始化MPU6050
 * 返回：0成功 其他：失败
 */
uint8_t MPU_Init(void) {
    MPU_IIC_Init(); // 初始化IIC总线
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X80); // 复位MPU6050
    HAL_Delay(100);
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X00); // 唤醒MPU6050
    MPU_Write_Byte(MPU_GYRO_CFG_REG, 0X18); // 陀螺仪传感器,±2000dps
    MPU_Write_Byte(MPU_ACCEL_CFG_REG, 0X00); // 加速度传感器,±2g
    MPU_Write_Byte(MPU_SAMPLE_RATE_REG, 19); // 设置采样率50Hz = 1000 / (1 + 19)
    MPU_Write_Byte(MPU_CFG_REG, 4); // 设置数字低通滤波器
    MPU_Write_Byte(MPU_INT_EN_REG, 0X00); // 关闭所有中断
    MPU_Write_Byte(MPU_USER_CTRL_REG, 0X00); // I2C主模式关闭
    MPU_Write_Byte(MPU_FIFO_EN_REG, 0X00); // 关闭FIFO
    MPU_Write_Byte(MPU_INTBP_CFG_REG, 0X80); // INT引脚低电平有效
    const uint8_t res = MPU_Read_Byte(MPU_DEVICE_ID_REG);
    if (res == MPU_ADDR) // 器件ID正确
    {
        printf("器件ID读取正确\r\n");
        MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X01); // 设置CLKSEL,PLL X轴为参考
        MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0X00); // 加速度与陀螺仪都工作
        MPU_Write_Byte(MPU_SAMPLE_RATE_REG, 19); // 设置采样率为50Hz
        MPU_Write_Byte(MPU_CFG_REG, 4); // 设置数字低通滤波器
        return 0;
    } else
        return 1;
}

/*读取温度
 * 返回值放大100倍
 */
short MPU_Get_Temperature(void) {
    uint8_t buf[2];
    MPU_Read_Len(MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buf);
    const short raw = ((uint16_t) buf[0] << 8) | buf[1];
    const float temp = 36.53 + ((double) raw) / 340;
    return temp * 100;
}

/*得到陀螺仪值(原始值)
 *gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
 * 返回值:0,成功 其他错误
 */
uint8_t MPU_Get_Gyroscope(short *gx, short *gy, short *gz) {
    uint8_t buf[6];
    const uint8_t res = MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
    if (res == 0) {
        *gx = ((uint16_t) buf[0] << 8) | buf[1];
        *gy = ((uint16_t) buf[2] << 8) | buf[3];
        *gz = ((uint16_t) buf[4] << 8) | buf[5];
    }
    return res;
}

/* 得到加速度值(原始值)
 * gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
 * 返回值:0,成功 其他错误
 */
uint8_t MPU_Get_Accelerometer(short *ax, short *ay, short *az) {
    uint8_t buf[6];
    const uint8_t res = MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);
    if (res == 0) {
        *ax = ((uint16_t) buf[0] << 8) | buf[1];
        *ay = ((uint16_t) buf[2] << 8) | buf[3];
        *az = ((uint16_t) buf[4] << 8) | buf[5];
    }
    return res;
}

/*IIC连续写
 * addr:器件地址 reg:寄存器地址 len:写入长度 buf:数据区
 * 返回:0正常 其他错误
 */
uint8_t MPU_Write_Len(const uint8_t addr, const uint8_t reg, const uint8_t len,
                      const uint8_t *buf) {
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr << 1) | 0); // 发送器件地址+写命令
    if (MPU_IIC_Wait_Ack()) // 等待应答
    {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Send_Byte(reg); // 写寄存器地址
    MPU_IIC_Wait_Ack(); // 等待应答
    for (uint8_t i = 0; i < len; i++) {
        MPU_IIC_Send_Byte(buf[i]); // 发送数据
        if (MPU_IIC_Wait_Ack()) // 等待ACK
        {
            MPU_IIC_Stop();
            return 1;
        }
    }
    MPU_IIC_Stop();
    return 0;
}

/*IIC连续读
 * addr:器件地址 reg:要读取的寄存器地址 len:要读取的长度 buf:读取到的数据存储区
 * 返回:0正常 其他错误
 */
uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf) {
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr << 1) | 0); // 发送器件地址+写命令
    if (MPU_IIC_Wait_Ack()) // 等待应答
    {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Send_Byte(reg); // 写寄存器地址
    MPU_IIC_Wait_Ack(); // 等待应答
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr << 1) | 1); // 发送器件地址+读命令
    MPU_IIC_Wait_Ack(); // 等待应答
    while (len) {
        if (len == 1)
            *buf = MPU_IIC_Read_Byte(0); // 读数据,发送nACK
        else
            *buf = MPU_IIC_Read_Byte(1); // 读数据,发送ACK
        len--;
        buf++;
    }
    MPU_IIC_Stop(); // 产生一个停止条件
    return 0;
}

/*IIC写一个字节
 * reg:寄存器地址 data:数据
 * 返回:0正常 其他错误
 */
uint8_t MPU_Write_Byte(uint8_t reg, uint8_t data) {
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((MPU_ADDR << 1) | 0); // 发送器件地址+写命令
    if (MPU_IIC_Wait_Ack()) // 等待应答
    {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Send_Byte(reg); // 写寄存器地址
    MPU_IIC_Wait_Ack(); // 等待应答
    MPU_IIC_Send_Byte(data); // 发送数据
    if (MPU_IIC_Wait_Ack()) // 等待ACK
    {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Stop();
    return 0;
}

/*IIC读一个字节
 * reg:寄存器地址
 * 返回读取的数据
 */
uint8_t MPU_Read_Byte(uint8_t reg) {
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((MPU_ADDR << 1) | 0); // 发送器件地址+写命令
    MPU_IIC_Wait_Ack(); // 等待应答
    MPU_IIC_Send_Byte(reg); // 写寄存器地址
    MPU_IIC_Wait_Ack(); // 等待应答
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((MPU_ADDR << 1) | 1); // 发送器件地址+读命令
    MPU_IIC_Wait_Ack(); // 等待应答
    uint8_t res = MPU_IIC_Read_Byte(0); // 读取数据,发送nACK
    MPU_IIC_Stop(); // 产生一个停止条件
    return res;
}
