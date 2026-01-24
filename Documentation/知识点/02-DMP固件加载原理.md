# DMP固件加载原理

## 概述

MPU6050内置的DMP (Digital Motion Processor) 是一个嵌入式数字运动处理器，需要加载固件(firmware)才能工作。

## 为什么需要加载固件

### MPU6050原始能力

MPU6050传感器原始输出：
- 陀螺仪：原始ADC值（需要转换为度/s）
- 加速度计：原始ADC值（需要转换为g）
- 温度传感器：原始ADC值

**问题**：
- 数据格式不直观（需要手动转换）
- 运动融合算法复杂（需要自己实现姿态解算）
- CPU负担重（需要持续计算）

### DMP的能力

加载DMP固件后，MPU6050可以：
- 自动进行传感器融合
- 直接输出四元数（quaternion）或欧拉角（pitch/roll/yaw）
- 内置各种运动检测算法
- 减轻主CPU负担

## DMP固件是什么

### 固件来源

DMP固件是由InvenSense公司提供的预编译二进制代码，存储在MPU6050的OTP（One-Time Programmable）内存或通过I2C加载到DMP内存。

### 固件内容

```c
// eMPL库中定义的DMP固件
const unsigned char dmp_memory[MPU_DMP_CODE_SIZE] = {
    0x00, 0x01, 0x02, 0x03, ...  // DMP二进制代码
};
```

## 固件加载流程

### 1. 基础MPU初始化

```c
// USER/IMU/MPU6050/mpu6050.c
MPU_Init() {
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0x80);  // 复位
    HAL_Delay(100);
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0x00);  // 唤醒
    MPU_Write_Byte(MPU_GYRO_CFG_REG, 0X18);  // 陀螺仪±2000dps
    MPU_Write_Byte(MPU_ACCEL_CFG_REG, 0X00); // 加速度±2g
}
```

### 2. DMP固件加载

```c
// USER/IMU/MPU6050/eMPL/inv_mpu.c
mpu_dmp_init() {
    mpu_init();                      // 基础初始化
    mpu_set_sensors();                 // 使能传感器
    mpu_configure_fifo();               // 配置FIFO
    dmp_load_motion_driver_firmware(); // ★ 加载DMP固件 ★
    dmp_set_orientation();              // 设置方向矩阵
    dmp_enable_feature();               // 使能DMP功能
    run_self_test();                   // 传感器自检
    mpu_set_dmp_state(1);             // 启动DMP
}
```

### 3. 关键函数说明

#### dmp_load_motion_driver_firmware()

```c
// inv_mpu_dmp_motion_driver.c
uint8_t dmp_load_motion_driver_firmware(void) {
    // 将DMP固件加载到MPU6050的DMP内存
    // 固件大小：MPU_DMP_CODE_SIZE（约3KB）

    // 步骤：
    // 1. 通过I2C逐块写入固件
    // 2. 写入DMP程序计数器
    // 3. 验证写入成功
}
```

#### 为什么需要后续配置

**固件加载后配置不生效的原因**：
- DMP固件加载后，还需要配置其运行参数
- 配置参数保存在DMP寄存器中（不是MPU6050寄存器）

```c
dmp_set_fifo_rate(100);         // 设置DMP输出速率
dmp_enable_feature(...);       // 使能所需功能（四元数、运动检测等）
```

## 为什么配置会覆盖之前的配置

### 配置执行顺序问题

```c
// 正确的顺序（保持一致）：
MPU_Write_Byte(MPU_SAMPLE_RATE_REG, 9);   // ← 步骤1: 设置100Hz (1000/(9+1)=100Hz)
mpu_dmp_init();                            // ← 步骤2: DMP初始化也设为100Hz
// 结果：最终是100Hz
```

### DMP初始化中的覆盖操作

```c
// inv_mpu.c: 2906
mpu_set_sample_rate(DEFAULT_MPU_HZ);  // 覆盖之前的设置
```

**DEFAULT_MPU_HZ** 默认为100，所以：
1. 先设置基础采样率为100Hz (SMPLRT_DIV = 9)
2. DMP初始化后，内部采样率保持100Hz
3. 最终DMP输出速率为100Hz

## DMP内存映射

### MPU6050内存结构

```
┌─────────────────────────────────────┐
│         用户寄存器 (0x00-0x7F)           │
│  - 配置寄存器                          │
│  - 状态寄存器                          │
├─────────────────────────────────────┤
│         DMP内存 (0x000-0x3FF)          │
│  - DMP固件代码                        │
│  - DMP运行时数据                      │
│  - DMP配置参数                        │
└─────────────────────────────────────┘
```

### 固件加载后的内存使用

```
DMP内存分配（示例）:
0x000-0x0FF: DMP程序代码
0x100-0x1FF: DMP数据存储
0x200-0x2FF: DMP堆栈
0x300-0x3FF: DMP变量
```

## DMP工作流程

### 1. 初始化阶段

```
MPU6050上电
    ↓
基础初始化（陀螺仪/加速度计）
    ↓
加载DMP固件到DMP内存
    ↓
配置DMP参数（方向矩阵、功能使能）
    ↓
传感器自检（验证硬件正常）
    ↓
启动DMP处理
```

### 2. 运行阶段

```
传感器采集（陀螺仪+加速度）
    ↓
DMP自动处理（内部硬件加速）
    ↓
运动融合算法（姿态解算）
    ↓
FIFO缓存（四元数或欧拉角）
    ↓
INT引脚触发（数据就绪）
    ↓
主MCU读取FIFO数据
```

## DMP特性

### 支持的输出格式

1. **四元数 (Quaternion)**
   - q0, q1, q2, q3
   - 姿态解算精度高
   - 无万向锁问题

2. **欧拉角 (Euler Angle)**
   - Pitch (俯仰角): -90° ~ +90°
   - Roll (横滚角): -180° ~ +180°
   - Yaw (航向角): -180° ~ +180°

3. **原始数据**
   - 陀螺仪 (3轴)
   - 加速度计 (3轴)

### 可配置功能

```c
DMP_FEATURE_6X_LP_QUAT        // 6轴低功耗四元数
DMP_FEATURE_TAP                 // 敲击检测
DMP_FEATURE_ANDROID_ORIENT      // 安卓方向检测
DMP_FEATURE_SEND_RAW_ACCEL     // 原始加速度数据
DMP_FEATURE_SEND_CAL_GYRO       // 校准后的陀螺仪数据
DMP_FEATURE_GYRO_CAL            // 陀螺仪校准
```

## 固件大小和内存影响

### DMP固件大小

- 代码大小：约3KB
- 占用DMP内存：约1KB（代码+数据）
- MPU6050内部总DMP内存：1KB (0x400字节)

### 内存占用对比

| 项目 | 不使用DMP | 使用DMP |
|------|-----------|---------|
| Flash占用 | ~10KB | ~13KB (+3KB) |
| RAM占用 | ~8KB | ~10KB (+2KB) |
| CPU占用 | 高（持续计算） | 低（仅读取数据） |

## 常见问题

### Q1: DMP初始化失败（错误码4）

**现象**：`dmp_load_motion_driver_firmware()` 返回错误

**可能原因**：
- I2C通信异常
- MPU6050硬件问题
- 时钟配置错误

**解决**：
- 检查I2C连接（PB12-SCL, PB13-SDA）
- 检查时钟配置（I2C时钟由APB1提供）

### Q2: DMP数据更新不稳定

**现象**：欧拉角跳变、延迟大

**可能原因**：
- 中断处理时间长，导致DMP FIFO溢出
- 采样率配置不合理
- 传感器校准不准确

**解决**：
- 提高 `dmp_set_fifo_rate()` 值（降低采样率）
- 优化中断处理时间
- 重新运行自检

### Q3: DMP输出延迟大

**现象**：数据延迟超过100ms

**原因**：
- DMP处理时间本身需要时间
- FIFO读取间隔太长

**解决**：
- 减小FIFO读取间隔
- 使用DMP中断驱动模式（本项目已实现）

## 配置要点

### 1. 采样率配置

```c
// inv_mpu.h:30
#define DEFAULT_MPU_HZ  (100)  // 100Hz输出速率

// 同步修改mpu6050.c中的采样率寄存器：
MPU_Write_Byte(MPU_SAMPLE_RATE_REG, 9);  // 100Hz (1000/(9+1)=100Hz)

// 如需修改为其他频率（例如50Hz）：
#define DEFAULT_MPU_HZ  (50)
MPU_Write_Byte(MPU_SAMPLE_RATE_REG, 19); // 50Hz (1000/(19+1)=50Hz)
```

### 2. FIFO速率配置

```c
// FIFO速率必须 ≥ 采样率
dmp_set_fifo_rate(100);  // 100Hz输出
```

### 3. 功能使能

```c
// 只使能需要的功能，减少DMP计算负担
dmp_enable_feature(
    DMP_FEATURE_6X_LP_QUAT |      // 四元数
    DMP_FEATURE_SEND_RAW_ACCEL |     // 原始加速度
    DMP_FEATURE_SEND_CAL_GYRO        // 校准陀螺仪
);
```

## 参考资料

- MPU6050 DMP数据手册 (Invenense)
- eMPL库源码 (`USER/IMU/MPU6050/eMPL/`)
- DMP固件加载函数 (`dmp_load_motion_driver_firmware`)

---

**相关文档**：
- `03_IMU/MPU6050_IMU架构详解.md` - IMU模块说明
- `03_IMU/MPU_INT配置总结.md` - 中断配置说明

---

**文档版本**：1.0
**最后更新**：2026-01-24
