# STM32 Core外设配置详解

## 文档概述

本文档详细说明了RobotChassis_Firmware项目中STM32F103系列微控制器的Core目录配置，包括所有外设的初始化参数、引脚映射和功能说明。

**生成工具**：STM32CubeMX
**MCU型号**：STM32F103xE（高容量设备）
**文档版本**：v1.0
**更新日期**：2026-01-17

---

## 目录

1. [Core目录结构](#1-core目录结构)
2. [系统时钟配置](#2-系统时钟配置)
3. [ADC配置](#3-adc配置)
4. [I2C配置](#4-i2c配置)
5. [USART配置](#5-usart配置)
6. [TIM定时器配置](#6-tim定时器配置)
7. [GPIO配置](#7-gpio配置)
8. [中断配置](#8-中断配置)
9. [资源使用统计](#9-资源使用统计)

---

## 1. Core目录结构

```
Core/
├── Inc/                          # 头文件目录
│   ├── adc.h                     # ADC配置头文件
│   ├── gpio.h                    # GPIO配置头文件
│   ├── i2c.h                     # I2C配置头文件
│   ├── main.h                    # 主程序头文件
│   ├── stm32f1xx_hal_conf.h      # HAL库配置文件
│   ├── stm32f1xx_it.h            # 中断处理头文件
│   ├── tim.h                     # 定时器配置头文件
│   └── usart.h                   # USART配置头文件
│
└── Src/                          # 源文件目录
    ├── adc.c                     # ADC配置实现
    ├── gpio.c                    # GPIO配置实现
    ├── i2c.c                     # I2C配置实现
    ├── main.c                    # 主程序
    ├── stm32f1xx_hal_msp.c       # MSP初始化函数
    ├── stm32f1xx_it.c            # 中断服务函数
    ├── syscalls.c                # 系统调用（newlib）
    ├── sysmem.c                  # 内存管理
    ├── system_stm32f1xx.c        # 系统初始化
    ├── tim.c                     # 定时器配置实现
    └── usart.c                   # USART配置实现
```

### 文件职责说明

| 文件 | 职责 | 是否由CubeMX生成 |
|------|------|----------------|
| **main.c** | 主程序入口、用户代码 | 部分（USER CODE段可编辑） |
| **stm32f1xx_hal_msp.c** | MSP初始化函数（底层硬件初始化） | 完全 |
| **stm32f1xx_it.c** | 中断服务函数 | 部分（用户可添加中断处理） |
| **system_stm32f1xx.c** | 系统启动、时钟配置 | 完全 |
| **syscalls.c** | newlib C库系统调用实现 | 完全 |
| **sysmem.c** | 堆内存管理 | 完全 |
| **xxx.c/h** | 各外设配置代码 | 完全 |

---

## 2. 系统时钟配置

### 2.1 时钟源配置

**主时钟源**：外部高速时钟（HSE）
- **频率**：假设使用8MHz晶振（需查看具体硬件）
- **PLL倍频**：9倍
- **系统时钟频率**：72MHz（STM32F103最大频率）

**时钟树结构**：
```
HSE (8MHz)
    ↓
PLL (×9)
    ↓
SYSCLK = 72MHz
    ↓
    ├─→ AHB (HCLK)  = 72MHz  (CPU、存储器、DMA)
    ├─→ APB1 (PCLK1) = 36MHz  (TIM2-7, USART2-3, I2C1-2)
    └─→ APB2 (PCLK2) = 72MHz  (TIM1,8, ADC1-2, USART1, SPI1)
```

### 2.2 外设时钟分频

| 总线 | 预分频 | 频率 | 说明 |
|-----|--------|------|-----|
| AHB | 1 | 72MHz | 直接输出，不分频 |
| APB1 | 2 | 36MHz | 高速外设时钟，2分频 |
| APB2 | 1 | 72MHz | 低速外设时钟，不分频 |
| ADC | 6 | 12MHz | ADC最大频率14MHz，6分频 |

### 2.3 定时器时钟倍频

**重要特性**：APB1上的定时器时钟自动×2（如果APB1分频>1）

```
APB1 = 36MHz
    ↓
TIM2-7 时钟 = 36MHz × 2 = 72MHz

APB2 = 72MHz
    ↓
TIM1,8 时钟 = 72MHz × 1 = 72MHz
```

**原因**：STM32设计保证定时器在APB分频时仍能获得最大性能。

---

## 3. ADC配置

### 3.1 ADC2配置

**实例**：`ADC_HandleTypeDef hadc2`

#### 3.1.1 基本参数

| 参数 | 值 | 说明 |
|-----|---|------|
| Instance | ADC2 | 使用ADC2外设 |
| ScanConvMode | DISABLE | 单通道转换，不扫描 |
| ContinuousConvMode | ENABLE | 连续转换模式 |
| DiscontinuousConvMode | DISABLE | 不使用不连续模式 |
| ExternalTrigConv | ADC_SOFTWARE_START | 软件触发转换 |
| DataAlign | ADC_DATAALIGN_RIGHT | 右对齐 |
| NbrOfConversion | 1 | 转换通道数：1 |

#### 3.1.2 通道配置

| 参数 | 值 | 说明 |
|-----|---|------|
| Channel | ADC_CHANNEL_8 | 通道8 |
| Rank | ADC_REGULAR_RANK_1 | 规则转换通道1 |
| SamplingTime | ADC_SAMPLETIME_1CYCLE_5 | 采样时间：1.5个周期 |

**采样时间计算**：
- ADC时钟：12MHz
- 采样周期：1.5 × (1/12MHz) = 0.125μs
- **总转换时间**：采样时间 + 12.5周期 = 14周期 ≈ 1.17μs

#### 3.1.3 引脚映射

```
PB0 → ADC2_IN8
```

**功能**：电池电压检测
- 模拟信号输入
- 无需外部上下拉电阻
- 连续转换，软件读取结果

#### 3.1.4 应用场景

**电池电压监测**：
```c
// 启动ADC
HAL_ADC_Start(&hadc2);

// 读取转换结果
HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
uint32_t adc_value = HAL_ADC_GetValue(&hadc2);

// 转换为电压（假设12位ADC，3.3V参考）
float voltage = (adc_value / 4095.0f) * 3.3f;

// 如果有分压电路，需要计算实际电压
float battery_voltage = voltage *分压系数;
```

---

## 4. I2C配置

### 4.1 I2C1配置

**实例**：`I2C_HandleTypeDef hi2c1`

#### 4.1.1 基本参数

| 参数 | 值 | 说明 |
|-----|---|------|
| Instance | I2C1 | 使用I2C1外设 |
| ClockSpeed | 100000 | 时钟速度：100kHz（标准模式） |
| DutyCycle | I2C_DUTYCYCLE_2 | 占空比：2:1（低/高） |
| OwnAddress1 | 0 | 本机地址（从机模式） |
| AddressingMode | I2C_ADDRESSINGMODE_7BIT | 7位地址模式 |
| DualAddressMode | I2C_DUALADDRESS_DISABLE | 禁用双地址 |
| OwnAddress2 | 0 | 第二本机地址 |
| GeneralCallMode | I2C_GENERALCALL_DISABLE | 禁用广播呼叫 |
| NoStretchMode | I2C_NOSTRETCH_DISABLE | 启用时钟延长 |

#### 4.1.2 引脚映射

| 引脚 | 功能 | 复用功能 | 说明 |
|-----|------|---------|------|
| PB8 | I2C1_SCL | AF_OD | I2C时钟（开漏输出） |
| PB9 | I2C1_SDA | AF_OD | I2C数据（开漏输出） |

**复用功能重映射**：
```c
__HAL_AFIO_REMAP_I2C1_ENABLE();
```
- 启用I2C1重映射
- 默认I2C1在PB6/PB7，重映射到PB8/PB9

#### 4.1.3 时钟配置

**I2C时钟计算**：
```
APB1 = 36MHz
I2C时钟 = 36MHz / (2 × DutyCycle) = 36MHz / 2 = 18MHz

SCL频率 = I2C时钟 / (2 × ClockSpeed)
        = 18MHz / (2 × 100000)
        = 90（实际上寄存器配置值为CCR）
```

**模式**：标准模式（Standard Mode, 100kHz）
- 适用于大多数应用
- 可靠性高，抗干扰能力强

#### 4.1.4 应用场景

**MPU6050 IMU通信**：
```c
// 读取MPU6050寄存器
uint8_t data;
HAL_I2C_Mem_Read(&hi2c1, 0x68<<1, 0x75, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

// 写入MPU6050寄存器
uint8_t config = 0x00;
HAL_I2C_Mem_Write(&hi2c1, 0x68<<1, 0x6B, I2C_MEMADD_SIZE_8BIT, &config, 1, 100);
```

**GPIO模拟I2C vs 硬件I2C**：
- 本项目I2C1用于硬件I2C（如果需要）
- MPU6050实际使用GPIO模拟I2C（mpuiic.c）
- 硬件I2C可用于其他I2C设备

---

## 5. USART配置

### 5.1 USART1配置

**实例**：`UART_HandleTypeDef huart1`

#### 5.1.1 基本参数

| 参数 | 值 | 说明 |
|-----|---|------|
| Instance | USART1 | 使用USART1外设 |
| BaudRate | 115200 | 波特率：115200bps |
| WordLength | UART_WORDLENGTH_8B | 8位数据位 |
| StopBits | UART_STOPBITS_1 | 1位停止位 |
| Parity | UART_PARITY_NONE | 无奇偶校验 |
| Mode | UART_MODE_TX_RX | 收发模式 |
| HwFlowCtl | UART_HWCONTROL_NONE | 无硬件流控 |
| OverSampling | UART_OVERSAMPLING_16 | 16倍过采样 |

#### 5.1.2 引脚映射

| 引脚 | 功能 | 复用功能 | 说明 |
|-----|------|---------|------|
| PA9 | USART1_TX | AF_PP | 发送引脚（推挽输出） |
| PA10 | USART1_RX | INPUT | 接收引脚（输入） |

**GPIO配置**：
```c
// TX引脚：推挽复用输出，高速
GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

// RX引脚：输入，无上下拉
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_NOPULL;
```

#### 5.1.3 中断配置

| 中断 | 优先级 | 抢占优先级 | 子优先级 | 状态 |
|-----|-------|----------|---------|-----|
| USART1_IRQn | 0 | 0 | 0 | 启用 |

**最高优先级**：抢占优先级0，表示最高优先级中断

#### 5.1.4 波特率计算

```
USART1时钟 = APB2 = 72MHz

波特率 = USART时钟 / (16 × USARTDIV)
115200 = 72000000 / (16 × USARTDIV)
USARTDIV = 72000000 / (16 × 115200) = 39.0625

USART_BRR寄存器 = 39.0625 × 16 = 625（0x0271）
```

#### 5.1.5 应用场景

**调试输出（printf重定向）**：
```c
// 初始化
RetargetInit(&huart1);
printf("System initialized!\n");
```

**通信协议**：
- 与上位机通信
- 发送传感器数据
- 接收控制指令

---

### 5.2 USART2配置

**实例**：`UART_HandleTypeDef huart2`

#### 5.2.1 基本参数

与USART1配置完全相同：
- 波特率：115200bps
- 8位数据位，1位停止位
- 无校验，无流控

#### 5.2.2 引脚映射

| 引脚 | 功能 | 复用功能 | 说明 |
|-----|------|---------|------|
| PA2 | USART2_TX | AF_PP | 发送引脚 |
| PA3 | USART2_RX | INPUT | 接收引脚 |

#### 5.2.3 中断配置

| 中断 | 优先级 | 抢占优先级 | 子优先级 | 状态 |
|-----|-------|----------|---------|-----|
| USART2_IRQn | 0 | 0 | 0 | 启用 |

#### 5.2.4 应用场景

**通信协议实现**：
- 接收上位机控制指令
- 发送底盘状态数据
- 自定义协议解析（见comm_protocol.c）

---

## 6. TIM定时器配置

本项目使用了7个定时器，每个都有特定功能。

### 6.1 TIM1 - 舵机PWM控制

**实例**：`TIM_HandleTypeDef htim1`
**模式**：PWM输出

#### 6.1.1 基本参数

| 参数 | 值 | 说明 |
|-----|---|------|
| Instance | TIM1 | 高级定时器1 |
| Prescaler | 143 | 预分频器：144分频 |
| CounterMode | TIM_COUNTERMODE_UP | 向上计数 |
| Period | 9999 | 自动重装载值：10000 |
| ClockDivision | TIM_CLOCKDIVISION_DIV1 | 无时钟分频 |
| RepetitionCounter | 0 | 重复计数器：0 |
| AutoReloadPreload | ENABLE | 自动重装载预装载启用 |

#### 6.1.2 PWM频率计算

```
TIM1时钟 = APB2 = 72MHz

定时器频率 = 72MHz / (Prescaler + 1) / (Period + 1)
           = 72MHz / 144 / 10000
           = 50Hz

PWM周期 = 1 / 50Hz = 20ms（标准舵机控制周期）
```

#### 6.1.3 PWM分辨率

```
PWM分辨率 = Period + 1 = 10000级

最小脉宽变化 = 20ms / 10000 = 2μs
```

#### 6.1.4 通道配置

| 通道 | 引脚 | 模式 | 极性 | 功能 |
|-----|------|------|------|------|
| CH2N | PB14 | PWM1 | HIGH | 舵机1控制 |
| CH3N | PB15 | PWM1 | HIGH | 舵机2控制 |

**注意**：使用互补通道（CHxN），非标准通道

#### 6.1.5 舵机控制角度

**标准舵机脉宽**：
- 0.5ms → -90°（或0°）
- 1.5ms → 中位
- 2.5ms → +90°（或180°）

**CCR寄存器值计算**：
```c
// 0.5ms脉宽
CCR_0_5ms = (0.5 / 20) * 10000 = 250

// 1.5ms脉宽（中位）
CCR_1_5ms = (1.5 / 20) * 10000 = 750

// 2.5ms脉宽
CCR_2_5ms = (2.5 / 20) * 10000 = 1250

// 设置舵机角度
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 750);  // 中位
```

---

### 6.2 TIM2 - 编码器A

**实例**：`TIM_HandleTypeDef htim2`
**模式**：编码器模式

#### 6.2.1 基本参数

| 参数 | 值 | 说明 |
|-----|---|------|
| Instance | TIM2 | 通用定时器2 |
| Prescaler | 0 | 无预分频 |
| CounterMode | TIM_COUNTERMODE_UP | 向上计数 |
| Period | 65535 | 最大计数值（16位） |
| EncoderMode | TIM_ENCODERMODE_TI12 | 编码器模式3（TI1+TI2） |

#### 6.2.2 编码器参数

| 参数 | 值 | 说明 |
|-----|---|------|
| IC1Polarity | TIM_ICPOLARITY_RISING | TI1上升沿触发 |
| IC1Selection | TIM_ICSELECTION_DIRECTTI | 直接连接 |
| IC1Prescaler | TIM_ICPSC_DIV1 | 无预分频 |
| IC1Filter | 0 | 无滤波 |
| IC2Polarity | TIM_ICPOLARITY_RISING | TI2上升沿触发 |
| IC2Selection | TIM_ICSELECTION_DIRECTTI | 直接连接 |
| IC2Prescaler | TIM_ICPSC_DIV1 | 无预分频 |
| IC2Filter | 0 | 无滤波 |

#### 6.2.3 引脚映射

| 引脚 | 功能 | 说明 |
|-----|------|------|
| PA15 | TIM2_CH1 | 编码器A相 |
| PB3 | TIM2_CH2 | 编码器B相 |

**重映射**：
```c
__HAL_AFIO_REMAP_TIM2_PARTIAL_1();
```
- 启用TIM2部分重映射1
- CH1从PA0重映射到PA15
- CH2从PA1重映射到PB3

#### 6.2.4 编码器工作原理

**TI12模式（4倍频）**：
- A相上升沿 → 计数+1
- A相下降沿 → 计数+1
- B相上升沿 → 计数+1
- B相下降沿 → 计数+1

**分辨率**：
- 编码器线数：PPR（每转脉冲数）
- 4倍频后分辨率 = 4 × PPR
- 例如：PPR=500，则分辨率=2000 counts/rev

---

### 6.3 TIM3 - 编码器B

**实例**：`TIM_HandleTypeDef htim3`
**模式**：编码器模式

#### 6.3.1 引脚映射

| 引脚 | 功能 | 说明 |
|-----|------|------|
| PB4 | TIM3_CH1 | 编码器A相 |
| PB5 | TIM3_CH2 | 编码器B相 |

**重映射**：
```c
__HAL_AFIO_REMAP_TIM3_PARTIAL();
```

#### 6.3.2 配置

与TIM2相同，使用TI12编码器模式。

---

### 6.4 TIM4 - 编码器C

**实例**：`TIM_HandleTypeDef htim4`
**模式**：编码器模式

#### 6.4.1 引脚映射

| 引脚 | 功能 | 说明 |
|-----|------|------|
| PB6 | TIM4_CH1 | 编码器A相 |
| PB7 | TIM4_CH2 | 编码器B相 |

**无重映射**：使用默认引脚

---

### 6.5 TIM5 - 编码器D

**实例**：`TIM_HandleTypeDef htim5`
**模式**：编码器模式

#### 6.5.1 引脚映射

| 引脚 | 功能 | 说明 |
|-----|------|------|
| PA0 | TIM5_CH1 | 编码器A相 |
| PA1 | TIM5_CH2 | 编码器B相 |

**特殊**：PA0也是WKUP引脚（唤醒功能）

---

### 6.6 TIM6 - 系统基准定时器

**实例**：`TIM_HandleTypeDef htim6`
**模式**：基本定时器（时基）

#### 6.6.1 基本参数

| 参数 | 值 | 说明 |
|-----|---|------|
| Instance | TIM6 | 基本定时器6 |
| Prescaler | 71 | 预分频器：72分频 |
| CounterMode | TIM_COUNTERMODE_UP | 向上计数 |
| Period | 9999 | 自动重装载值：10000 |
| AutoReloadPreload | ENABLE | 预装载启用 |

#### 6.6.2 定时频率计算

```
TIM6时钟 = APB1 = 36MHz（重映射后72MHz？）

定时频率 = 72MHz / (71 + 1) / (9999 + 1)
         = 72MHz / 72 / 10000
         = 100Hz

定时周期 = 1 / 100Hz = 10ms
```

#### 6.6.3 中断配置

| 中断 | 优先级 | 抢占优先级 | 子优先级 | 状态 |
|-----|-------|----------|---------|-----|
| TIM6_IRQn | 0 | 0 | 0 | 启用 |

#### 6.6.4 应用场景

**系统时基**：
```c
// 在TIM6中断中
void TIM6_IRQHandler(void) {
    if (LL_TIM_IsActiveFlag_UPDATE(TIM6)) {
        LL_TIM_ClearFlag_UPDATE(TIM6);

        // 每10ms执行一次
        Run_Times++;  // 循环计数器
    }
}
```

**主循环时序控制**：
```c
void main(void) {
    while(1) {
        if (Run_Times % 4 == 0) {
            // 每40ms执行一次
        }

        if (Run_Times % 10 == 0) {
            // 每100ms执行一次
        }
    }
}
```

---

### 6.7 TIM8 - 电机PWM控制

**实例**：`TIM_HandleTypeDef htim8`
**模式**：PWM输出

#### 6.7.1 基本参数

| 参数 | 值 | 说明 |
|-----|---|------|
| Instance | TIM8 | 高级定时器8 |
| Prescaler | 1 | 预分频器：2分频 |
| CounterMode | TIM_COUNTERMODE_UP | 向上计数 |
| Period | 1439 | 自动重装载值：1440 |
| ClockDivision | TIM_CLOCKDIVISION_DIV1 | 无时钟分频 |
| RepetitionCounter | 0 | 重复计数器：0 |
| AutoReloadPreload | ENABLE | 自动重装载预装载启用 |

#### 6.7.2 PWM频率计算（重要）

```
TIM8时钟 = APB2 = 72MHz

PWM频率 = 72MHz / (Prescaler + 1) / (Period + 1)
        = 72MHz / 2 / 1440
        = 25kHz
```

**为什么是25kHz？**
1. 超出人耳听觉范围（20-20kHz），消除电机啸叫
2. 平衡电机驱动效率和MOSFET开关损耗
3. 常用电机驱动频率范围：10-50kHz

#### 6.7.3 PWM分辨率

```
PWM分辨率 = Period + 1 = 1440级

最小PWM占空比变化 = 1 / 1440 = 0.069%

PWM精度 = 12bit (2^10 = 1024 < 1440 < 2^11 = 2048)
```

#### 6.7.4 通道配置

| 通道 | 引脚 | 模式 | 极性 | 功能 |
|-----|------|------|------|------|
| CH1 | PC6 | PWM1 | HIGH | 电机A PWM |
| CH2 | PC7 | PWM1 | HIGH | 电机B PWM |
| CH3 | PC8 | PWM1 | HIGH | 电机C PWM |
| CH4 | PC9 | PWM1 | HIGH | 电机D PWM |

#### 6.7.5 电机控制示例

```c
// 启动PWM
HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

// 设置PWM占空比
// speed范围：-1440 到 +1440
// 0 = 停止，正值正转，负值反转
void Motor_SetSpeed(TIM_HandleTypeDef *htim, uint32_t channel, int16_t speed) {
    if (speed > 0) {
        // 正转
        __HAL_TIM_SET_COMPARE(htim, channel, speed);
        // 设置方向引脚...
    } else {
        // 反转
        __HAL_TIM_SET_COMPARE(htim, channel, -speed);
        // 设置方向引脚...
    }
}

// 示例：设置电机A为50%占空比正转
Motor_SetSpeed(&htim8, TIM_CHANNEL_1, 720);  // 1440/2 = 720
```

---

## 7. GPIO配置

### 7.1 GPIO引脚总览

#### 7.1.1 输出引脚

| 引脚 | 功能 | 模式 | 速度 | 初始状态 | 说明 |
|-----|------|------|------|---------|------|
| PC0 | MOTOR_C_IN1 | 输出推挽 | 低 | 低电平 | 电机C方向1 |
| PC1 | MOTOR_C_IN2 | 输出推挽 | 低 | 低电平 | 电机C方向2 |
| PC2 | MOTOR_B_IN1 | 输出推挽 | 低 | 低电平 | 电机B方向1 |
| PC3 | MOTOR_B_IN2 | 输出推挽 | 低 | 低电平 | 电机B方向2 |
| PC4 | MOTOR_A_IN2 | 输出推挽 | 低 | 低电平 | 电机A方向2 |
| PC5 | MOTOR_A_IN1 | 输出推挽 | 低 | 低电平 | 电机A方向1 |
| PA11 | MOTOR_D_IN1 | 输出推挽 | 低 | 低电平 | 电机D方向1 |
| PA12 | MOTOR_D_IN2 | 输出推挽 | 低 | 低电平 | 电机D方向2 |
| PC12 | MPU_INT | 输出推挽 | 低 | 低电平 | MPU中断控制 |
| PD2 | LED | 输出推挽 | 低 | 低电平 | LED指示灯 |

**电机方向控制逻辑**：
```
IN1  IN2  电机状态
0    0    停止（刹车）
0    1    正转
1    0    反转
1    1    刹车
```

#### 7.1.2 输入引脚

| 引脚 | 功能 | 模式 | 上拉 | 说明 |
|-----|------|------|------|------|
| PB1 | KEY_GPIO | 输入 | 无 | 按键输入 |

#### 7.1.3 复用功能引脚

| 引脚 | 复用功能 | 类型 | 说明 |
|-----|---------|------|------|
| PA0 | TIM5_CH1 | 输入 | 编码器D A相 |
| PA1 | TIM5_CH2 | 输入 | 编码器D B相 |
| PA2 | USART2_TX | 复用推挽 | 串口2发送 |
| PA3 | USART2_RX | 输入 | 串口2接收 |
| PA9 | USART1_TX | 复用推挽 | 串口1发送 |
| PA10 | USART1_RX | 输入 | 串口1接收 |
| PA15 | TIM2_CH1 | 输入 | 编码器A A相 |
| PB3 | TIM2_CH2 | 输入 | 编码器A B相 |
| PB4 | TIM3_CH1 | 输入 | 编码器B A相 |
| PB5 | TIM3_CH2 | 输入 | 编码器B B相 |
| PB6 | TIM4_CH1 | 输入 | 编码器C A相 |
| PB7 | TIM4_CH2 | 输入 | 编码器C B相 |
| PB8 | I2C1_SCL | 复用开漏 | I2C时钟 |
| PB9 | I2C1_SDA | 复用开漏 | I2C数据 |
| PC6 | TIM8_CH1 | 复用推挽 | 电机A PWM |
| PC7 | TIM8_CH2 | 复用推挽 | 电机B PWM |
| PC8 | TIM8_CH3 | 复用推挽 | 电机C PWM |
| PC9 | TIM8_CH4 | 复用推挽 | 电机D PWM |
| PB14 | TIM1_CH2N | 复用推挽 | 舵机1 PWM |
| PB15 | TIM1_CH3N | 复用推挽 | 舵机2 PWM |

#### 7.1.4 模拟输入引脚

| 引脚 | 模拟功能 | 说明 |
|-----|---------|------|
| PB0 | ADC2_IN8 | 电池电压检测 |

---

### 7.2 GPIO特性说明

#### 7.2.1 输出速度

| 速度等级 | 频率 | 应用场景 |
|---------|------|---------|
| GPIO_SPEED_FREQ_LOW | 2MHz | LED、控制引脚 |
| GPIO_SPEED_FREQ_MEDIUM | 10MHz | 一般GPIO |
| GPIO_SPEED_FREQ_HIGH | 50MHz | 高速信号 |

**本项目**：
- 电机方向引脚：低速（节省功耗）
- UART TX：高速（保证信号质量）

#### 7.2.2 复用模式

| 模式 | 说明 | 应用 |
|-----|------|------|
| GPIO_MODE_AF_PP | 复用推挽输出 | UART TX、TIM PWM |
| GPIO_MODE_AF_OD | 复用开漏输出 | I2C SDA/SCL |
| GPIO_MODE_INPUT | 浮空输入 | UART RX、编码器 |

**I2C为什么用开漏？**
- 需要线与功能
- 双向通信
- 外部需要上拉电阻（通常4.7kΩ）

---

## 8. 中断配置

### 8.1 中断优先级

STM32F1使用4位优先级，通常配置为：
- **抢占优先级**（Preemption Priority）：高优先级可以打断低优先级
- **子优先级**（Sub Priority）：抢占优先级相同时，子优先级高的先执行

**本项目配置**：
```
优先级组：NVIC_PriorityGroup_0（4位子优先级，0位抢占优先级）
所有中断优先级：0（最高优先级）
```

### 8.2 启用的中断

| 中断源 | 优先级 | 触发条件 | 处理函数 |
|-------|-------|---------|---------|
| USART1_IRQn | 0 | 接收完成 | USART1_IRQHandler() |
| USART2_IRQn | 0 | 接收完成 | USART2_IRQHandler() |
| TIM6_IRQn | 0 | 定时溢出 | TIM6_IRQHandler() |
| EXTI0_IRQn | ? | PA0外部中断 | EXTI0_IRQHandler() |
| EXTI1_IRQn | ? | PB1外部中断 | EXTI1_IRQHandler() |
| EXTI2_IRQn | ? | PC2外部中断 | EXTI2_IRQHandler() |
| EXTI3_IRQn | ? | PB3外部中断 | EXTI3_IRQHandler() |

**注意**：
- 编码器不使用中断，定时器自动计数
- UART中断用于接收数据（非阻塞）
- TIM6中断用于系统时基

### 8.3 中断使用示例

#### 8.3.1 UART接收中断

```c
// 在stm32f1xx_it.c中
void USART1_IRQHandler(void) {
    HAL_UART_IRQHandler(&huart1);
}

// 在主程序中启动中断接收
uint8_t rx_byte;
HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

// 接收完成回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // 处理接收到的数据
        Comm_RxCallback(huart);

        // 重新启动接收
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}
```

#### 8.3.2 定时器中断

```c
// 在stm32f1xx_it.c中
void TIM6_DAC_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim6);
}

// 在主程序中启动定时器
HAL_TIM_Base_Start_IT(&htim6);

// 定时器溢出回调
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        Run_Times++;  // 循环计数器
    }
}
```

---

## 9. 资源使用统计

### 9.1 外设使用情况

| 外设类型 | 使用数量 | 使用详情 | 备注 |
|---------|---------|---------|------|
| **定时器** | 7/8 | TIM1,2,3,4,5,6,8 | TIM7未使用 |
| **UART** | 2/5 | USART1, USART2 | UART3,4,5未使用 |
| **I2C** | 1/2 | I2C1 | I2C2未使用 |
| **SPI** | 0/2 | - | SPI1,2未使用 |
| **ADC** | 1/3 | ADC2 | ADC1,3未使用 |
| **DMA** | 0 | - | 未使用DMA |

### 9.2 GPIO引脚使用

| 端口 | 总引脚数 | 已使用 | 使用率 | 主要功能 |
|-----|---------|-------|--------|---------|
| GPIOA | 16 | 8 | 50% | UART、编码器、电机D |
| GPIOB | 16 | 10 | 62.5% | I2C、编码器、按键 |
| GPIOC | 16 | 13 | 81.25% | 电机A/B/C、MPU、舵机 |
| GPIOD | 16 | 1 | 6.25% | LED |
| 总计 | 64 | 32 | 50% | - |

### 9.3 定时器资源分配

| 定时器 | 模式 | 功能 | 通道使用 | 时钟频率 |
|-------|------|------|---------|---------|
| TIM1 | PWM | 舵机控制 | CH2N, CH3N | 72MHz |
| TIM2 | Encoder | 编码器A | CH1, CH2 | 72MHz |
| TIM3 | Encoder | 编码器B | CH1, CH2 | 72MHz |
| TIM4 | Encoder | 编码器C | CH1, CH2 | 72MHz |
| TIM5 | Encoder | 编码器D | CH1, CH2 | 72MHz |
| TIM6 | Base | 系统时基 | - | 72MHz → 100Hz |
| TIM7 | - | 未使用 | - | - |
| TIM8 | PWM | 电机驱动 | CH1,2,3,4 | 72MHz |

### 9.4 内存资源估算

| 资源类型 | 容量 | 使用估算 | 使用率 |
|---------|------|---------|--------|
| Flash | 512KB | ~50KB | ~10% |
| SRAM | 64KB | ~10KB | ~15.6% |
| EEPROM | - | - | - |

**Flash占用分布**：
- HAL库代码：~30KB
- 用户代码：~10KB
- 系统初始化：~5KB
- 保留空间：~450KB

---

## 10. 重要配置说明

### 10.1 电源管理

**工作电压**：
- VDD：3.3V（典型值）
- VDD范围：2.0V - 3.6V
- VBAT：1.8V - 3.6V（后备电源）

**功耗估算**：
- 运行模式（72MHz）：~50mA
- 睡眠模式：~5mA
- 待机模式：~2μA

### 10.2 时钟源选择

**外部晶振推荐**：
- 主晶振（HSE）：8MHz（常用、稳定）
- RTC晶振（LSE）：32.768kHz（如使用RTC）

**时钟配置建议**：
```
系统时钟：72MHz（最大性能）
APB1：36MHz（高速外设）
APB2：72MHz（最高速外设）
ADC时钟：≤14MHz（确保精度）
```

### 10.3 调试接口

**SWD接口**（推荐）：
- SWCLK：PA14
- SWDIO：PA13
- 仅需2根线 + GND

**JTAG接口**（需要更多引脚）：
- JTMS: PA13
- JTCK: PA14
- JTDI: PA15
- JTDO: PB3
- NJTRST: PB4

**注意**：部分JTAG引脚被其他功能占用（编码器），建议使用SWD调试。

---

## 11. 修改配置指南

### 11.1 使用STM32CubeMX修改

1. 打开`.ioc`文件
2. 在图形界面中修改外设配置
3. 重新生成代码
4. 用户代码（USER CODE段）会保留

### 11.2 手动修改（不推荐）

如果需要手动修改：

**修改步骤**：
1. 修改`Src/xxx.c`中的初始化函数
2. 同步修改`Inc/xxx.h`中的宏定义
3. 重新编译测试

**注意事项**：
- 不要修改`USER CODE END`之外的代码
- 下次CubeMX生成代码时会被覆盖
- 建议在用户代码段添加初始化后的配置

---

## 12. 常见问题

### 12.1 如何添加新的PWM通道？

**方法1：使用空闲定时器通道**
1. 检查TIM8是否有空闲通道（本项目已用完）
2. 或使用TIM1的空闲通道

**方法2：使用新定时器**
1. CubeMX中启用新定时器
2. 配置PWM参数
3. 生成代码

### 12.2 如何修改PWM频率？

**修改步骤**：
1. 计算新的Period和Prescaler
2. 在`tim.c`中修改`htim8.Init`参数
3. 重新编译

**公式**：
```
目标频率 = 72MHz / (Prescaler + 1) / (Period + 1)
```

### 12.3 如何修改UART波特率？

**方法**：
1. CubeMX中修改USART配置
2. 或在`usart.c`中修改`Init.BaudRate`

**常用波特率**：
- 9600：低速调试
- 115200：标准调试
- 921600：高速数据传输

### 12.4 如何添加外部中断？

**步骤**：
1. CubeMX中配置GPIO为EXTI模式
2. 设置触发方式（上升沿/下降沿/双边沿）
3. 配置中断优先级
4. 实现中断回调函数

---

## 13. 扩展建议

### 13.1 启用DMA

**推荐使用DMA的外设**：
- UART（减轻CPU负担）
- ADC（自动采样）
- 定时器触发ADC

**优点**：
- 降低CPU占用
- 提高数据传输效率
- 实现高速通信

### 13.2 使用看门狗

**独立看门狗（IWDG）**：
- 独立时钟（LSI，40kHz）
- 用于防止程序跑飞
- 超时复位系统

**窗口看门狗（WWDG）**：
- 系统时钟
- 精确监控程序执行时间

### 13.3 低功耗优化

**睡眠模式**：
- CPU停止，外设运行
- 通过中断唤醒

**停止模式**：
- 大部分时钟停止
- SRAM数据保留
- 外部中断唤醒

**待机模式**：
- 最低功耗
- 仅备份寄存器和RTC
- 特定引脚唤醒

---

## 附录A：寄存器配置速查

### A.1 TIM8 PWM配置

```c
htim8.Instance = TIM8;
htim8.Init.Prescaler = 1;                    // 2分频
htim8.Init.Period = 1439;                    // 1440计数
htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
HAL_TIM_PWM_Init(&htim8);
```

### A.2 TIM6定时器配置

```c
htim6.Instance = TIM6;
htim6.Init.Prescaler = 71;                   // 72分频
htim6.Init.Period = 9999;                    // 10000计数
HAL_TIM_Base_Init(&htim6);
```

---

## 附录B：引脚图

```
STM32F103引脚分配（简化版）

                  ┌─────────────────┐
          PC9 ○━━  │                 │  ━━○ PA0-WKUP (TIM5_CH1)
   TIM8_CH4  │    │                 │    │ TIM5_CH2
                  │     STM32F1     │
          PC8 ○━━  │                 │  ━━○ PA1
   TIM8_CH3  │    │                 │    │
                  │                 │
          PC7 ○━━  │                 │  ━━○ PA2
   TIM8_CH2  │    │                 │    │ USART2_TX
                  │                 │
          PC6 ○━━  │                 │  ━━○ PA3
   TIM8_CH1  │    │                 │    │ USART2_RX
                  │                 │
          PC5 ○━━  │                 │  ━━○ PA9
 MOTOR_A_IN1 │    │                 │    │ USART1_TX
                  │                 │
          PC4 ○━━  │                 │  ━━○ PA10
 MOTOR_A_IN2 │    │                 │    │ USART1_RX
                  │                 │
          PC3 ○━━  │                 │  ━━○ PA11
 MOTOR_B_IN2 │    │                 │    │ MOTOR_D_IN1
                  │                 │
          PC2 ○━━  │                 │  ━━○ PA12
 MOTOR_B_IN1 │    │                 │    │ MOTOR_D_IN2
                  │                 │
          PC1 ○━━  │                 │  ━━○ PA15
 MOTOR_C_IN2 │    │                 │    │ TIM2_CH1
                  │                 │
          PC0 ○━━  │                 │
 MOTOR_C_IN1 │    │                 │
                  └─────────────────┘
```

---

**文档结束**

如有疑问或需要更详细的说明，请参考STM32F1xx参考手册或HAL库用户指南。
