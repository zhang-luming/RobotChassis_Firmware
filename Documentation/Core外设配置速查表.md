# Core外设配置速查表

## 快速参考指南

本文档提供Core目录外设配置的快速参考，便于开发时查阅。

---

## 定时器配置总览

| 定时器 | 功能 | 频率 | 分辨率 | 引脚 |
|--------|------|------|--------|------|
| **TIM1** | 舵机PWM | 50Hz | 10000级 | PB14, PB15 |
| **TIM2** | 编码器A | - | 16位 | PA15, PB3 |
| **TIM3** | 编码器B | - | 16位 | PB4, PB5 |
| **TIM4** | 编码器C | - | 16位 | PB6, PB7 |
| **TIM5** | 编码器D | - | 16位 | PA0, PA1 |
| **TIM6** | 系统时基 | 100Hz | - | - |
| **TIM8** | 电机PWM | 25kHz | 1440级 | PC6-PC9 |

---

## 通信外设

### UART配置

| UART | 波特率 | TX | RX | 功能 |
|------|--------|-----|-----|------|
| USART1 | 115200 | PA9 | PA10 | 调试输出 |
| USART2 | 115200 | PA2 | PA3 | 通信协议 |

### I2C配置

| I2C | 频率 | SCL | SDA | 功能 |
|-----|------|-----|-----|------|
| I2C1 | 100kHz | PB8 | PB9 | 硬件I2C（备用） |

---

## GPIO引脚映射

### 电机控制引脚

| 电机 | IN1 | IN2 | PWM | TIM通道 |
|-----|-----|-----|-----|---------|
| A | PC5 | PC4 | PC6 | TIM8_CH1 |
| B | PC2 | PC3 | PC7 | TIM8_CH2 |
| C | PC0 | PC1 | PC8 | TIM8_CH3 |
| D | PA11 | PA12 | PC9 | TIM8_CH4 |

**方向控制**：
- IN1=0, IN2=0 → 刹车
- IN1=0, IN2=1 → 正转
- IN1=1, IN2=0 → 反转
- IN1=1, IN2=1 → 刹车

### 编码器引脚

| 编码器 | A相 | B相 | 定时器 |
|--------|------|------|--------|
| A | PA15 | PB3 | TIM2 |
| B | PB4 | PB5 | TIM3 |
| C | PB6 | PB7 | TIM4 |
| D | PA0 | PA1 | TIM5 |

### 其他GPIO

| 引脚 | 功能 | 类型 | 说明 |
|-----|------|------|------|
| PB0 | ADC2_IN8 | 模拟输入 | 电池电压检测 |
| PB1 | KEY_GPIO | 数字输入 | 按键 |
| PC12 | MPU_INT | 数字输出 | MPU中断控制 |
| PD2 | LED | 数字输出 | LED指示灯 |

---

## PWM参数计算

### TIM8（电机PWM）

```c
频率 = 72MHz / (Prescaler + 1) / (Period + 1)
     = 72MHz / 2 / 1440
     = 25kHz

占空比(%) = CCR / (Period + 1) × 100
          = CCR / 1440 × 100
```

**示例**：
```c
// 50%占空比
__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 720);

// 100%占空比
__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 1440);

// 0%占空比
__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
```

### TIM1（舵机PWM）

```c
频率 = 72MHz / (143 + 1) / (9999 + 1)
     = 72MHz / 144 / 10000
     = 50Hz (周期20ms)

脉宽 = CCR / 10000 × 20ms
```

**示例**：
```c
// 0.5ms (0度)
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 250);

// 1.5ms (90度/中位)
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 750);

// 2.5ms (180度)
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1250);
```

---

## 系统时基（TIM6）

```c
中断频率 = 72MHz / (71 + 1) / (9999 + 1)
         = 72MHz / 72 / 10000
         = 100Hz (每10ms一次)
```

**时序控制**：
```c
// 每10ms
if (Run_Times % 1 == 0) { }

// 每40ms
if (Run_Times % 4 == 0) { }

// 每100ms
if (Run_Times % 10 == 0) { }

// 每1秒
if (Run_Times % 100 == 0) { }
```

---

## ADC配置

### ADC2参数

```c
通道: ADC_CHANNEL_8
采样时间: 1.5个周期
转换模式: 连续转换
数据对齐: 右对齐
引脚: PB0
```

**电压计算**：
```c
// 启动ADC
HAL_ADC_Start(&hadc2);

// 等待转换完成
HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);

// 读取值
uint32_t raw = HAL_ADC_GetValue(&hadc2);

// 转换为电压 (3.3V参考，12位ADC)
float voltage = (raw / 4095.0f) * 3.3f;

// 如果有分压电路
float battery_voltage = voltage * 分压系数;
```

---

## 中断优先级

所有外设中断优先级均设为0（最高优先级）

| 中断源 | 抢占优先级 | 子优先级 | 功能 |
|--------|----------|---------|------|
| USART1 | 0 | 0 | 调试串口接收 |
| USART2 | 0 | 0 | 通信串口接收 |
| TIM6 | 0 | 0 | 系统时基 |

---

## 时钟配置

```
系统时钟: 72MHz
AHB: 72MHz
APB1: 36MHz
APB2: 72MHz
ADC: 12MHz
```

---

## 常用代码片段

### 启动定时器PWM

```c
// 电机PWM
HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

// 舵机PWM
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

// 系统时基
HAL_TIM_Base_Start_IT(&htim6);
```

### 启动UART接收中断

```c
uint8_t rx_byte;
HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
```

### 读取编码器

```c
// 读取计数值
int16_t encoder_a = __HAL_TIM_GET_COUNTER(&htim2);
int16_t encoder_b = __HAL_TIM_GET_COUNTER(&htim3);
int16_t encoder_c = __HAL_TIM_GET_COUNTER(&htim4);
int16_t encoder_d = __HAL_TIM_GET_COUNTER(&htim5);
```

---

## 资源占用

| 资源 | 使用量 | 总量 | 占用率 |
|-----|--------|------|--------|
| GPIO | 32 | 64 | 50% |
| 定时器 | 7 | 8 | 87.5% |
| UART | 2 | 5 | 40% |
| I2C | 1 | 2 | 50% |

---

## 修改配置注意事项

### ⚠️ CubeMX生成的代码

**可安全修改的位置**：`/* USER CODE BEGIN */` 和 `/* USER CODE END */` 之间

**会被覆盖的位置**：其他所有代码

### ⚠️ 重要提醒

1. **定时器引脚已完全使用**，如需新增PWM需要更换定时器或使用软件PWM
2. **编码器模式占用4个定时器**，这是为了实现四轮独立测速
3. **UART中断优先级设为0**，可能会影响其他实时性要求
4. **TIM8 PWM频率25kHz**，已超出人耳听觉范围，避免啸叫

---

## 调试技巧

### 查看定时器计数值

```c
// 实时查看编码器计数
printf("ENC_A: %d, ENC_B: %d\n",
       __HAL_TIM_GET_COUNTER(&htim2),
       __HAL_TIM_GET_COUNTER(&htim3));
```

### 查看PWM占空比

```c
// 查看当前PWM值
uint32_t pwm1 = __HAL_TIM_GET_COMPARE(&htim8, TIM_CHANNEL_1);
printf("PWM1: %lu / 1440\n", pwm1);
```

### 查看系统时钟

```c
// 查看各总线时钟
printf("SYSCLK: %lu Hz\n", HAL_RCC_GetSysClockFreq());
printf("HCLK: %lu Hz\n", HAL_RCC_GetHCLKFreq());
printf("PCLK1: %lu Hz\n", HAL_RCC_GetPCLK1Freq());
printf("PCLK2: %lu Hz\n", HAL_RCC_GetPCLK2Freq());
```

---

**快速参考结束**

详细配置请参考《Core外设配置详解.md》
