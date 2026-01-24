# DMA工作机制详解

## 概述

DMA (Direct Memory Access，直接内存访问) 是一种硬件数据传输机制，可以在不占用CPU的情况下，在内存和外设之间直接传输数据。

## 为什么使用DMA

### 不使用DMA的问题

```c
// 传统方式：CPU参与的数据传输
uint8_t data[100];
for (int i = 0; i < 100; i++) {
    USART1->DR = data[i];           // CPU写入
    while (!(USART1->SR & 0x40));   // CPU等待发送完成
}
```

**问题**：
- CPU必须参与每个字节的传输
- 传输过程中CPU无法执行其他任务
- 高速传输时CPU占用率高

### 使用DMA的优势

```c
// DMA方式：硬件自动传输
HAL_UART_Transmit_DMA(&huart1, data, 100);  // 启动后立即返回
// CPU可以继续执行其他任务
while (uart_tx_state != READY);  // 等待传输完成标志
```

**优势**：
- CPU只需启动传输，传输过程由DMA硬件完成
- CPU占用率大幅降低
- 适合大批量数据传输

## DMA工作原理

### 基本架构

```
┌─────────────┐         ┌──────────────┐
│   内存     │         │    外设       │
 │  (SRAM)    │         │  (UART/ADC)   │
 └─────┬───────┘         └──────┬───────┘
       │                       │
       │         ┌─────────▼─────────┐
       │         │                   │
       │         │      DMA控制器     │
       │         │   (AHB Bus Master)  │
       │         │                   │
       │         └─────────┬─────────┘
       │                   │
       ▼                   ▼
   DMA触发信号        DMA请求/应答
```

### DMA传输流程

1. **配置阶段**
   - 设置源地址（内存地址）
   - 设置目标地址（外设数据寄存器）
   - 设置传输数据量
   - 配置传输方向（M2M, M2P, P2M, P2P）

2. **启动传输**
   - 使能DMA通道
   - DMA等待触发信号

3. **传输执行**
   - DMA向CPU请求总线控制权
   - 获得控制权后，直接从源地址读取数据
   - 将数据写入目标地址
   - 传输完成后释放总线

4. **传输完成**
   - DMA设置中断标志
   - CPU可读取传输状态

## DMA寄存器

### 关键寄存器（以STM32F1为例）

| 寄存器 | 说明 |
|--------|------|
| **CCR (通道配置寄存器)** | 传输方向、循环模式、外设地址等 |
| **CNDTR (数据项数量寄存器)** | 剩余待传输数据量 |
| **CPAR (外设地址寄存器)** | 外设数据寄存器地址 |
| **CMAR (内存地址寄存器)** | 内存数据地址 |
| **ISR (中断状态寄存器)** | 传输完成中断标志 |

## DMA传输类型

### 1. 内存到内存 (M2M)
```
内存 → DMA → 内存
用途：内存数据复制
```

### 2. 内存到外设 (M2P)
```
内存 → DMA → 外设寄存器
用途：UART发送、ADC数据采集
```

### 3. 外设到内存 (P2M)
```
外设寄存器 → DMA → 内存
用途：UART接收、ADC数据采集
```

### 4. 外设到外设 (P2P)
```
外设寄存器 → DMA → 外设寄存器
用途：外设间数据转发
```

## STM32 DMA配置示例

### UART DMA发送配置

```c
// 1. 配置DMA通道
DMA_HandleTypeDef hdma_usart1_tx;
hdma_usart1_tx.Instance = DMA1_Channel4;  // USART1_TX使用DMA1 Ch4
hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
hdma_usart1_tx.PeriphInc = DMA_PINC_DISABLE;  // 外设地址不自增
hdma_usart_tx.MemInc = DMA_MINC_ENABLE;     // 内存地址自增
hdma_usart_tx.Mode = DMA_NORMAL;             // 正常模式（非循环）
hdma_usart_tx.Priority = DMA_PRIORITY_LOW;

// 2. 配置DMA链接到UART外设
__HAL_LINKDMA(&huart1, hdmatx_0_74);

// 3. 启动DMA传输
HAL_UART_Transmit_DMA(&huart1, data, size);

// 4. 等待传输完成
while (huart1.gState != HAL_UART_STATE_READY);
```

### UART DMA接收配置（循环模式）

```c
// 配置循环模式，自动接收数据
hdma_usart2_rx.Mode = DMA_CIRCULAR;           // 循环模式
hdma_usart2_rx.BufferSize = RX_BUFFER_SIZE;  // 缓冲区大小

// 启动DMA接收
HAL_UART_Receive_DMA(&huart2, rx_buffer, RX_BUFFER_SIZE);

// DMA自动填充缓冲区，CPU读取处理
```

## DMA与中断配合

### 传输完成中断

```c
void DMA1_Channel4_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_usart1_tx);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    // 传输完成回调
    if (huart->Instance == USART1) {
        // 数据发送完成处理
    }
}
```

### 半传输完成中断（用于大数据分批处理）

```c
void DMA1_Channel4_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_usart1_tx);
}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart) {
    // 半传输完成：前半部分数据已发送
    // 可以加载后半部分数据
}
```

## DMA循环模式

### 工作原理

```
缓冲区: [数据0][数据1][数据2][数据3] (4字节)
         ↑      ↑      ↑      ↑
         └──────┴──────┴──────┘ (循环)

DMA自动回绕：
0 → 1 → 2 → 3 → 0 → 1 → 2 → 3 → ...
```

### 配置要点

```c
hdma_usart2_rx.Mode = DMA_CIRCULAR;
hdma_usart2_rx.BufferSize = RX_BUFFER_SIZE;  // 必须是2的幂次
```

**数据读取技巧**：
```c
#define RX_BUFFER_SIZE 256

volatile uint16_t rx_read_index = 0;
volatile uint16_t rx_write_index = 0;

// 在DMA中断回调中
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        rx_write_index = RX_BUFFER_SIZE - hdma_usart2_rx.Instance->CNDTR;
    }
}

// 在主循环中读取数据
uint16_t available_data(void) {
    if (rx_write_index >= rx_read_index) {
        return rx_write_index - rx_read_index;
    } else {
        return RX_BUFFER_SIZE - rx_read_index + rx_write_index;
    }
}
```

## DMA与中断优先级冲突

### 问题场景

DMA传输需要占用总线，可能同时有其他中断请求总线控制权。

### 解决方案

1. **DMA优先级**：DMA控制器有优先级仲裁机制
2. **总线矩阵**：STM32有总线矩阵，可以同时传输
3. **中断优先级**：DMA中断优先级通常低于关键中断

## 常见问题

### Q1: DMA传输数据错乱
**原因**：
- DMA配置的外设地址不自增，但内存地址自增
- 外设寄存器需要固定地址

**解决**：
```c
hdma_usart_tx.PeriphInc = DMA_PINC_DISABLE;  // 外设地址固定
hdma_usart_tx.MemInc = DMA_MINC_ENABLE;      // 内存地址自增
```

### Q2: DMA传输后外设无反应
**原因**：
- DMA外设地址错误
- 外设时钟未使能
- 外设未正确配置（如UART未使能DMA请求）

**检查**：
```c
// 检查USART DMA请求是否使能
// 在USART配置中需要设置
USART1->CR3 |= USART_CR3_DMAT;  // 使能DMA发送请求
```

### Q3: DMA循环模式数据丢失
**原因**：
- CPU读取速度慢于DMA写入速度
- 读指针追不上写指针

**解决**：使用双缓冲区或提高读取频率

## DMA优化建议

### 1. 传输大小选择
- 小数据（<16字节）：直接用CPU
- 中等数据（16-1KB）：使用DMA一次传输
- 大数据（>1KB）：使用DMA+半传输中断

### 2. 内存对齐
```c
// 确保DMA数据地址对齐（4字节对齐最佳）
__attribute__((aligned(4))) uint8_t dma_buffer[1024];
```

### 3. 使用缓存（Cache一致性）
```c
// 如果使用DMA，需要考虑Cache一致性
SCB_CleanInvalidateDCache_by_Addr((uint32_t)addr, size);
```

## 参考资料

- STM32F103x Reference Manual - DMA控制器章节
- STM32 HAL库 DMA使用指南
 - STM32F1 DMA配置详解

---

**相关文档**：
- `02_硬件/Core外设配置详解.md` - 外设配置说明
- `05_通信/` - UART DMA通信实现

---

**文档版本**：1.0
**最后更新**：2026-01-24
