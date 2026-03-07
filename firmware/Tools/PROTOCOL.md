# 机器人底盘通信协议文档

## 概述

本文档描述了机器人底盘MCU与上位机之间的串口通信协议。

**基本参数：**
- 串口：UART2
- 波特率：115200（可配置921600）
- 数据位：8
- 校验位：None
- 停止位：1

**协议特性：**
- 字节序：Little-Endian（小端序，低字节在前）
- 校验方式：XOR校验和
- 帧格式：支持带时间戳和不带时间戳两种格式

---

## 帧格式

### 通用格式

```
[帧头][功能码][数据...][可选时间戳][校验和][帧尾]
```

| 字段 | 大小 | 说明 |
|------|------|------|
| 帧头 | 1B | 固定值 0xFC |
| 功能码 | 1B | 见功能码定义表 |
| 数据 | 变长 | int16_t数组（小端序） |
| 时间戳 | 0/8B | MCU上报帧包含，PC下发帧不包含 |
| 校验和 | 1B | XOR校验（从帧头到校验前所有字节） |
| 帧尾 | 1B | 固定值 0xDF |

**校验和计算：**
```python
checksum = 0
for byte in frame[from_start:to_before_checksum]:
    checksum ^= byte
```

### MCU上报帧格式（带时间戳）

**格式：** `[0xFC][FuncCode][Data...][TxTimestamp(8B)][Checksum][0xDF]`

**时间戳：**
- 64位微秒时间戳（小端序）
- 表示MCU开始发送本帧的时间
- 用于时间同步和延迟分析

### PC下发帧格式（不带时间戳）

**格式：** `[0xFC][FuncCode][Data...][Checksum][0xDF]`

---

## 功能码定义

### MCU → PC 上报数据

| 功能码 | 名称 | 数据长度 | 数据格式 | 说明 |
|--------|------|----------|----------|------|
| 0x01 | 电池电压 | 2B | 1×int16 | 电压值（mV） |
| 0x02 | 编码器 | 16B | 8×int16 | 4个编码器位置 |
| 0x03 | IMU数据 | 18B | 9×int16 | 欧拉角+陀螺仪+加速度 |
| 0x10 | PTP同步 | 8B | 4×int16 | 时间同步响应 |

### PC → MCU 控制指令

| 功能码 | 名称 | 数据长度 | 数据格式 | 说明 |
|--------|------|----------|----------|------|
| 0x04 | 电机速度 | 8B | 4×int16 | 4个电机目标速度 |
| 0x05 | PID参数 | 6B | 3×int16 | PID参数（Kp, Ki, Kd） |
| 0x06 | 舵机控制 | - | - | 待实现 |
| 0x10 | PTP同步 | 1B | 1×uint8 | 时间同步请求 |

---

## 详细帧格式

### 0x01 电池电压 (MCU→PC)

**帧长度：** 14字节

```
[0xFC][0x01][电压(2B)][时间戳(8B)][校验和][0xDF]
  1B    1B    2B        8B        1B     1B
```

**数据：**
- 电压：int16_t，单位mV
- 例如：12000 = 12.0V

**示例：**
```
发送：FC 01 E0 2E [时间戳8字节] [校验] DF
解析：12000 mV = 12.0V
```

---

### 0x02 编码器 (MCU→PC)

**帧长度：** 28字节

```
[0xFC][0x02][数据(16B)][时间戳(8B)][校验和][0xDF]
  1B    1B    16B       8B        1B     1B
```

**数据格式：** 8个int16_t组成4个int32_t

```
[ A低 ][ A高 ][ B低 ][ B高 ][ C低 ][ C高 ][ D低 ][ D高 ]
  2B    2B    2B    2B    2B    2B    2B    2B
```

**重构int32_t（小端序）：**
```c
int32_t value = (int32_t)high << 16 | (uint16_t)low;
```

**示例：**
```
发送：FC 02 00 10 00 20 00 30 00 40 [时间戳] [校验] DF
解析：A=4096, B=8192, C=12288, D=16384
```

---

### 0x03 IMU数据 (MCU→PC)

**帧长度：** 30字节

```
[0xFC][0x03][数据(18B)][时间戳(8B)][校验和][0xDF]
  1B    1B    18B       8B        1B     1B
```

**数据结构：** 9个int16_t

| 索引 | 名称 | 单位 | 说明 |
|------|------|------|------|
| 0 | Pitch | °/100 | 俯仰角 |
| 1 | Roll | °/100 | 横滚角 |
| 2 | Yaw | °/100 | 航向角 |
| 3 | Gyro X | rad/s/100 | X轴角速度 |
| 4 | Gyro Y | rad/s/100 | Y轴角速度 |
| 5 | Gyro Z | rad/s/100 | Z轴角速度 |
| 6 | Accel X | G/100 | X轴加速度 |
| 7 | Accel Y | G/100 | Y轴加速度 |
| 8 | Accel Z | G/100 | Z轴加速度 |

**示例：**
```
发送：FC 03 0A 00 14 00 1E 00 28 00 32 00 3C 00 46 00 50 00 5A 00 [时间戳] [校验] DF
解析：
  欧拉角: Pitch=10.00°, Roll=20.00°, Yaw=30.00°
  陀螺仪: X=40.00, Y=50.00, Z=60.00 rad/s
  加速度: X=70.00, Y=80.00, Z=90.00 G
```

---

### 0x04 电机速度 (PC→MCU)

**帧长度：** 12字节

```
[0xFC][0x04][速度A(2B)][速度B(2B)][速度C(2B)][速度D(2B)][校验和][0xDF]
  1B    1B    2B         2B         2B         2B         1B     1B
```

**数据：**
- 速度：int16_t，单位CPS（Counts Per Second）
- 范围：-32768 ~ 32767

**示例：**
```
发送：FC 04 64 00 64 00 C8 00 C8 00 [校验] DF
设置：A=100, B=100, C=200, D=200 CPS
```

---

### 0x05 PID参数 (PC→MCU)

**帧长度：** 11字节

```
[0xFC][0x05][Kp(2B)][Ki(2B)][Kd(2B)][校验和][0xDF]
  1B    1B   2B     2B     2B     1B     1B
```

**数据：**
- 参数：int16_t，实际值需除以100
- 例如：150 = 1.50

**示例：**
```
发送：FC 05 96 00 0A 00 32 00 [校验] DF
设置：Kp=1.50, Ki=0.10, Kd=0.50
```

---

### 0x10 PTP时间同步 (双向)

#### 请求帧 (PC→MCU)

**帧长度：** 5字节

```
[0xFC][0x10][0x01][校验和][0xDF]
  1B    1B   1B    1B     1B
```

**数据：**
- 0x01：PTP同步请求消息类型

#### 响应帧 (MCU→PC)

**帧长度：** 20字节

```
[0xFC][0x10][t2(8B)][t3(8B)][校验和][0xDF]
  1B    1B   8B     8B     1B     1B
```

**时间戳说明：**
- t2：MCU接收请求完成的时间戳（4个int16_t，小端序）
- t3：MCU发送响应开始的时间戳（8字节时间戳，自动添加）

**时间戳重构：**
```python
# 从4个int16_t重构64位时间戳
t2 = (data[3] << 48) | (data[2] << 32) | (data[1] << 16) | data[0]
```

**PTP计算公式：**
```
offset = ((t2 + g_offset - t1) - (t4 - (t3 + g_offset))) / 2
delay  = ((t4 - t1) + ((t3 + g_offset) - (t2 + g_offset))) / 2
```

其中：
- t1：PC发送请求时刻
- t2：MCU接收请求时刻
- t3：MCU发送响应时刻
- t4：PC接收响应时刻
- g_offset：MCU到Linux的时间偏移

---

## 工具使用

### serial_reader.py

读取并解析MCU上报数据：

```bash
# 列出可用串口
python3 serial_reader.py -l

# 读取所有数据
python3 serial_reader.py -p /dev/ttyUSB0

# 只显示IMU数据
python3 serial_reader.py -p /dev/ttyUSB0 -f imu

# 显示原始帧数据
python3 serial_reader.py -p /dev/ttyUSB0 -r

# 保存到CSV文件
python3 serial_reader.py -p /dev/ttyUSB0 -s data.csv

# 高波特率 + 统计信息
python3 serial_reader.py -p /dev/ttyUSB0 -b 921600 --stats
```

### frame_generator.py

生成PC下发控制帧：

```bash
# 交互模式
python3 frame_generator.py

# 生成电机速度帧
python3 frame_generator.py motor 100 100 200 200

# 生成PID参数帧
python3 frame_generator.py pid 1.5 0.1 0.5
```

### sync_ptp.py

PTP时间同步测试：

```bash
# 默认参数（0.5s间隔）
python3 sync_ptp.py /dev/ttyUSB0

# 自定义间隔
python3 sync_ptp.py /dev/ttyUSB0 -i 0.2

# 高波特率
python3 sync_ptp.py /dev/ttyUSB0 -b 921600
```

---

## 数据流示例

### 正常运行流程

```
PC                             MCU
|                              |
|<------[0x01] 电池电压--------|
|<------[0x02] 编码器----------|  100Hz (IMU中断触发)
|<------[0x03] IMU数据---------|
|                              |
|---[0x04] 电机速度----------->|
|---[0x05] PID参数------------>|
|                              |
|---[0x10] PTP请求------------>|
|<------[0x10] PTP响应---------|
|                              |
```

### 时间同步流程

```
t1: PC发送PTP请求
t2: MCU接收请求（记录）
t3: MCU发送响应（记录）
t4: PC接收响应（记录）

计算: offset = ((t2 - t1) - (t4 - t3)) / 2
```

---

## 调试技巧

### 1. 检查帧完整性

使用 `-r` 参数查看原始帧数据：

```bash
python3 serial_reader.py -p /dev/ttyUSB0 -r
```

输出示例：
```
[TX:12345.678ms] 电池电压: 12000 mV (12.00 V)
  原始: FC 01 E0 2E [时间戳8字节] [校验] DF
```

### 2. 过滤特定数据

只查看关心类型的数据：

```bash
# 只看IMU
python3 serial_reader.py -p /dev/ttyUSB0 -f imu

# 只看编码器
python3 serial_reader.py -p /dev/ttyUSB0 -f encoder

# 同时查看多个
python3 serial_reader.py -p /dev/ttyUSB0 -f imu encoder
```

### 3. 记录数据用于分析

保存到CSV文件：

```bash
python3 serial_reader.py -p /dev/ttyUSB0 -s log.csv
```

然后用Excel/Pandas分析。

### 4. 检查通信频率

使用 `--stats` 参数实时查看频率：

```bash
python3 serial_reader.py -p /dev/ttyUSB0 --stats
```

输出示例：
```
统计摘要
============================================================
运行时长: 10.0 秒

总帧数: 1024

[电池电压]
  帧数: 50 (5.0 fps)

[编码器]
  帧数: 1000 (100.0 fps)
  频率: 100.0 Hz

[IMU数据]
  帧数: 1000 (100.0 fps)
  频率: 100.0 Hz
============================================================
```

---

## 常见问题

### Q: 为什么有些帧校验失败？
A: 可能原因：
- 波特率不匹配
- 串口干扰
- 缓冲区溢出（降低数据率或增加波特率）

### Q: 编码器数据为什么是8个int16？
A: 编码器使用32位位置值，需要两个16位组合：
```
int32_t pos = (int32_t)high << 16 | (uint16_t)low;
```

### Q: PTP时间戳怎么解读？
A: 响应帧包含两个时间戳：
- t2（数据段）：MCU接收时刻，4×int16_t重构
- t3（自动添加）：MCU发送时刻，8字节完整时间戳

### Q: 如何测试电机控制？
A: 使用frame_generator.py生成帧，然后通过串口发送：
```bash
python3 frame_generator.py motor 100 100 100 100
# 复制输出的十六进制，通过串口工具发送
```

---

## 版本历史

- v1.0 - 初始版本，支持基本通信
- v2.0 - 添加时间戳支持
- v3.0 - 优化DMA传输，降低CPU占用
- v4.0 - 合并IMU数据，编码器同步上报

---

## 参考信息

- MCU: STM32F103xE
- IMU: MPU6050 (DMP)
- 编码器: 4×AB相编码器
- 电机: 4×直流减速电机
- 固件位置: `/root/project/RobotChassis/firmware/`
