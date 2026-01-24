# 机器人底盘上位机工具说明

## 概述

本目录包含两个Python上位机工具，用于与机器人底盘固件通信：

1. **`serial_reader.py`** - 串口数据读取器
2. **`serial_controller.py`** - 控制指令发送器

---

## 一、串口数据读取器 (`serial_reader.py`)

### 功能

从串口读取数据并解析通信协议，实时显示：
- 电池电压
- 编码器位置
- 陀螺仪数据
- 加速度数据
- 欧拉角数据
- 电机速度
- PID参数
- 舵机角度

### 安装依赖

```bash
pip install pyserial
```

### 使用方法

#### 1. 列出可用串口

```bash
python serial_reader.py --list
```

输出示例：
```
可用串口设备:
  1. /dev/ttyUSB0 - USB Serial
     制造商: FTDI
```

#### 2. 读取所有数据

```bash
# Linux
python serial_reader.py -p /dev/ttyUSB0

# Windows
python serial_reader.py -p COM3

# 自动选择串口（只有一个串口时）
python serial_reader.py
```

#### 3. 过滤特定数据类型

```bash
# 只显示电池电压和欧拉角
python serial_reader.py -p /dev/ttyUSB0 -f battery euler

# 可选的数据类型：
#   battery   - 电池电压
#   encoder   - 编码器
#   gyro      - 陀螺仪
#   accel     - 加速度
#   euler     - 欧拉角
#   motor     - 电机速度
#   pid       - PID参数
#   servo     - 舵机控制
```

#### 4. 显示原始数据

```bash
python serial_reader.py -p /dev/ttyUSB0 --raw
```

### 输出示例

```
[14:23:45.123] 电池电压: 12000 mV (12.00 V)
[14:23:45.134] 欧拉角: Roll=0.50°, Pitch=1.20°, Yaw=45.30°
[14:23:45.145] 陀螺仪: X=0.01 rad/s, Y=-0.02 rad/s, Z=0.00 rad/s
[14:23:45.156] 加速度: X=0.00 G, Y=0.01 G, Z=1.00 G
[14:23:45.167] 编码器: Motor A=1234, B=-567, C=890, D=-321
```

---

## 二、控制指令发送器 (`serial_controller.py`)

### 功能

发送控制指令到机器人底盘：
- 设置电机速度
- 设置PID参数
- 设置舵机角度

### 使用方法

#### 1. 交互式控制模式

```bash
python serial_controller.py -p /dev/ttyUSB0 --interactive
```

交互命令：
```
命令:
  w/s/a/d - 前进/后退/左转/右转
  space   - 停止
  q       - 退出
  p <kp> <ki> <kd> - 设置PID参数
  s <servo1> <servo2> - 设置舵机
```

示例：
```
> w              # 前进
> a              # 左转
> space          # 停止
> p 1.5 0.2 0.8  # 设置PID参数
> s 90 45        # 设置舵机角度
> q              # 退出
```

#### 2. 运行示例程序

```bash
python serial_controller.py -p /dev/ttyUSB0 --example
```

示例程序会：
1. 设置PID参数 (Kp=1.5, Ki=0.2, Kd=0.8)
2. 设置舵机到90度
3. 电机前进2秒
4. 电机后退2秒
5. 停止所有电机

### Python代码示例

```python
from serial_controller import RobotController

# 连接到机器人
controller = RobotController('/dev/ttyUSB0')
controller.connect()

# 设置电机速度 (单位: centi-CPS = CPS/100)
controller.set_motor_speed(100, 100, 100, 100)  # 前进

# 等待2秒
import time
time.sleep(2)

# 停止
controller.stop_all_motors()

# 设置PID参数
controller.set_pid(1.5, 0.2, 0.8)

# 设置舵机
controller.set_servo(90, 45)

# 断开连接
controller.disconnect()
```

---

## 三、通信协议说明

### 帧格式

```
[0xFC][FuncCode][Data...][Checksum][0xDF]
```

| 字段 | 说明 |
|------|------|
| 帧头 | 0xFC |
| 功能码 | 1字节 |
| 数据 | 可变长度int16_t数组（大端序） |
| 校验和 | XOR校验（包括帧头、功能码和数据段） |
| 帧尾 | 0xDF |

### 功能码定义

| 功能码 | 名称 | 数据长度 | 说明 |
|--------|------|----------|------|
| 0x01 | 电池电压 | 2字节 (1×int16) | 单位：mV |
| 0x02 | 编码器 | 8字节 (4×int16) | 编码器计数值 |
| 0x03 | 陀螺仪 | 6字节 (3×int16) | 单位：rad/s ×100 |
| 0x04 | 加速度 | 6字节 (3×int16) | 单位：G ×100 |
| 0x05 | 欧拉角 | 6字节 (3×int16) | 单位：度 ×100 |
| 0x06 | 电机速度 | 8字节 (4×int16) | 单位：centi-CPS |
| 0x07 | PID参数 | 6字节 (3×int16) | 单位：×100 |
| 0x08 | 舵机控制 | 2字节 (2×int8) | 角度值 (0-180) |

### 数据格式

所有数据使用**大端序**（Big-Endian）int16_t格式，除了舵机控制使用int8_t。

---

## 四、常见问题

### 1. 串口权限错误 (Linux)

```bash
# 添加用户到dialout组
sudo usermod -a -G dialout $USER

# 重新登录后生效
```

### 2. 找不到串口设备

- 检查USB连接
- 使用 `--list` 参数查看可用串口
- Windows: 检查设备管理器中的端口
- Linux: 检查 `dmesg | grep tty`

### 3. 数据乱码

- 确认波特率为 **115200**
- 检查数据位、停止位、奇偶校验配置

---

## 五、开发说明

### 扩展读取器

在 `serial_reader.py` 中的 `format_data()` 函数中添加自定义格式化逻辑：

```python
def format_data(func_code: int, data: list) -> str:
    if func_code == 0x01:
        # 自定义格式化
        return f"电压: {data[0]/1000:.2f}V"
    # ...
```

### 扩展控制器

在 `serial_controller.py` 中添加新的控制函数：

```python
def custom_command(self, param1, param2):
    frame = build_frame(0x09, [param1, param2])  # 使用自定义功能码
    self.send_frame(frame)
```

---

## 六、技术支持

如有问题，请查看：
- 固件源码：`USER/Communication/comm_protocol.c`
- 协议定义：`USER/Communication/comm_protocol.h`
- 项目文档：`CLAUDE.md`
