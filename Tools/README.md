# 通信帧生成工具

## 工具位置

```
Tools/frame_generator.py
```

## 使用方法

### 方式1：命令行模式（推荐）

**生成电机速度控制帧**：
```bash
python3 Tools/frame_generator.py motor <A> <B> <C> <D>

# 示例：所有电机速度为10
python3 Tools/frame_generator.py motor 10 10 10 10
# 输出: FC 06 00 0A 00 0A 00 0A 00 0A FA DF
```

**生成PID参数设置帧**：
```bash
python3 Tools/frame_generator.py pid <Kp> <Ki> <Kd>

# 示例：Kp=1.5, Ki=0.1, Kd=0.05
python3 Tools/frame_generator.py pid 1.5 0.1 0.05
# 输出: FC 07 00 96 00 0A 00 05 62 DF
```

### 方式2：交互模式

```bash
python3 Tools/frame_generator.py
```

然后按照提示选择功能和输入参数。

## 常用参数示例

### 电机速度帧

| 场景 | 速度值 | 命令 |
|------|--------|------|
| 停止 | 0 0 0 0 | `python3 Tools/frame_generator.py motor 0 0 0 0` |
| 慢速 | 5 5 5 5 | `python3 Tools/frame_generator.py motor 5 5 5 5` |
| 中速 | 10 10 10 10 | `python3 Tools/frame_generator.py motor 10 10 10 10` |
| 快速 | 20 20 20 20 | `python3 Tools/frame_generator.py motor 20 20 20 20` |

### PID参数帧

| 场景 | Kp | Ki | Kd | 命令 |
|------|----|----|----|------|
| 柔软 | 1.0 | 0.05 | 0.01 | `python3 Tools/frame_generator.py pid 1.0 0.05 0.01` |
| 中等 | 1.5 | 0.10 | 0.05 | `python3 Tools/frame_generator.py pid 1.5 0.1 0.05` |
| 较硬 | 2.0 | 0.15 | 0.08 | `python3 Tools/frame_generator.py pid 2.0 0.15 0.08` |
| 快速响应 | 3.0 | 0.20 | 0.10 | `python3 Tools/frame_generator.py pid 3.0 0.2 0.1` |

## 帧格式说明

### 电机速度控制帧

```
[FC][06][速度A高][速度A低][速度B高][速度B低][速度C高][速度C低][速度D高][速度D低][Checksum][DF]
  0   1    2      3     4      5     6      7     8      9     10       11
```

- 速度单位：centi-CPS（CPS/100）
- 数据格式：大端序int16_t
- 校验和：前10个字节的XOR

### PID参数设置帧

```
[FC][07][Kp高][Kp低][Ki高][Ki低][Kd高][Kd低][Checksum][DF]
  0   1   2    3    4    5    6    7      8        9
```

- 参数单位：放大100倍
- 数据格式：大端序int16_t
- 校验和：前8个字节的XOR
- 应用范围：所有4个电机

## 使用流程

1. 运行工具生成帧
2. 复制输出的十六进制字符串
3. 在串口工具中以十六进制格式发送
