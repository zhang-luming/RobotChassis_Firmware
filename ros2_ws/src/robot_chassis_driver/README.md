# RobotChassis ROS2 驱动

基于 STM32F103 的四轮差速底盘 ROS2 驱动程序。

## 功能特性

- ✅ **串口通信**: 与 MCU 串口通信（115200 baud）
- ✅ **PTP 时间同步**: 亚毫秒级时间戳同步
- ✅ **传感器数据融合**: 编码器 + IMU（100Hz）
- ✅ **里程计发布**: `nav_msgs/Odometry` + TF 变换
- ✅ **IMU 数据发布**: `sensor_msgs/Imu`（含姿态四元数）
- ✅ **速度控制**: `geometry_msgs/Twist` → 四电机 CPS 控制

## 硬件架构

```
MCU (STM32F103)
├── 4x 编码器 (TIM2/3/4/5)
├── MPU6050 IMU (I2C, 100Hz DMP)
└── UART2 (115200, 8N1)
     ↔ ROS2 驱动节点
```

## 通信协议

### MCU → ROS2

| 功能码 | 数据大小 | 说明 |
|--------|----------|------|
| `0x10` | 20字节 | PTP 时间同步响应 |
| `0x20` | 38字节 | 传感器数据（编码器4 + IMU9） |

### ROS2 → MCU

| 功能码 | 数据大小 | 说明 |
|--------|----------|------|
| `0x04` | 12字节 | 电机速度控制（4×int16 CPS） |
| `0x10` | 6字节 | PTP 同步请求 |

## 构建方法

```bash
cd ros2_ws

# 安装依赖
rosdep install --from-paths src --ignore-src -r -y

# 构建工作空间
colcon build --packages-select robot_chassis_driver

# 加载环境
source install/setup.bash
```

## 配置说明

配置文件：`config/robot_chassis.yaml`

### 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `serial_port` | `/dev/ttyUSB0` | 串口设备路径 |
| `serial_baudrate` | `115200` | 串口波特率 |
| `wheelbase` | `0.5` | 轮距（米） |
| `wheel_radius` | `0.1` | 轮半径（米） |
| `ticks_per_meter` | `1000.0` | 编码器每米脉冲数 |
| `odom_x_scale` | `1.0` | 线速度校准系数 |
| `odom_yaw_scale` | `1.0` | 角速度校准系数 |

### 参数标定

**线速度标定**：
```bash
# 1. 直行 10 米，测量里程计读数
# 2. 计算：odom_x_scale = 实际距离 / 测量距离
```

**角速度标定**：
```bash
# 1. 原地旋转 10 圈，测量里程计读数
# 2. 计算：odom_yaw_scale = 实际角度 / 测量角度
```

## 使用方法

### 启动驱动节点

```bash
# 使用默认串口 /dev/ttyUSB0
ros2 launch robot_chassis_driver robot_chassis.launch.py

# 指定串口设备
ros2 launch robot_chassis_driver robot_chassis.launch.py serial_port:=/dev/ttyACM0
```

### 发送速度指令

```bash
# 直行 0.5 m/s
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# 原地旋转 1.0 rad/s
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 1.0}}"
```

### 查看传感器数据

```bash
# 里程计
ros2 topic echo /odom

# IMU
ros2 topic echo /imu

# TF 变换
ros2 run tf2_ros tf2_echo odom base_link
```

## ROS 话题

### 发布的话题

| 话题 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `/odom` | `nav_msgs/Odometry` | 100Hz | 里程计数据 |
| `/imu` | `sensor_msgs/Imu` | 100Hz | IMU 数据 |
| `/tf` | `tf2_msgs/TFMessage` | 100Hz | odom→base_link 变换 |

### 订阅的话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 速度控制指令 |

## 坐标系

```
odom → base_link → imu_link
```

- `odom`: 里程计坐标系（世界坐标系）
- `base_link`: 机器人底盘坐标系
- `imu_link`: IMU 传感器坐标系

## 运动学模型

四轮差速驱动：

```
左轮速度 = v - ω × wheelbase / 2
右轮速度 = v + ω × wheelbase / 2

其中：
- v: 线速度 (m/s)
- ω: 角速度 (rad/s)
- wheelbase: 轮距 (m)
```

## 故障排查

### 串口权限问题

```bash
# 添加用户到 dialout 组
sudo usermod -a -G dialout $USER

# 重新登录后生效
```

### 节点无法连接 MCU

```bash
# 检查串口设备
ls -l /dev/ttyUSB*

# 检查串口波特率
stty -F /dev/ttyUSB0

# 查看原始串口数据
sudo apt install minicom
minicom -D /dev/ttyUSB0 -b 115200
```

### PTP 同步失败

检查固件是否正确响应 PTP 请求（功能码 0x10）：

```bash
# 查看驱动日志
ros2 launch robot_chassis_driver robot_chassis.launch.py | grep PTP
```

## 相关链接

- **MCU 固件**: `../../firmware/`
- **通信协议**: `../../firmware/USER/Communication/comm_protocol.h`
- **调试工具**: `../../firmware/Tools/serial_reader.py`

## 许可证

MIT License
