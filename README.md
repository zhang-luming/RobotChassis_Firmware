# RobotChassis - 四驱机器人底盘项目

基于 STM32F103xE 的四轮差速驱动机器人底盘项目，包含底层固件和 ROS2 上层应用。

## 项目结构

```
.
├── firmware/           # STM32 固件源码
│   ├── USER/          # 应用模块（电机控制、IMU、通信等）
│   ├── Core/          # HAL/Core 初始化
│   ├── Drivers/       # STM32 HAL 驱动
│   ├── Tools/         # 工具脚本
│   ├── build.sh       # 构建脚本
│   ├── BUILD.md       # 构建说明
│   └── README.md      # 固件说明
│
└── ros2_ws/           # ROS2 工作空间
    └── src/           # ROS2 包源码
```

## 快速开始

### 固件编译

```bash
cd firmware
./build.sh
```

详细说明请查看 [firmware/README.md](firmware/README.md) 和 [firmware/BUILD.md](firmware/BUILD.md)

### ROS2 工作空间

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 硬件规格

- **MCU**: STM32F103xE (256KB Flash, 48KB RAM)
- **电机**: 4x 直流减速电机，独立编码器反馈
- **IMU**: MPU6050 (DMP 姿态融合)
- **通信**: UART2 (115200), 自定义协议
- **电源**: ADC2 电压监测

## 通信协议

**帧格式**: `[0xFC][FuncCode][Data...][Checksum][0xDF]`

| 功能码 | 说明 |
|--------|------|
| 0x01   | 电池电压 |
| 0x02   | 编码器位置 |
| 0x03   | 陀螺仪数据 |
| 0x04   | 加速度计数据 |
| 0x05   | 欧拉角 (DMP) |
| 0x06   | 电机速度控制 |
| 0x07   | PID 参数设置 |
| 0x08   | 舵机控制 |

**校验和**: 校验和之前所有字节的 XOR

## 开发文档

- [固件架构说明](.claude/CLAUDE.md)
- [固件构建指南](firmware/BUILD.md)
- [固件 README](firmware/README.md)

## 许可证

本项目仅供学习和研究使用。
