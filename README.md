# STM32 RobotChassis Firmware

基于STM32F103xE的机器人底盘固件项目，支持四轮独立驱动、IMU姿态测量、舵机控制和串口通信。

## 项目特性

### 硬件功能
- ✅ **四轮独立电机控制** - TIM8 PWM输出，25kHz频率
- ✅ **四路编码器接口** - TIM2/3/4/5，4倍频计数
- ✅ **MPU6050 IMU** - 集成DMP数字运动处理器
- ✅ **舵机控制** - TIM1 PWM输出，50Hz频率
- ✅ **串口通信** - USART1调试 + USART2通信协议
- ✅ **电池电压监测** - ADC2连续采样
- ✅ **LED状态指示** - 可视化系统状态

### 软件架构
- 🏗️ **模块化设计** - 每个功能独立模块，清晰接口
- ⏱️ **实时任务调度** - TIM6基准，10ms/40ms/200ms多周期任务
- 🔄 **PID闭环控制** - 电机速度精确控制
- 📡 **自定义通信协议** - 帧格式 + XOR校验
- 📚 **完整文档体系** - 每个模块都有详细说明

---

## 快速开始

### 环境要求

**必需工具**：
- CMake 3.22+
- Ninja
- arm-none-eabi-gcc (ARM GCC工具链)
- OpenOCD (烧录工具，可选)

**推荐IDE**：
- CLion (已配置CMake)
- VSCode + CMake Tools插件
- STM32CubeIDE

### 构建项目

```bash
# 配置并编译（Debug模式）
cmake --preset Debug
cmake --build build/Debug

# 配置并编译（Release模式）
cmake --preset Release
cmake --build build/Release
```

### 烧录到板子

**使用OpenOCD**：
```bash
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
  -c "program build/Debug/RobotChassis_Firmware.elf verify reset exit"
```

**使用ST-Link Utility**：
打开软件 → 打开`.elf`文件 → Target → Program & Verify

---

## 目录结构

```
RobotChassis_Firmware/
├── Core/                    # STM32CubeMX生成的核心代码
│   ├── Inc/                 # 头文件（外设配置）
│   └── Src/                 # 源文件（中断、初始化）
├── USER/                    # 用户应用代码（主要功能模块）
│   ├── System/              # 系统基础设施（retarget）
│   ├── Communication/       # 串口通信协议
│   ├── EXIT/                # 外部中断处理
│   ├── IMU/                 # MPU6050驱动和DMP
│   ├── MotorControl/        # 电机PID控制
│   ├── PowerManagement/     # 电源管理
│   ├── ServoControl/        # 舵机控制
│   ├── LedControl/          # LED控制
│   └── user_config.*        # 全局配置
├── Drivers/                 # STM32 HAL驱动库
├── Documentation/           # 项目文档
├── cmake/                   # CMake构建配置
├── CMakeLists.txt           # 主构建文件
├── CLAUDE.md                # Claude Code开发指南
└── README.md                # 本文件
```

---

## 硬件配置

### MCU规格
- **型号**：STM32F103xE
- **主频**：72MHz
- **Flash**：512KB
- **RAM**：64KB
- **封装**：LQFP64

### 外设使用
| 外设 | 用途 | 引脚 |
|-----|------|------|
| TIM1 | 舵机PWM | PB14, PB15 |
| TIM2-5 | 编码器 | PA15,PB3 / PB4,PB5 / PB6,PB7 / PA0,PA1 |
| TIM6 | 系统时基 | - |
| TIM8 | 电机PWM | PC6, PC7, PC8, PC9 |
| USART1 | 调试串口 | PA9(TX), PA10(RX) |
| USART2 | 通信串口 | PA2(TX), PA3(RX) |
| ADC2 | 电池检测 | PB0 |

详细的引脚映射和外设配置请参考：
- 📖 [Core外设配置详解](Documentation/Core外设配置详解.md)
- 📋 [Core外设配置速查表](Documentation/Core外设配置速查表.md)

---

## 通信协议

### 帧格式
```
[帧头][功能码][数据10字节][校验][帧尾]
 0xFC  1字节   10字节    1字节  0xDF
```

### 功能码定义
| 功能码 | 名称 | 方向 | 说明 |
|-------|------|------|------|
| 0x01 | 电池电压 | 上报 | 电池电压(mV) |
| 0x02 | 编码器 | 上报 | 四路编码器计数值 |
| 0x03 | 陀螺仪 | 上报 | 三轴角速度 |
| 0x04 | 加速度 | 上报 | 三轴加速度 |
| 0x05 | 欧拉角 | 上报 | 俯仰/横滚/航向角 |
| 0x06 | 电机速度 | 下发 | 设置目标速度 |
| 0x07 | PID参数 | 下发 | 配置PID参数 |
| 0x08 | 舵机控制 | 下发 | 设置舵机角度 |

### 示例

**设置电机A速度**：
```python
# 功能码0x06，电机A(0)，目标速度100
data = [0x06, 0x00, 100, 0, 0, 0, 0, 0, 0, 0, 0]
checksum = xor_checksum(data)
frame = [0xFC] + data + [checksum, 0xDF]
```

详细协议说明请参考 `USER/Communication/comm_protocol.h`

---

## 开发指南

### 添加新功能模块

1. **创建目录**：`USER/NewModule/`
2. **编写代码**：`new_module.c/h`
3. **更新CMakeLists.txt**：
   ```cmake
   target_sources(${CMAKE_PROJECT_NAME} PRIVATE
       USER/NewModule/new_module.c
   )
   target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
       USER/NewModule
   )
   ```
4. **更新文档**：修改 `USER/README.md`

### 修改外设配置

1. 打开 `RobotChassis_Firmware.ioc`（STM32CubeMX）
2. 修改配置
3. 重新生成代码
4. 用户代码（`/* USER CODE */`段）会保留

⚠️ **注意**：不要直接修改Core目录中USER CODE段之外的代码

### 使用printf调试

```c
#include "System/retarget.h"

// 在main()中初始化
RetargetInit(&huart1);

// 使用printf
printf("Debug: motor speed = %d\n", speed);
```

调试完成后移除 `#define Debug` 宏以禁用printf。

---

## 常见问题

### 编译错误
**问题**：找不到头文件
**解决**：
```bash
rm -rf build/Debug
cmake --preset Debug
cmake --build build/Debug
```

### 烧录失败
**问题**：OpenOCD连接失败
**解决**：
1. 检查ST-Link连接
2. 确认设备：`openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "init; exit"`
3. 尝试重置板子

### MPU6050初始化失败
**问题**：DMP初始化返回错误码4
**解决**：
1. 检查I2C引脚连接（PB12-SCL, PB13-SDA）
2. 确认AD0引脚接地（I2C地址0x68）
3. 测量电源电压是否稳定

### 电机不转动
**问题**：设置速度后电机无反应
**检查**：
1. TIM8 PWM是否启动：`HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1)`
2. 方向GPIO配置是否正确
3. PWM值是否在范围内（0-1439）

---

## 文档索引

### 项目文档
- 📖 [CLAUDE.md](CLAUDE.md) - Claude Code开发指南
- 📂 [USER/README.md](USER/README.md) - 用户模块架构说明

### 硬件文档
- 📖 [Core外设配置详解](Documentation/Core外设配置详解.md) - 完整的外设配置说明
- 📋 [Core外设配置速查表](Documentation/Core外设配置速查表.md) - 快速参考手册

### 模块文档
- 📖 [MPU6050 IMU架构详解](Documentation/MPU6050/MPU6050_IMU架构详解.md) - IMU工作原理
- 📖 [System/README.md](USER/System/README.md) - 系统基础设施说明

---

## 工具和资源

### Python工具
- **pid_command_generator.py** - PID配置命令生成器
  ```bash
  python pid_command_generator.py --motor 0 --kp 1.5 --ki 0.1 --kd 0.05
  ```

### 在线资源
- [STM32F103参考手册](https://www.st.com/resource/en/reference_manual/cd00171190.pdf)
- [HAL库用户指南](https://www.st.com/resource/en/user_manual/um1850-description-of-stm32f1xx-hal-drivers-stmicroelectronics.pdf)
- [MPU6050寄存器手册](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)

---

## 许可证

本项目仅供学习和研究使用。

---

## 贡献

欢迎提交Issue和Pull Request！

### 提交规范
- feat: 新功能
- fix: 修复bug
- docs: 文档更新
- refactor: 代码重构
- test: 测试相关
- chore: 构建/工具更新

### Commit消息格式
```
<type>(<scope>): <subject>

<body>

<footer>
```

示例：
```
feat(MotorControl): 添加速度平滑控制算法

使用滑动平均滤波减少电机抖动，
提高运动平稳性。

Closes #123
```

---

## 作者

**luming.zhang** <ming-zhanglu@outlook.com>

---

## 更新日志

### v1.0.0 (2026-01-17)
- ✨ 初始版本发布
- ✅ 四轮电机控制
- ✅ MPU6050 IMU集成
- ✅ 串口通信协议
- ✅ 完整项目文档

---

**Happy Coding! 🚀**
