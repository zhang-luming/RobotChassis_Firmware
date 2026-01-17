# USER目录结构说明

## 目录概述

USER目录包含用户应用代码，按功能模块组织，每个模块都有独立的子目录。

## 目录结构

```
USER/
│
├── System/                    # 【系统级基础设施】
│   ├── retarget.c             # C标准库I/O重定向实现
│   ├── retarget.h             # C标准库I/O重定向接口
│   └── README.md              # System目录说明
│
├── Communication/             # 【通信模块】
│   ├── comm_protocol.c        # 通信协议实现
│   └── comm_protocol.h        # 通信协议定义
│   功能：UART通信协议、数据打包解包、指令处理
│
├── EXIT/                      # 【外部中断处理】
│   ├── EXIT.c                 # 外部中断实现
│   └── EXIT.h                 # 外部中断接口
│   功能：编码器信号中断处理
│
├── IMU/                       # 【惯性测量单元】
│   ├── imu_data.c             # IMU数据封装层
│   ├── imu_data.h             # IMU数据接口
│   └── MPU6050/               # MPU6050驱动和DMP库
│       ├── mpu6050.c          # MPU6050基础驱动
│       ├── mpu6050.h          # 寄存器定义
│       ├── mpuiic.c           # I2C通信实现
│       ├── mpuiic.h           # I2C接口
│       └── eMPL/              # InvenSense官方DMP库
│           ├── inv_mpu.c      # DMP核心功能
│           ├── inv_mpu.h
│           ├── inv_mpu_dmp_motion_driver.c
│           ├── inv_mpu_dmp_motion_driver.h
│           ├── dmpKey.h       # DMP固件密钥
│           └── dmpmap.h       # DMP内存映射
│   功能：姿态角测量、加速度测量、角速度测量
│
├── LedControl/                # 【LED控制】
│   ├── led_control.c          # LED控制实现
│   └── led_control.h          # LED控制接口
│   功能：LED状态指示、呼吸灯效果
│
├── MotorControl/              # 【电机控制】
│   ├── motor_control.c        # 电机控制实现
│   └── motor_control.h        # 电机控制接口
│   功能：电机速度控制、PWM输出、方向控制
│
├── PowerManagement/           # 【电源管理】
│   ├── power_management.c     # 电源管理实现
│   └── power_management.h     # 电源管理接口
│   功能：电池电压检测、电量显示、低电量保护
│
├── ServoControl/              # 【舵机控制】
│   ├── servo_control.c        # 舵机控制实现
│   └── servo_control.h        # 舵机控制接口
│   功能：舵机角度控制、PWM生成
│
├── user_config.c              # 【用户配置】
└── user_config.h              # 全局配置和参数定义
```

## 模块依赖关系

```
                    ┌─────────────────┐
                    │   main.c        │
                    │   (主程序)       │
                    └────────┬────────┘
                             │
         ┌───────────────────┼───────────────────┐
         │                   │                   │
         ↓                   ↓                   ↓
  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
  │ Communication│    │ MotorControl│    │   IMU       │
  └─────────────┘    └─────────────┘    └─────────────┘
         │                   │                   │
         └───────────────────┼───────────────────┘
                             ↓
                    ┌─────────────┐
                    │   System    │
                    │ (retarget)  │
                    └─────────────┘
```

## 模块功能详解

### 1. System（系统基础设施）
**职责**：提供系统级基础服务
- **retarget**：C标准库I/O重定向，支持printf()调试输出

**特点**：
- 独立于业务逻辑
- 可复用于其他项目
- 最少的依赖关系

### 2. Communication（通信模块）
**职责**：与上位机或其他设备通信
- 实现自定义通信协议
- 数据打包和校验
- 指令解析和执行

**使用场景**：
- 接收上位机控制指令
- 发送传感器数据
- 参数配置和调试

### 3. EXIT（外部中断）
**职责**：处理外部中断事件
- 编码器信号捕获
- 实时事件响应

**使用场景**：
- 测速编码器
- 限位开关
- 急停按钮

### 4. IMU（惯性测量单元）
**职责**：提供姿态和运动信息
- **imu_data**：统一的数据接口
- **MPU6050**：硬件驱动层
  - **eMPL**：DMP算法库

**输出数据**：
- 欧拉角（pitch、roll、yaw）
- 加速度（ax、ay、az）
- 角速度（gx、gy、gz）

**架构**：三层设计
```
应用层：imu_data.c         统一接口
   ↓
算法层：eMPL/               DMP姿态解算
   ↓
驱动层：mpu6050.c/h         硬件操作
```

### 5. LedControl（LED控制）
**职责**：LED指示灯控制
- 状态指示（电源、通信、故障）
- 视觉反馈
- 调试辅助

### 6. MotorControl（电机控制）
**职责**：电机驱动和控制
- PWM生成
- 速度调节
- 方向控制

**典型应用**：
- 底盘驱动电机
- 云台电机
- 机械臂电机

### 7. PowerManagement（电源管理）
**职责**：电源状态监控
- 电池电压检测
- 电量估算
- 低电量警告

### 8. ServoControl（舵机控制）
**职责**：舵机驱动
- PWM信号生成
- 角度控制
- 速度控制

## 代码组织原则

### 1. 模块化
- 每个功能模块独立目录
- 清晰的接口定义
- 最小化模块间耦合

### 2. 分层设计
```
应用层：业务逻辑和功能组合
   ↓
驱动层：硬件抽象和设备驱动
   ↓
硬件层：STM32 HAL库
```

### 3. 命名规范
- 目录：小写，下划线分隔（如motor_control）
- 文件：小写，下划线分隔（如motor_control.c）
- 函数：模块前缀 + 动作（如Motor_SetSpeed）

### 4. 头文件组织
```c
/* 头文件保护 */
#ifndef _MODULE_NAME_H
#define _MODULE_NAME_H

/* 头文件包含 */
#include "main.h"

/* 宏定义 */
#define ...

/* 类型定义 */
typedef ...

/* 函数声明 */
...

/* 条件编译结束 */
#endif
```

## 添加新模块指南

当需要添加新功能模块时，请按以下步骤操作：

### 1. 创建目录结构
```bash
mkdir USER/NewModule
```

### 2. 创建头文件（new_module.h）
```c
#ifndef __NEW_MODULE_H
#define __NEW_MODULE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 功能说明 */
/* 宏定义 */
/* 类型定义 */
/* 函数声明 */

#ifdef __cplusplus
}
#endif

#endif /* __NEW_MODULE_H */
```

### 3. 创建源文件（new_module.c）
```c
/**
 * @file    new_module.c
 * @brief   新模块实现
 */

#include "new_module.h"

/* 私有变量 */
/* 私有函数 */
/* 公共函数实现 */
```

### 4. 在main.c中引用
```c
#include "new_module.h"

void main(void) {
    /* 初始化 */
    NewModule_Init();

    while(1) {
        /* 主循环 */
        NewModule_Process();
    }
}
```

### 5. 更新文档
- 更新本README.md
- 添加模块功能说明
- 标注依赖关系

## 依赖关系说明

### 硬件依赖
所有模块依赖STM32 HAL库提供的硬件驱动。

### 模块间依赖
- **System**：被所有模块依赖（retarget用于调试）
- **IMU**：依赖MPU6050驱动和eMPL库
- **Communication**：可能依赖其他模块获取数据
- **其他模块**：相对独立

### 全局配置
- **user_config.h**：所有模块共享的配置参数
- 避免模块间直接访问全局变量

## 编译配置

确保以下路径已添加到编译器包含路径：
```
-IUSER/Communication
-IUSER/EXIT
-IUSER/IMU
-IUSER/IMU/MPU6050
-IUSER/IMU/MPU6050/eMPL
-IUSER/LedControl
-IUSER/MotorControl
-IUSER/PowerManagement
-IUSER/ServoControl
-IUSER/System
```

## 维护指南

### 代码审查要点
1. 接口设计是否合理
2. 错误处理是否完善
3. 注释是否清晰
4. 是否符合编码规范

### 测试建议
1. 单元测试：独立测试每个模块
2. 集成测试：测试模块间交互
3. 硬件测试：在实际硬件上验证

### 版本控制
- 保持模块接口稳定
- 重大变更更新文档
- 记录已知问题

## 更新日志

### 2026-01-17
- 创建System目录
- 将retarget文件移至System目录
- 添加详细的代码注释
- 创建本README文档

## 联系方式

项目维护者：RobotChassis Team
文档更新：2026-01-17
