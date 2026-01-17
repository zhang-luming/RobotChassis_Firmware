# System目录说明

## 目录概述

本目录包含系统级的基础设施代码，为整个项目提供底层支持和服务。

## 包含文件

### retarget.c / retarget.h
**功能描述**：
C标准库I/O重定向模块，用于在嵌入式系统中实现printf()、scanf()等标准输入输出功能。

**主要功能**：
- 将printf()输出重定向到UART串口
- 支持scanf()从UART串口输入
- 支持getchar()、putchar()等标准I/O函数
- 自动禁用stdout缓冲，实现实时输出

**使用方法**：
```c
// 在main()函数中初始化
MX_USART1_UART_Init();  // 先初始化UART
RetargetInit(&huart1);  // 再初始化重定向

// 之后可以直接使用标准I/O函数
printf("System started!\n");
int value = scanf("%d", &value);
```

**技术细节**：
- 通过重写newlib C库的系统调用接口实现（_write、_read等）
- 使用HAL_UART函数进行底层通信
- 不依赖半主机模式（SEMIHOSTING），可独立运行
- 阻塞式I/O，适合调试使用

**注意事项**：
- 必须在使用printf()前调用RetargetInit()
- printf()会阻塞CPU，不适合高频调用
- 建议生产环境减少使用或改用环形缓冲区

## 目录结构

```
USER/
├── System/           # 系统级基础设施
│   ├── retarget.c    # C标准库I/O重定向实现
│   ├── retarget.h    # C标准库I/O重定向接口
│   └── README.md     # 本说明文件
│
├── Communication/    # 通信模块
│   └── comm_protocol.*
│
├── EXIT/             # 外部中断处理
│   └── EXIT.*
│
├── IMU/              # 惯性测量单元
│   ├── imu_data.*    # IMU数据封装
│   └── MPU6050/      # MPU6050驱动和DMP
│
├── LedControl/       # LED控制
│   └── led_control.*
│
├── MotorControl/     # 电机控制
│   └── motor_control.*
│
├── PowerManagement/  # 电源管理
│   └── power_management.*
│
├── ServoControl/     # 舵机控制
│   └── servo_control.*
│
├── user_config.c     # 用户配置
└── user_config.h
```

## 代码规范

System目录中的代码应遵循以下规范：

1. **通用性**：提供的功能应具有通用性，不依赖特定业务逻辑
2. **独立性**：尽量减少对其他模块的依赖
3. **可移植性**：代码应易于移植到其他项目
4. **文档化**：详细的注释和文档说明

## 添加新模块

如果要添加新的系统级模块，请遵循以下步骤：

1. 确认该模块属于系统级基础设施（而非业务模块）
2. 在本目录创建相应的.c和.h文件
3. 添加详细的文件头注释和函数注释
4. 更新本README.md文件
5. 确保代码符合项目编码规范

## 联系方式

如有问题或建议，请联系项目维护者。
