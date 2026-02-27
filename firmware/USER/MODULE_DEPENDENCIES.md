# RobotChassis 固件模块依赖分析

## 模块列表

| 模块 | 头文件 | 初始化函数 | 更新函数 | 说明 |
|------|--------|-----------|---------|------|
| System/Timer | `timer.h` | `Time_Init()` | - | 微秒时间戳 (TIM7) |
| System/Retarget | `retarget.h` | `RetargetInit()` | - | printf重定向 (USART1) |
| System/Debug | `debug.h` | - | - | 调试日志宏 |
| LedControl | `led_control.h` | `LED_Init()` | `LED_Update()` | LED状态指示 |
| MotorControl | `motor_control.h` | `Motor_Init()` | `Motor_UpdateControl()` | 4路电机PID控制 |
| Communication | `comm_protocol.h` | `Comm_Init()` | `Comm_Update()` | 串口协议 (UART2) |
| IMU | `imu.h` | `IMU_Init()` | `IMU_IRQHandler()` | MPU6050 DMP |
| PowerManagement | `power_management.h` | `Power_Init()` | `Power_Update()` | 电池电压监测 |
| ServoControl | `servo_control.h` | `Servo_Init()` | `Servo_Update()` | PWM舵机控制 |
| TimeSync | `ptp_sync.h` | `PTP_Init()` | - | PTP时间同步 |

## 初始化顺序

```
1. HAL_Init() / SystemClock_Config()  [HAL库]
2. MX_xxx_Init()                       [外设初始化]
3. Time_Init()                         [时间戳基础]
4. RetargetInit()                      [调试输出]
5. LED_Init()                          [状态指示]
6. Motor_Init()                        [编码器定时器]
7. Power_Init()                        [ADC]
8. Servo_Init()                        [PWM定时器]
9. PTP_Init()                          [时间同步]
10. Comm_Init()                        [串口通信]
11. IMU_Init() + IMU_SetEnabled(1)     [IMU中断]
```

## 中断依赖

### IMU中断 (PC12, EXTI15_10)
```
IMU_IRQHandler()
  ├── IMU_UpdateAndPublish()
  │   └── Comm_SendDataFrame() × 3    [发送欧拉角/陀螺仪/加速度]
  └── Motor_ReadAndReport()
      └── Comm_SendDataFrame()         [发送编码器数据]
```

**关键依赖：** IMU中断依赖 `Comm_Init()` 和 `Motor_Init()`

### TIM6中断 (10ms系统滴答)
```
TIM6_IRQHandler()
  └── 设置超时标志
```

**主循环检查超时后调用：**
- `Comm_Update()` - 处理接收的命令
- `Motor_UpdateControl()` - PID控制
- `LED_Update()` - LED状态机
- `Servo_Update()` - 舵机控制
- `Power_Update()` - 电压监测 (每200ms)

## 潜在问题点

### 1. IMU中断风暴
- **现象：** 系统卡死，LED停止闪烁
- **原因：** MPU6050 INT引脚持续触发，中断未正确清除
- **排查：** 在 `IMU_IRQHandler()` 添加计数器和调试输出

### 2. I2C通信阻塞
- **位置：** `IMU_UpdateAndPublish()` → `mpu_dmp_get_data()`
- **影响：** 中断执行时间过长
- **解决：** 确保I2C超时配置合理

### 3. UART发送阻塞
- **位置：** `Comm_SendDataFrame()` 在中断中调用
- **影响：** 中断执行时间过长
- **解决：** 使用非阻塞发送或缓冲队列

## 调试建议

1. **分阶段初始化：** 先不启用IMU中断，测试基础功能
2. **添加中断计数：** 检测中断风暴
3. **测量中断执行时间：** 确保中断处理时间 < 1ms
