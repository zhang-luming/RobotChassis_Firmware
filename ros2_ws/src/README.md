# ROS2 包源码目录

此目录用于存放 RobotChassis 的 ROS2 包。

## 预期包结构

```
src/
├── robot_chassis_bringup/    # 启动配置和 launch 文件
├── robot_chassis_driver/     # 串口通信驱动节点
├── robot_chassis_controller/ # 运动控制节点
├── robot_chassis_description/# URDF 模型描述
└── robot_chassis_nav/        # 导航相关配置
```

## 创建新包

```bash
cd ros2_ws
ros2 pkg create --build-type ament_python <package_name>
# 或
ros2 pkg create --build-type ament_cmake <package_name>
```

## 编译工作空间

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 通信接口

待实现的消息/服务接口：
- `MotorSpeed.msg` - 电机速度消息
- `IMUData.msg` - IMU 数据消息
- `SetPID.srv` - PID 参数服务
