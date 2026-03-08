#!/usr/bin/env python3

"""
RobotChassis底盘驱动启动文件
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
import ament_index_python.packages

def generate_launch_description():
    # 声明launch参数
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='串口设备路径'
    )

    config_file = os.path.join(
        ament_index_python.packages.get_package_share_directory('robot_chassis_driver'),
        'config',
        'robot_chassis.yaml'
    )

    # RobotChassis驱动节点
    robot_chassis_node = Node(
        package='robot_chassis_driver',
        executable='robot_chassis_node',
        name='robot_chassis_node',
        output='screen',
        parameters=[config_file],
        arguments=[['serial_port:=', LaunchConfiguration('serial_port')]]
    )

    # IMU滤波节点 (Madgwick滤波器)
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('/imu/data_raw', '/imu/data_raw'),  # 输入：订阅原始IMU数据
            ('/imu/data', '/imu/data')           # 输出：发布滤波后的IMU数据
        ]
    )

    return LaunchDescription([
        serial_port_arg,
        robot_chassis_node,
        imu_filter_node,
    ])
