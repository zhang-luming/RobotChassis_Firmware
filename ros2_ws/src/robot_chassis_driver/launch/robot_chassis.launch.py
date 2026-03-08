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

    return LaunchDescription([
        serial_port_arg,
        robot_chassis_node,
    ])
