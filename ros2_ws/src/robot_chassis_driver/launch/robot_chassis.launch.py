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
        'base.yaml'
    )

    ekf_config_file = os.path.join(
        ament_index_python.packages.get_package_share_directory('robot_chassis_driver'),
        'config',
        'ekf.yaml'
    )

    # RobotChassis驱动节点
    robot_chassis_node = Node(
        package='robot_chassis_driver',
        executable='robot_chassis_node',
        name='robot_chassis_node',
        output='screen',
        parameters=[config_file, {'serial_port': LaunchConfiguration('serial_port')}]
    )

    # IMU滤波节点 (Madgwick滤波器)
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('imu/data_raw', '/imu/data_raw'),  # 节点订阅 imu/data_raw -> 实际话题 /imu/data_raw
            ('imu/data', '/imu/data')           # 节点发布 imu/data -> 实际话题 /imu/data
        ]
    )

    # EKF滤波节点 (robot_localization)
    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file],
        remappings=[
            ('/wheel_odom', '/wheel_odom'),  # 轮速里程计
            ('/imu/data', '/imu/data'),      # IMU数据
            ('/odometry/filtered', '/odom')  # 融合后里程计
        ]
    )

    # 静态TF发布器 (odom -> wheel_odom)
    # 连接EKF融合里程计坐标系到轮速里程计坐标系
    # 通常为identity变换，用于在同一TF树中对比两种里程计
    static_tf_odom_to_wheel_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_wheel_odom',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',           # 平移 (米)
            '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',  # 旋转 (四元数)
            '--frame-id', 'odom',                               # 父坐标系 (EKF融合里程计)
            '--child-frame-id', 'wheel_odom'                    # 子坐标系 (轮速里程计)
        ]
    )

    # 静态TF发布器 (base_link -> imu_link)
    # 注意：需要根据实际IMU安装位置调整平移和旋转参数
    static_tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_link',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.1',           # 平移 (米)
            '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',  # 旋转 (四元数)
            '--frame-id', 'base_link',                          # 父坐标系
            '--child-frame-id', 'imu_link'                      # 子坐标系
        ]
    )

    return LaunchDescription([
        serial_port_arg,
        robot_chassis_node,
        imu_filter_node,
        ekf_filter_node,
        static_tf_odom_to_wheel_odom,
        static_tf_base_to_imu,
    ])
