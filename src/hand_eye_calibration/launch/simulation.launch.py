#!/usr/bin/env python3
"""
手眼标定仿真环境启动文件
集成 Gazebo、机械臂、MoveIt2 和标定节点
"""

import os
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction
)
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成启动描述"""

    # 声明参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='是否使用仿真时间'
    )

    auto_load_calibration_arg = DeclareLaunchArgument(
        'auto_load_calibration',
        default_value='false',
        description='是否自动加载已保存的标定结果'
    )

    calibration_file_arg = DeclareLaunchArgument(
        'calibration_file',
        default_value='results/calibration.yaml',
        description='标定结果文件路径'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    auto_load_calibration = LaunchConfiguration('auto_load_calibration')
    calibration_file = LaunchConfiguration('calibration_file')

    # 1. 启动 Gazebo + 机械臂（复用 ros2_dummy_arm_810）
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('dummy_moveit_config'),
                'launch',
                'demo_gazebo_rviz.launch.py'
            ])
        ])
    )

    # 2. 启动相机节点（需要相机配置）
    # 注意：这里假设相机已经通过 realsense2_camera 发布图像
    # 如果需要仿真相机，可以在这里添加 Gazebo 插件

    # 3. 手眼标定节点（延迟启动，等待系统稳定）
    calibration_node = Node(
        package='hand_eye_calibration',
        executable='calibration_node',
        name='hand_eye_calibration',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'auto_load_calibration': auto_load_calibration,
            'calibration_file': calibration_file,
        }]
    )

    # 延迟启动标定节点（等待 Gazebo 和机械臂初始化完成）
    calibration_delayed = TimerAction(
        period=5.0,
        actions=[calibration_node]
    )

    return LaunchDescription([
        use_sim_time_arg,
        auto_load_calibration_arg,
        calibration_file_arg,
        gazebo_launch,
        calibration_delayed,
    ])
