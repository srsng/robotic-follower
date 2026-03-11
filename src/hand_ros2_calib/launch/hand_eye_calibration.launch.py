#!/usr/bin/env python3
"""
手眼标定启动文件

启动相机、机械臂和标定节点
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    生成启动描述

    Returns:
        LaunchDescription: 启动配置
    """
    # RealSense 相机启动（如果配置了）
    # 假设 realsense2_camera 包已安装

    # 手眼标定节点
    calibration_node = Node(
        package='hand_ros2_calib',
        executable='calibration_node',
        name='hand_eye_calibration',
        output='screen',
        parameters=[
            # 标定板配置
            {'pattern_type': 'circles_asymmetric'},
            {'board_cols': 4},
            {'board_rows': 5},
            {'square_size_mm': 20.0},

            # 标定数量配置
            {'intrinsic_num_images': 20},
            {'extrinsic_num_poses': 15},

            # 手眼标定模式
            {'hand_eye_mode': 'EIH'},  # Eye-in-Hand
        ],
    )

    return LaunchDescription([
        calibration_node,
    ])
