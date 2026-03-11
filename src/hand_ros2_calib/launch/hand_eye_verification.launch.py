#!/usr/bin/env python3
"""
手眼标定验证启动文件

启动标定结果验证节点
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    生成启动描述

    Returns:
        LaunchDescription: 启动配置
    """
    # 验证节点
    verification_node = Node(
        package='hand_ros2_calib',
        executable='verification_node',
        name='hand_eye_verification',
        output='screen',
        parameters=[
            {'calibration_file': './calib_extrinsic.xml'},
            {'visualize': True},
        ],
    )

    return LaunchDescription([
        verification_node,
    ])
