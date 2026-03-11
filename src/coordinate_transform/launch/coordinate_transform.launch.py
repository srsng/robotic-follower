#!/usr/bin/env python3
"""
坐标变换模块启动文件

启动 TF 调试节点，提供坐标变换功能
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    生成启动描述

    Returns:
        LaunchDescription: 启动配置
    """
    # TF 调试节点
    tf_debug_node = Node(
        package='coordinate_transform',
        executable='tf_debug_node',
        name='tf_debug_node',
        output='screen',
        parameters=[
            {'debug_frequency': 1.0},
        ],
    )

    return LaunchDescription([
        tf_debug_node,
    ])
