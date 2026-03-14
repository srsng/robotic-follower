#!/usr/bin/env python3
"""视觉跟随系统启动文件。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成视觉跟随系统的 Launch 描述。"""

    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    config_file = LaunchConfiguration('config_file', default='')

    # 视觉跟随节点
    visual_follow_node = Node(
        package='visual_follow',
        executable='follow_node',
        name='visual_follow',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'config_file': config_file,
        }]
    )

    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value='',
            description='Path to visual follow config file'
        ),

        # 启动节点
        visual_follow_node,
    ])
