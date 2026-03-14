#!/usr/bin/env python3
"""感知系统启动文件。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成感知系统的 Launch 描述。"""

    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    config_file = LaunchConfiguration('config_file', default='')

    # 感知节点
    perception_node = Node(
        package='perception',
        executable='perception_node',
        name='perception',
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
            description='Path to perception config file'
        ),

        # 启动节点
        perception_node,
    ])
