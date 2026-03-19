#!/usr/bin/env python3
"""从 .bin 文件读取点云进行测试的 Launch 文件。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 声明启动参数
    bin_file = LaunchConfiguration('bin_file')
    run_detection = LaunchConfiguration('run_detection', default='true')

    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'bin_file',
            description='Path to the .bin point cloud file'
        ),
        DeclareLaunchArgument(
            'run_detection',
            default_value='true',
            description='Run 3D detection on the point cloud'
        ),

        # Bin 发布节点
        Node(
            package='perception',
            executable='bin_publisher_node',
            name='bin_publisher',
            output='screen',
            parameters=[{
                'bin_file': bin_file,
                'run_detection': run_detection,
            }]
        ),

        # Open3D 可视化节点
        Node(
            package='perception',
            executable='open3d_visualizer_node',
            name='open3d_visualizer',
            output='screen',
        ),
    ])
