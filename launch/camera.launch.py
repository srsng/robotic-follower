#!/usr/bin/env python3
"""RealSense 相机启动文件。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成 RealSense 相机的 Launch 描述。"""

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_name',
            default_value='camera',
            description='Camera name'
        ),
        DeclareLaunchArgument(
            'depth_width',
            default_value='640',
            description='Depth image width'
        ),
        DeclareLaunchArgument(
            'depth_height',
            default_value='480',
            description='Depth image height'
        ),
        DeclareLaunchArgument(
            'depth_fps',
            default_value='30',
            description='Depth FPS'
        ),

        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera',
            output='screen',
            parameters=[{
                'align_depth.enable': True,
                'depth_module.depth_profile': '640x480x30',
                'rgb_camera.color_profile': '640x480x30',
                'enable_depth': True,
                'enable_color': True,
                'enable_infra1': False,
                'enable_infra2': False,
            }]
        ),
    ])
