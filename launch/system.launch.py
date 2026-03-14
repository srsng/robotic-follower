#!/usr/bin/env python3
"""系统级启动文件 - 启动完整的视觉跟随系统。"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成完整系统的 Launch 描述。"""

    # 启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    start_camera = LaunchConfiguration('start_camera', default='true')
    start_calibration = LaunchConfiguration('start_calibration', default='false')

    # 相机 Launch
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros2_ws'),
                'launch',
                'camera.launch.py'
            ])
        ]),
        condition=lambda context: context.perform_substitution(start_camera) == 'true'
    )

    # 机械臂 MoveIt Launch
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('dummy_moveit_config'),
                'launch',
                'demo_real_arm.launch.py'
            ])
        ])
    )

    # 感知系统 Launch
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('perception'),
                'launch',
                'perception.launch.py'
            ])
        ])
    )

    # 视觉跟随 Launch
    follow_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('visual_follow'),
                'launch',
                'follow.launch.py'
            ])
        ])
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('start_calibration', default_value='false'),

        camera_launch,
        moveit_launch,
        perception_launch,
        follow_launch,
    ])
