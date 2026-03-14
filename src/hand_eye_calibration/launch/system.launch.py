#!/usr/bin/env python3
"""
ROS2 机械臂视觉跟随系统 - 完整系统启动文件
用途：一键启动所有必需的节点
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """生成完整系统的 Launch 描述"""

    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 1. RealSense 相机启动
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'align_depth.enable': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',
            'depth_module.profile': '640x480x30',
            'rgb_camera.profile': '640x480x30',
        }.items()
    )

    # 2. 手眼标定节点（自动加载标定结果并发布 TF）
    calibration_node = Node(
        package='hand_eye_calibration',
        executable='calibration_node',
        name='hand_eye_calibration',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'auto_load_calibration': True,
            'calibration_file': 'results/calibration.yaml',
        }]
    )

    # 3. 感知节点
    perception_node = Node(
        package='perception',
        executable='perception_node',
        name='perception',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    # 4. 视觉跟随协调器
    visual_follow_node = Node(
        package='visual_follow',
        executable='follow_node',
        name='visual_follow',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    # 5. RViz 可视化
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('hand_eye_calibration'),
        'config',
        'system.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=None  # 可选：添加条件启动
    )

    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        # 启动节点
        realsense_launch,
        calibration_node,
        perception_node,
        visual_follow_node,
        rviz_node,  # 可选：取消注释以启动 RViz
    ])
