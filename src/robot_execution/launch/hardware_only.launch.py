#!/usr/bin/env python3
"""仅启动硬件接口和控制器（支持仿真模式）"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_execution')

    # 声明参数
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='是否使用仿真模式（无硬件时设为true）'
    )

    # 使用 xacro 生成 URDF
    robot_description_content = Command([
        'xacro',
        PathJoinSubstitution([pkg_share, 'urdf', 'dummy-ros2.xacro']),
        'use_sim:=', LaunchConfiguration('use_sim'),
    ])

    return LaunchDescription([
        # 声明参数
        use_sim_arg,

        # Static TF Publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_link']
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[{'robot_description': robot_description_content}]
        ),

        # Controller Manager (ros2_control_node)
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='ros2_control_node',
            output='both',
            parameters=[{'robot_description': robot_description_content}]
        ),

        # Joint State Broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            name='joint_state_broadcaster_spawner',
            output='both',
            arguments=[
                'joint_state_broadcaster',
                '-c', '/controller_manager'
            ]
        ),

        # Joint Trajectory Controller
        Node(
            package='controller_manager',
            executable='spawner',
            name='dummy_arm_controller_spawner',
            output='both',
            arguments=[
                'dummy_arm_controller',
            '-c', '/controller_manager',
                '-p', PathJoinSubstitution([pkg_share, 'config', 'dummy_controllers.yaml'])
            ]
        ),
    ])
