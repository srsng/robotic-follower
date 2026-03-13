#!/usr/bin/env python3
"""ros2_control 系统启动（仅启动硬件接口和控制器）"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_exec_pkg = FindPackageShare(package='robot_execution')

    # 声明参数
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='是否使用仿真模式（无硬件时设为true）'
    )

    # 使用 xacro 处理 URDF
    # 设置 use_sim 环境变量让xacro正确读取
    env_var = SetParameter(
        name='use_sim',
        value=LaunchConfiguration('use_sim')
    )

    # Static TF Publisher
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_link']
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {
                'robot_description': PathJoinSubstitution([
                    robot_exec_pkg, 'urdf', 'dummy-ros2.xacro'
                ])
            },
            env_var
        ]
    )

    # Controller Manager (ros2_control_node)
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='ros2_control_node',
        output='both',
        parameters=[
            {
                'robot_description': PathJoinSubstitution([
                    robot_exec_pkg, 'urdf', 'dummy-ros2.xacro'
                ])
            },
            env_var
        ]
    )

    # Joint State Broadcaster
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        output='both',
        arguments=[
            'joint_state_broadcaster',
            '-c', '/controller_manager'
        ]
    )

    # Joint Trajectory Controller
    jtc_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='dummy_arm_controller_spawner',
        output='both',
        arguments=[
            'dummy_arm_controller',
            '-c', '/controller_manager',
            '-p', PathJoinSubstitution([robot_exec_pkg, 'config', 'moveit_controllers.yaml'])
        ]
    )

    launch_nodes = [
        use_sim_arg,
        static_tf,
        robot_state_publisher,
        controller_manager_node,
        jsb_spawner,
        jtc_spawner,
    ]

    return LaunchDescription(launch_nodes)
