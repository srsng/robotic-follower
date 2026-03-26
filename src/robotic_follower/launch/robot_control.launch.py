#!/usr/bin/env python3
"""机械臂控制启动文件。

启动完整的机械臂控制栈：
1. MoveIt2 (move_group)
2. robot_state_publisher
3. static_transform_publisher (world -> base_link)
4. joint_state_remapper (关节状态重映射)
5. arm_control (机械臂控制节点)
6. motion_planning (运动规划节点)

使用：
    ros2 launch robotic_follower robot_control.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # MoveIt2 配置 (复用 dummy_moveit_config)
    moveit_config = (
        MoveItConfigsBuilder("dummy-ros2", package_name="dummy_moveit_config")
        .robot_description(file_path="config/dummy-ros2.urdf.xacro")
        .robot_description_semantic(file_path="config/dummy-ros2.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # 静态 TF: world -> base_link
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # MoveGroup node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # 关节状态重映射节点
    joint_state_remapper = Node(
        package="robotic_follower",
        executable="joint_state_remapper",
        name="joint_state_remapper",
        output="screen",
        parameters=[
            {"input_topic": "/joint_states"},
            {"output_topic": "/robotic_follower/joint_states"},
        ],
    )

    # 机械臂控制节点
    arm_control_node = Node(
        package="robotic_follower",
        executable="arm_control",
        name="arm_control",
        output="screen",
        parameters=[
            {
                "group_name": "dummy_arm",
                "base_frame": "base_link",
                "end_effector_frame": "link6_1_1",
                "max_velocity": 0.3,
                "max_acceleration": 0.3,
            }
        ],
    )

    # 运动规划节点
    motion_planning_node = Node(
        package="robotic_follower",
        executable="motion_planning",
        name="motion_planning",
        output="screen",
        parameters=[
            {
                "target_object": "chair",
                "follow_distance": 0.5,
                "max_planning_time": 1.0,
                "robot_base_frame": "base_link",
                "camera_frame": "camera_depth_optical_frame",
            }
        ],
    )

    return LaunchDescription([
        static_tf,
        robot_state_publisher,
        move_group_node,
        joint_state_remapper,
        arm_control_node,
        motion_planning_node,
    ])