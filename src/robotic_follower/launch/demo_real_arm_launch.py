#!/usr/bin/env python3
"""
基于 ros2_dummy_arm_810 dummy_moveit_config demo_real_arm.launch.py 修改

Simple and Reliable Launch File for Real Robot with RViz
This launch file focuses on getting RViz working correctly with the real robot
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from launch import LaunchDescription


def generate_launch_description():

    # Get the path to dummy_with_d435i.urdf.xacro from robotic_follower package
    # This ensures TF consistency with perception_real.launch.py

    robotic_follower_share = get_package_share_directory("robotic_follower")
    composite_urdf_path = os.path.join(
        robotic_follower_share, "urdf", "dummy_with_d435i.urdf.xacro"
    )

    # Use the COMPOSITE URDF (arm + camera) - ensures TF consistency with perception launch
    moveit_config = (
        MoveItConfigsBuilder("dummy-ros2", package_name="dummy_moveit_config")
        .robot_description(file_path=composite_urdf_path)
        .robot_description_semantic(file_path="config/dummy-ros2.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # moveit_config = (
    #     MoveItConfigsBuilder("dummy-ros2", package_name="dummy_moveit_config")
    #     .robot_description(file_path="config/dummy-ros2.urdf.xacro")
    #     .robot_description_semantic(file_path="config/dummy-ros2.srdf")
    #     .trajectory_execution(file_path="config/moveit_controllers.yaml")
    #     .to_moveit_configs()
    # )

    # RViz with MoveIt configuration
    rviz_config_file = os.path.join(
        get_package_share_directory("dummy_moveit_config"),
        "config",
        "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Static TF publisher
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Robot State Publisher - CRITICAL for joint state visualization
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # MoveGroup node for motion planning
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"missing_registered_joints_are_passive": True},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    return LaunchDescription(
        [
            rviz_node,
            static_tf,
            robot_state_publisher,
            move_group_node,
        ]
    )
