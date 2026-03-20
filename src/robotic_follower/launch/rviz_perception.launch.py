#!/usr/bin/env python3
"""RViz 感知可视化组合启动文件。

该文件是通用的 RViz 可视化启动模块，可被其他 launch 文件 include。
同时启动感知数据转发和（可选的）机械臂模型显示。

使用方式：
    # 作为独立 launch 文件
    ros2 launch robotic_follower rviz_perception.launch.py

    # 在其他 launch 文件中 include
    from launch_ros.actions import Node
    from launch_ros.substitutions import FindPackageShare
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('robotic_follower'), '/launch/rviz_perception.launch.py'
        ]),
        launch_arguments={
            'use_robot_model': 'true',  # 或 'false'
            'rviz_config': 'default'
        }.items()
    )

启动参数：
    - use_robot_model (bool, 默认 true): 是否显示机械臂模型
    - rviz_config (str, 默认 "default"): rviz 配置文件名（不含 .rviz 后缀）
    - robot_description_pkg (str, 默认 "dummy-ros2_description"): 机械臂 URDF 包名
    - robot_xacro_file (str, 默认 ""): 机械臂 xacro 文件路径（为空则使用包内默认）
"""

import os
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import LaunchDescription


def get_robot_description(context) -> str | None:
    """获取机械臂 URDF 描述。"""
    from ament_index_python.packages import get_package_share_directory

    robot_desc_pkg = context.perform_substitution(LaunchConfiguration("robot_description_pkg"))
    robot_xacro_file = context.perform_substitution(LaunchConfiguration("robot_xacro_file"))

    if robot_xacro_file and os.path.exists(os.path.expanduser(robot_xacro_file)):
        xacro_file = os.path.expanduser(robot_xacro_file)
    else:
        try:
            dummy_desc_share = get_package_share_directory(robot_desc_pkg)
            xacro_file = os.path.join(dummy_desc_share, "urdf", "dummy-ros2.xacro")
        except Exception:
            return None

    import xacro
    robot_description_config = xacro.process_file(xacro_file)
    return robot_description_config.toxml()


def get_rviz_config_path(rviz_config_name: str) -> str:
    """获取 rviz 配置文件路径。"""
    from ament_index_python.packages import get_package_share_directory

    try:
        rviz_share = get_package_share_directory("robotic_follower")
        config_path = os.path.join(rviz_share, "rviz", f"{rviz_config_name}.rviz")
        if os.path.exists(config_path):
            return config_path
    except Exception:
        pass

    # fallback
    return os.path.expanduser(f"~/ros2_ws/install/robotic_follower/share/robotic_follower/rviz/{rviz_config_name}.rviz")


def generate_launch_description():
    """生成 Launch 描述。"""
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_robot_model",
            default_value="true",
            description="是否显示机械臂模型",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value="default",
            description="RViz 配置文件名（不含 .rviz 后缀）",
        ),
        DeclareLaunchArgument(
            "robot_description_pkg",
            default_value="dummy-ros2_description",
            description="机械臂 URDF 包名",
        ),
        DeclareLaunchArgument(
            "robot_xacro_file",
            default_value="",
            description="机械臂 xacro 文件路径（为空则使用包内默认）",
        ),
        OpaqueFunction(function=launch_rviz_nodes),
    ])


def launch_rviz_nodes(context):
    """启动 RViz 节点。"""
    use_robot_model = context.perform_substitution(
        LaunchConfiguration("use_robot_model")
    ).lower() in ("true", "1", "yes")

    rviz_config_name = context.perform_substitution(
        LaunchConfiguration("rviz_config")
    )
    rviz_config = get_rviz_config_path(rviz_config_name)

    nodes = []

    # 机械臂模型节点
    if use_robot_model:
        robot_urdf = get_robot_description(context)
        if robot_urdf:
            nodes.extend([
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    name="robot_state_publisher",
                    parameters=[{"robot_description": robot_urdf}],
                ),
                Node(
                    package="joint_state_publisher",
                    executable="joint_state_publisher",
                    name="joint_state_publisher",
                ),
            ])
        else:
            print("警告: 无法加载机械臂 URDF，跳过机械臂模型显示")

    # 感知数据转发节点
    nodes.append(
        Node(
            package="robotic_follower",
            executable="rviz_visualizer",
            name="rviz_visualizer",
            output="screen",
        )
    )

    # RViz2 窗口
    nodes.append(
        ExecuteProcess(
            cmd=["rviz2", "-d", rviz_config],
            output="screen",
            shell=False,
        )
    )

    return nodes
