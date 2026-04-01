#!/usr/bin/env python3
"""RViz 感知可视化组合启动文件。

该文件是通用的 RViz 可视化启动模块，可被其他 launch 文件 include。
启动感知数据转发和 RViz2 窗口。

使用方式：
    # 作为独立 launch 文件
    ros2 launch robotic_follower rviz_perception.launch.py

    # 在其他 launch 文件中 include
    ```
    from launch_ros.actions import Node
    from launch_ros.substitutions import FindPackageShare
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('robotic_follower'), '/launch/rviz_perception.launch.py'
        ]),
        launch_arguments={
            'rviz_config': 'default'
        }.items()
    )
    ```

启动参数：
    - rviz_config (str, 默认 "default"): rviz 配置文件名（不含 .rviz 后缀）
"""

import os

from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription


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
    return os.path.expanduser(
        f"~/ros2_ws/install/robotic_follower/share/robotic_follower/rviz/{rviz_config_name}.rviz"
    )


def generate_launch_description():
    """生成 Launch 描述。"""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value="default",
                description="RViz 配置文件名（不含 .rviz 后缀）",
            ),
            OpaqueFunction(function=launch_rviz_nodes),
        ]
    )


def launch_rviz_nodes(context):
    """启动 RViz 节点。"""
    rviz_config_name = context.perform_substitution(LaunchConfiguration("rviz_config"))
    rviz_config = get_rviz_config_path(rviz_config_name)

    nodes = []

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
