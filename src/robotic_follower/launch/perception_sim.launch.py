#!/usr/bin/env python3
"""仿真感知启动文件。

使用模拟相机数据进行感知测试：
1. 模拟相机节点（从 .bin 文件发布数据）
2. 3D 检测器
3. 可视化 (可选 Open3D 或 RViz)

使用场景：
    - 离线数据测试
    - 无相机环境下的算法开发
    - SUNRGBD 数据集验证

启动命令：
    # 使用 SUNRGBD 索引 + Open3D 可视化（默认）
    ros2 launch robotic_follower perception_sim.launch.py sunrgbd_idx:=1

    # 使用 .bin 文件 + RViz 可视化
    ros2 launch robotic_follower perception_sim.launch.py bin_file:=/path/to/data.bin gui:=rviz
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from robotic_follower.util.launch import (
    declare_configurable_parameters,
    set_configurable_parameters,
    to_urdf,
)


local_parameters = [
    {
        "name": "bin_file",
        "default": "",
        "description": "Path to the .bin point cloud file",
    },
    {
        "name": "sunrgbd_idx",
        "default": "-1",
        "description": "SUNRGBD dataset sample index number (1 ~ 10335)",
    },
    {
        "name": "publish_rate",
        "default": "1.0",
        "description": "Topic publishing rate (Hz)",
    },
    {
        "name": "gui",
        "default": "open3d",
        "description": 'Visualizer GUI type: "open3d" or "rviz"',
    },
]


def get_visualizer_node(context, visualizer_type):
    """根据 visualizer_type 返回对应的可视化节点列表。"""
    viz_type = context.perform_substitution(visualizer_type)
    assert viz_type in ("open3d", "rviz"), (
        "only support open3d or rviz for Visualizer GUI"
    )

    if viz_type == "open3d":
        return [
            Node(
                package="robotic_follower",
                executable="open3d_visualizer",
                name="open3d_visualizer",
                output="screen",
            )
        ]

    # rviz: 感知数据转发 + RViz2 窗口
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    FindPackageShare("robotic_follower"),
                    "/launch/rviz_perception.launch.py",
                ]
            ),
            launch_arguments={
                "rviz_config": "perception_rviz",
            }.items(),
        )
    ]


def generate_launch_description():
    """生成感知仿真系统的 Launch 描述。"""
    params = set_configurable_parameters(local_parameters)

    # 1. 模拟相机节点
    camera_sim_node = Node(
        package="robotic_follower",
        executable="camera_sim_node",
        name="camera_sim_node",
        output="screen",
        parameters=[
            {
                "bin_file": params["bin_file"],
                "sunrgbd_idx": params["sunrgbd_idx"],
                "publish_rate": params["publish_rate"],
                "pack_rgb": str(params["gui"]) == "rviz",
            }
        ],
    )

    # 1.5 D435i URDF 路径（用于 robot_state_publisher 发布相机 TF）
    xacro_path = os.path.join(
        get_package_share_directory("realsense2_description"),
        "urdf",
        "test_d435i_camera.urdf.xacro",
    )
    camera_urdf = to_urdf(
        xacro_path, {"use_nominal_extrinsics": "true", "add_plug": "true"}
    )

    # 1.6 robot_state_publisher 发布 D435i 相机 TF
    robot_state_publisher_node = Node(
        name="camera_model_node",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="",
        output="screen",
        arguments=[camera_urdf],
    )

    # 3. 3D 检测器
    detection_node = Node(
        package="robotic_follower",
        executable="detection_node",
        name="detection_node",
        output="screen",
        parameters=[
            {
                "pointcloud_topic": "/camera/camera/depth/color/points",
                "source_frame": "camera_depth_optical_frame",
                "target_frame": "camera_depth_optical_frame",
            }
        ],
    )

    # 4. 可视化节点
    visualizer_node = OpaqueFunction(function=get_visualizer_node, args=[params["gui"]])

    return LaunchDescription(
        [
            *declare_configurable_parameters(local_parameters),
            camera_sim_node,
            robot_state_publisher_node,
            detection_node,
            visualizer_node,
        ]
    )
