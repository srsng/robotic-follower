#!/usr/bin/env python3
"""实机感知启动文件

启动完整的感知流水线用于实机测试：
1. RealSense D435i 相机（原生点云 + TF 模型发布）
2. 3D 检测器（消费 realsense 原生点云）
3. RViz 可视化（点云 + 检测包围盒）

架构说明：
    realsense2_camera 发布点云 /camera/camera/depth/color/points
    → detection_node 订阅，执行 3D 检测
    → rviz_visualizer_node 转为 MarkerArray
    → RViz 显示点云、目标包围盒等可视化

TF 说明：
    robot_state_publisher 发布机械臂 TF：world → base_link →  link1_1_1 → ... → link6_1_1 → camera_link
    realsense2_camera 发布相机 TF：camera_link → camera_color_optical_frame → camera_depth_optical_frame

    joint_states 来源：由 机械臂控制器 或 fake_joint_states_publisher 发布

启动命令：
    ros2 launch robotic_follower perception_real.launch.py
"""

import os

from ament_index_python import get_package_share_directory
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from robotic_follower.util.launch import (
    declare_configurable_parameters,
    set_configurable_parameters,
)


local_parameters = [
    {
        "name": "handeye_name",
        "default": "dummy_handeye",
        "description": "Name of the handeye calibration",
    },
    {
        "name": "use_fused_rgbd_pipeline",
        "default": "true",
        "description": "Use fused RGBD detect+track node",
    },
    {
        "name": "fused_config_file",
        "default": "model/config/yolov8_seg_rgbd_track.yaml",
        "description": "Config file for fused RGBD node",
    },
]


def generate_launch_description():
    """生成感知系统的 Launch 描述"""
    params = set_configurable_parameters(local_parameters)

    # 1. RealSense 相机启动
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
                )
            ]
        ),
        launch_arguments={
            "align_depth.enable": "true",
            "pointcloud.enable": "true",
            "enable_color": "true",
            "enable_depth": "true",
            "depth_module.profile": "640x480x30",
            "rgb_camera.profile": "640x480x30",
            "camera_namespace": "camera",
            "publish_tf": "true",  # 相机发布 TF
            # "base_frame_id": "base_link",
        }.items(),
    )

    # 2. 3D 检测器（旧链路）  TODO: 重构
    detection_node = Node(
        package="robotic_follower",
        executable="detection_node",
        name="detection_node",
        output="screen",
        condition=UnlessCondition(params["use_fused_rgbd_pipeline"]),
        parameters=[
            {
                "pointcloud_topic": "/camera/camera/depth/color/points",
                "target_frame": "base_link",
            }
        ],
    )

    # 2b. 融合 RGBD 检测追踪节点（新链路）
    fused_node = Node(
        package="robotic_follower",
        executable="rgbd_detect_track_node",
        name="rgbd_detect_track_node",
        output="screen",
        condition=IfCondition(params["use_fused_rgbd_pipeline"]),
        parameters=[
            {
                "target_frame": "base_link",
                "config_file": params["fused_config_file"],
            }
        ],
    )

    # 3. RViz 可视化（订阅变换后的点云）
    # 获取 rviz 配置路径
    rviz_config = os.path.join(
        get_package_share_directory("robotic_follower"),
        "rviz",
        "perception_rviz.rviz",
    )

    # 4. 感知数据转发节点（订阅变换后的点云）
    rviz_visualizer_node = Node(
        package="robotic_follower",
        executable="rviz_visualizer",
        name="rviz_visualizer",
        output="screen",
    )

    # 5. RViz2 窗口
    rviz_node = ExecuteProcess(
        cmd=["rviz2", "-d", rviz_config],
        output="screen",
        shell=False,
    )

    return LaunchDescription(
        [
            # 声明参数
            *declare_configurable_parameters(local_parameters),
            # 启动节点
            realsense_launch,
            detection_node,
            fused_node,
            rviz_visualizer_node,
            rviz_node,
        ]
    )
