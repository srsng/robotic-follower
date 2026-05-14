#!/usr/bin/env python3
"""实机感知启动文件

启动完整的感知流水线用于实机测试：
1. RealSense D435i 相机（原生点云 + TF 模型发布）
2. 统一感知节点（实例分割 + 深度投影 + 3D 追踪）
3. RViz 可视化（点云 + 检测/追踪包围盒）

架构说明：
    realsense2_camera 发布 RGB / Depth / CameraInfo
    → detect_track_node 同步输入，执行检测和追踪
    → rviz_visualizer_node 转发调试图像和可视化消息
    → RViz 显示点云、目标包围盒、分割调试图等可视化

TF 说明：
    robot_state_publisher 发布机械臂 TF：world → base_link →  link1_1_1 → ... → link6_1_1 → camera_link
    realsense2_camera 发布相机 TF：camera_link → camera_color_optical_frame → camera_depth_optical_frame

    joint_states 来源：由 机械臂控制器 或 fake_joint_states_publisher 发布

启动命令：
    ros2 launch robotic_follower perception_real.launch.py
"""

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

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
        "name": "perception_config_file",
        "default": "model/config/yolov8_seg_rgbd_track.yaml",
        # "default": "model/config/fastsam_rgbd_track.yaml",
        "description": "Config file for unified perception node",
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
            "depth_module.depth_profile": "640x480x6",
            "rgb_camera.color_profile": "640x480x6",
            "camera_namespace": "camera",
            "publish_tf": "true",  # 相机发布 TF
            # "base_frame_id": "base_link",
        }.items(),
    )

    # 2. 统一 RGBD 检测追踪节点
    detect_track_node = Node(
        package="robotic_follower",
        executable="detect_track_node",
        name="detect_track_node",
        output="screen",
        parameters=[
            {
                "input_mode": "rgbd",
                "target_frame": "base_link",
                "config_file": params["perception_config_file"],
                "fallback_to_source_frame_when_tf_disconnected": False,
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
            detect_track_node,
            rviz_visualizer_node,
            rviz_node,
        ]
    )
