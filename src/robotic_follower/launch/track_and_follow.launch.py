#!/usr/bin/env python3
"""机器人跟随系统启动文件

启动追踪-跟随系统用于实机测试：

数据流：
    /perception/detections (3D检测结果)
        → tracking_node 执行多目标追踪
        → track_selector_node UI 选择目标
        → following_node 计算跟随点并驱动机械臂

启动命令：
    ros2 launch robotic_follower track_and_follow.launch.py

参数：
    follow_distance: 跟随距离（米），默认 0.15
    tracking_iou_threshold: 追踪 IOU 阈值，默认 0.3
    tracking_max_age: 追踪最大存活帧数，默认 30
    tracking_min_hits: 追踪最小命中次数，默认 3
"""

from launch_ros.actions import Node

from launch import LaunchDescription
from robotic_follower.util.launch import (
    declare_configurable_parameters,
    set_configurable_parameters,
)


local_parameters = [
    {
        "name": "follow_distance",
        "default": "0.15",
        "description": "Following distance in meters",
    },
    {
        "name": "tracking_iou_threshold",
        "default": "0.3",
        "description": "IOU threshold for tracking",
    },
    {
        "name": "tracking_max_age",
        "default": "30",
        "description": "Max age for tracking tracks",
    },
    {
        "name": "tracking_min_hits",
        "default": "3",
        "description": "Min hits for tracking tracks",
    },
]


def generate_launch_description():
    """生成全功能感知的 Launch 描述"""
    params = set_configurable_parameters(local_parameters)

    # # 1. 3D 多目标追踪节点
    # tracking_node = Node(
    #     package="robotic_follower",
    #     executable="tracking_node",
    #     name="tracking_node",
    #     output="screen",
    #     parameters=[
    #         {
    #             "input_topic": "/perception/detections",
    #             "output_topic": "/perception/tracked_objects",
    #             "iou_threshold": params["tracking_iou_threshold"],
    #             "max_age": params["tracking_max_age"],
    #             "min_hits": params["tracking_min_hits"],
    #         }
    #     ],
    # )

    # 2. 目标选择器 UI 节点
    track_selector_node = Node(
        package="robotic_follower",
        executable="track_selector_node",
        name="track_selector_node",
        output="screen",
        parameters=[
            {
                "tracking_topic": "/perception/tracked_objects",
            }
        ],
    )

    # 3. 目标跟随节点
    following_node = Node(
        package="robotic_follower",
        executable="following_node",
        name="following_node",
        output="screen",
        parameters=[
            {
                "follow_distance": params["follow_distance"],
                "tracking_topic": "/perception/tracked_objects",
                "selected_topic": "/perception/selected_target",
                "end_effector_frame": "link6_1_1",
                "update_rate": 5.0,
            }
        ],
    )

    return LaunchDescription(
        [
            # 声明参数
            *declare_configurable_parameters(local_parameters),
            # 启动节点
            # tracking_node,
            track_selector_node,
            following_node,
        ]
    )
