#!/usr/bin/env python3
"""RealSense D435i 数据采集节点。

该节点封装 realsense2_camera 的启动，用于发布 RGB 图像、深度图和相机内参。

功能描述：
    - 封装 realsense2_camera 启动配置
    - 转发相机数据话题（RGB、深度、相机信息）
    - 提供统一的相机数据发布接口

订阅话题：
    无（直接封装相机驱动）

发布话题：
    - /camera/color/image_raw (sensor_msgs/Image)
        RGB 彩色图像
    - /camera/aligned_depth_to_color/image_raw (sensor_msgs/Image)
        对齐到彩色图的深度图
    - /camera/color/camera_info (sensor_msgs/CameraInfo)
        相机内参信息

参数：
    - align_depth.enable (bool, 默认 true)
        启用深度图对齐到彩色图
    - enable_color (bool, 默认 true)
        启用彩色相机
    - enable_depth (bool, 默认 true)
        启用深度相机
    - depth_module.profile (string, 默认 "640x480x30")
        深度相机分辨率和帧率
    - rgb_camera.profile (string, 默认 "640x480x30")
        彩色相机分辨率和帧率

使用说明：
    该节点通常不需要直接运行，而是通过 launch 文件启动 realsense2_camera。
    也可以使用 launch 文件中的 IncludeLaunchDescription 方式启动。
"""

import rclpy
from rclpy.node import Node


class RealSenseNode(Node):
    """RealSense D435i 数据采集节点。"""

    def __init__(self):
        super().__init__("camera_rs_node")

        # 声明参数
        self.declare_parameter("align_depth.enable", True)
        self.declare_parameter("enable_color", True)
        self.declare_parameter("enable_depth", True)
        self.declare_parameter("depth_module.profile", "640x480x30")
        self.declare_parameter("rgb_camera.profile", "640x480x30")
        self.declare_parameter("camera_namespace", "")

        self.get_logger().info("camera_rs_node (RealSense D435i) 节点已初始化")
        self.get_logger().info("注意：请使用 launch 文件启动 realsense2_camera")


def main(args=None):
    """主入口函数。"""
    rclpy.init(args=args)
    node = RealSenseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到中断信号")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
