#!/usr/bin/env python3
"""点云处理器节点。

该节点接收深度图像或原始点云，进行滤波处理和密度计算后输出处理后的点云。

功能描述：
    - 接收 RealSense 深度图或原始点云
    - 使用相机内参将深度图转换为点云
    - 点云滤波（体素下采样、统计滤波、直通滤波）
    - 点云密度计算（高斯核密度估计）
    - 发布处理后的点云

订阅话题：
    - /camera/aligned_depth_to_color/image_raw (sensor_msgs/Image)
        对齐到彩色图的深度图
    - /camera/color/camera_info (sensor_msgs/CameraInfo)
        相机内参信息
    - /camera/color/image_raw (sensor_msgs/Image)
        RGB 彩色图像（用于点云染色，可选）

发布话题：
    - /camera/camera/depth/color/points (sensor_msgs/PointCloud2)
        处理后的点云（XYZ 或 XYZRGB 格式）

参数：
    - voxel_size (float, 默认 0.01)
        体素下采样尺寸（米）
    - statistical_nb_neighbors (int, 默认 20)
        统计滤波邻域点数
    - statistical_std_ratio (float, 默认 2.0)
        统计滤波标准差倍数
    - passthrough_axis (string, 默认 "z")
        直通滤波轴
    - passthrough_min (float, 默认 0.3)
        直通滤波最小值（米）
    - passthrough_max (float, 默认 2.0)
        直通滤波最大值（米）
    - density_bandwidth (float, 默认 0.05)
        密度核带宽
    - density_kernel (string, 默认 "gaussian")
        密度核类型
    - use_inverse_density (bool, 默认 True)
        使用逆密度
    - enable_color (bool, 默认 True)
        使用 RGB 图像给点云染色
"""

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

from robotic_follower.point_cloud.features.density import DensityCalculator
from robotic_follower.point_cloud.filters.filters import create_default_filter_pipeline
from robotic_follower.point_cloud.io.converters import (
    depth_to_pointcloud,
    extract_camera_intrinsics_from_msg,
)
from robotic_follower.point_cloud.io.ros_converters import numpy_to_pointcloud2


class PointCloudProcessorNode(Node):
    """点云处理器节点。"""

    def __init__(self):
        super().__init__("pointcloud_processor")

        # 滤波器参数
        self.declare_parameter("voxel_size", 0.01)
        self.declare_parameter("statistical_nb_neighbors", 20)
        self.declare_parameter("statistical_std_ratio", 2.0)
        self.declare_parameter("passthrough_axis", "z")
        self.declare_parameter("passthrough_min", 0.3)
        self.declare_parameter("passthrough_max", 2.0)

        # 密度参数
        self.declare_parameter("density_bandwidth", 0.05)
        self.declare_parameter("density_kernel", "gaussian")
        self.declare_parameter("use_inverse_density", True)

        # 染色参数
        self.declare_parameter("enable_color", True)

        # 初始化组件
        self.bridge = CvBridge()
        self.camera_intrinsics = None
        self.rgb_image = None

        # 创建滤波器
        filter_config = {
            "voxel_size": self.get_parameter("voxel_size").value,
            "statistical_nb_neighbors": self.get_parameter(
                "statistical_nb_neighbors"
            ).value,
            "statistical_std_ratio": self.get_parameter("statistical_std_ratio").value,
            "passthrough_axis": self.get_parameter("passthrough_axis").value,
            "passthrough_min": self.get_parameter("passthrough_min").value,
            "passthrough_max": self.get_parameter("passthrough_max").value,
        }
        self.filter_pipeline = create_default_filter_pipeline(filter_config)

        # 创建密度计算器
        density_config = {
            "bandwidth": self.get_parameter("density_bandwidth").value,
            "kernel": self.get_parameter("density_kernel").value,
            "normalize": True,
        }
        self.density_calculator = DensityCalculator(
            bandwidth=density_config["bandwidth"],
            kernel=density_config["kernel"],
            normalize=density_config["normalize"],
        )

        self.use_inverse_density = self.get_parameter("use_inverse_density").value
        self.enable_color = self.get_parameter("enable_color").value

        # 订阅话题
        self.depth_sub = self.create_subscription(
            Image, "/camera/aligned_depth_to_color/image_raw", self.depth_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/camera/color/camera_info", self.camera_info_callback, 10
        )
        self.rgb_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.rgb_callback, 5
        )

        # 发布话题
        self.pointcloud_pub = self.create_publisher(
            PointCloud2, "/camera/camera/depth/color/points", 10
        )

        self.get_logger().info("点云处理器节点已启动")

    def camera_info_callback(self, msg: CameraInfo):
        """相机内参回调。"""
        if self.camera_intrinsics is None:
            self.camera_intrinsics = extract_camera_intrinsics_from_msg(msg)
            self.get_logger().info(f"已接收相机内参: {self.camera_intrinsics}")

    def rgb_callback(self, msg: Image):
        """RGB 图像回调。"""
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"RGB 图像转换失败: {e}")

    def depth_callback(self, msg: Image):
        """深度图回调。"""
        if self.camera_intrinsics is None:
            self.get_logger().warn("等待相机内参...")
            return

        try:
            # 深度图转点云
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            points = depth_to_pointcloud(
                depth_image, self.camera_intrinsics, depth_scale=0.001, max_depth=10.0
            )

            if len(points) < 1000:
                self.get_logger().warn("点云点数过少，跳过处理")
                return

            # 点云滤波
            filtered_points = self.filter_pipeline.filter(points)

            if len(filtered_points) < 50:
                self.get_logger().warn("滤波后点数过少")
                return

            # 点云染色（如果有 RGB 图像）
            if self.enable_color and self.rgb_image is not None:
                from robotic_follower.point_cloud.io.projection import (
                    colorize_pointcloud,
                )

                # 构建 3x3 内参矩阵 K（从字典转换为 numpy 数组）
                fx = self.camera_intrinsics["fx"]
                fy = self.camera_intrinsics["fy"]
                cx = self.camera_intrinsics["cx"]
                cy = self.camera_intrinsics["cy"]
                K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)

                colored = colorize_pointcloud(
                    filtered_points[:, :3], self.rgb_image, None, K
                )

            # 发布点云
            # 根据点云维度决定是否打包 RGB（RViz RGB8 颜色变换需要 pack_rgb=True）
            has_color = filtered_points.shape[1] == 6
            pointcloud_msg = numpy_to_pointcloud2(
                filtered_points,
                frame_id="camera_depth_optical_frame",
                stamp=msg.header.stamp,
                pack_rgb=has_color,
            )
            self.pointcloud_pub.publish(pointcloud_msg)

        except Exception as e:
            self.get_logger().error(f"点云处理失败: {e}")


def main(args=None):
    """主入口函数。"""
    rclpy.init(args=args)
    node = PointCloudProcessorNode()

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
