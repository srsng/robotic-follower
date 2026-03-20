#!/usr/bin/env python3
"""RViz 可视化节点 - 纯数据转换，不做感知算法。

该节点提供话题转发和数据格式转换功能，优化 RViz2 可视化体验。
将感知模块的原始话题转换为 RViz2 友好的显示格式。

功能描述：
    - 点云话题转发（简单分流）
    - 3D 检测框转换为 RViz2 Marker（线框边界框）
    - 检测类别标签可视化（文本标记）
    - 深度图伪彩色渲染（增强可视化效果）
    - RGB 图像转发（话题分流）

订阅话题：
    - /perception/processed_pointcloud (sensor_msgs/PointCloud2)
        处理后的点云数据
    - /perception/detections (vision_msgs/Detection3DArray)
        3D 目标检测结果
    - /camera/aligned_depth_to_color/image_raw (sensor_msgs/Image)
        对齐到彩色图的深度图
    - /camera/color/image_raw (sensor_msgs/Image)
        RGB 彩色图像

发布话题：
    - /perception/pointcloud_display (sensor_msgs/PointCloud2)
        用于 RViz 显示的点云
    - /perception/detection_markers (visualization_msgs/MarkerArray)
        3D 检测框和标签标记
    - /perception/depth_colored (sensor_msgs/Image)
        伪彩色深度图（Jet 色图）
    - /perception/rgb_display (sensor_msgs/Image)
        用于显示的 RGB 图像

Marker 类型：
    - LINE_STRIP: 12 条边组成的线框边界框（黄色）
    - TEXT_VIEW_FACING: 类别和置信度标签（白色，悬浮在框顶）

深度图伪彩色处理：
    - 百分位裁断归一化（5%~95%）
    - 避免异常值影响可视化
    - OpenCV COLORMAP_JET 色图

设计理念：
    - 纯数据转发，不包含感知算法
    - 职责单一：可视化优化
    - 可独立运行，不影响主感知流水线
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray


class RVizVisualizerNode(Node):
    """RViz 可视化节点 - 负责话题转发和数据格式转换。"""

    def __init__(self):
        super().__init__("rviz_visualizer")

        self.bridge = CvBridge()

        # ========== 点云转发 ==========
        self.pc_sub = self.create_subscription(
            PointCloud2,
            "/perception/processed_pointcloud",
            self.pointcloud_callback,
            10,
        )
        self.pc_pub = self.create_publisher(
            PointCloud2, "/perception/pointcloud_display", 10
        )

        # ========== 检测框可视化 ==========
        self.det_sub = self.create_subscription(
            Detection3DArray, "/perception/detections", self.detection_callback, 10
        )
        self.det_pub = self.create_publisher(
            MarkerArray, "/perception/detection_markers", 10
        )

        # ========== 深度图伪彩色 ==========
        self.depth_sub = self.create_subscription(
            Image, "/camera/aligned_depth_to_color/image_raw", self.depth_callback, 10
        )
        self.depth_pub = self.create_publisher(Image, "/perception/depth_colored", 10)

        # ========== 彩色图转发 ==========
        self.rgb_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.rgb_callback, 10
        )
        self.rgb_pub = self.create_publisher(Image, "/perception/rgb_display", 10)

        self.get_logger().info("RViz 可视化节点已启动")

    def pointcloud_callback(self, msg: PointCloud2):
        """点云直接转发（话题分流）。"""
        self.pc_pub.publish(msg)

    def rgb_callback(self, msg: Image):
        """彩色图直接转发。"""
        self.rgb_pub.publish(msg)

    def detection_callback(self, msg: Detection3DArray):
        """Detection3DArray → MarkerArray（线框边界框）。"""
        marker_array = MarkerArray()

        # DELETEALL 清除旧标记
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        for i, det in enumerate(msg.detections):
            # 1. 线框边界框
            wire_marker = self._create_wirebox_marker(det, i, msg.header)
            marker_array.markers.append(wire_marker)

            # 2. 类别标签
            text_marker = self._create_label_marker(det, i, msg.header)
            marker_array.markers.append(text_marker)

        self.det_pub.publish(marker_array)

    def _create_wirebox_marker(self, det, index, header) -> Marker:
        """创建线框边界框（12条边）。"""
        marker = Marker()
        marker.header = header
        marker.ns = "wire_boxes"
        marker.id = index
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        center = det.bbox.center.position
        size = det.bbox.size
        sx, sy, sz = size.x / 2, size.y / 2, size.z / 2

        # 8个角点
        corners = [
            [-sx, -sy, -sz],
            [sx, -sy, -sz],
            [sx, sy, -sz],
            [-sx, sy, -sz],
            [-sx, -sy, sz],
            [sx, -sy, sz],
            [sx, sy, sz],
            [-sx, sy, sz],
        ]

        # 12条边连接顺序
        edges = [0, 1, 2, 3, 0, 4, 5, 6, 7, 4, 5, 1, 2, 6, 7, 3, 0]

        for idx in edges:
            p = Point()
            p.x = center.x + corners[idx][0]
            p.y = center.y + corners[idx][1]
            p.z = center.z + corners[idx][2]
            marker.points.append(p)

        marker.scale.x = 0.02  # 线宽2cm
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        return marker

    def _create_label_marker(self, det, index, header) -> Marker:
        """创建类别标签。"""
        marker = Marker()
        marker.header = header
        marker.ns = "labels"
        marker.id = index
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        center = det.bbox.center.position
        size = det.bbox.size

        marker.pose.position.x = center.x
        marker.pose.position.y = center.y
        marker.pose.position.z = center.z + size.z / 2.0 + 0.1
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.08

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        # 提取类别和置信度
        class_id = "Unknown"
        score = 0.0
        if det.results:
            class_id = det.results[0].hypothesis.class_id
            score = det.results[0].hypothesis.score

        marker.text = f"{class_id} {score:.2f}"
        return marker

    def depth_callback(self, msg: Image):
        """深度图 16UC1 → 伪彩色。"""
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")

            # 百分位裁断归一化（避免异常值影响）
            valid_depth = depth[depth > 0]
            if len(valid_depth) > 0:
                p5, p95 = np.percentile(valid_depth, [5, 95])
                depth_clipped = np.clip(depth, p5, p95)
                depth_norm = ((depth_clipped - p5) / (p95 - p5) * 255).astype(np.uint8)
            else:
                depth_norm = np.zeros_like(depth, dtype=np.uint8)

            depth_colored = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
            depth_msg = self.bridge.cv2_to_imgmsg(depth_colored, "rgb8")
            depth_msg.header = msg.header
            self.depth_pub.publish(depth_msg)
        except Exception as e:
            self.get_logger().error(f"深度图伪彩色处理失败: {e}")


def main(args=None):
    """主入口函数。"""
    rclpy.init(args=args)
    node = RVizVisualizerNode()

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
