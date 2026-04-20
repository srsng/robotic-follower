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
    - /camera/camera/depth/color/points (sensor_msgs/PointCloud2)
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
from robotic_follower_msgs.msg import TrackedObject3DArray
from sensor_msgs.msg import Image, PointCloud2
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

from robotic_follower.util.wrapper import NodeWrapper


class RVizVisualizerNode(NodeWrapper):
    """RViz 可视化节点 - 负责话题转发和数据格式转换。"""

    def __init__(self):
        super().__init__("rviz_visualizer")

        self.bridge = CvBridge()

        # ========== 点云转发 ==========
        self.pc_sub = self.create_subscription(
            PointCloud2,
            "/camera/camera/depth/color/points",
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

        self.tracked_custom_sub = self.create_subscription(
            TrackedObject3DArray,
            "/perception/tracked_objects_custom",
            self.tracked_custom_callback,
            10,
        )
        self.tracked_custom_pub = self.create_publisher(
            MarkerArray, "/perception/tracked_object_markers", 10
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

        self._info("RViz 可视化节点已启动")

    def pointcloud_callback(self, msg: PointCloud2):
        """点云回调：将 r,g,b 转换为 rgb 打包格式供 RViz 显示。"""
        # 检查是否有分开的 r, g, b 字段
        field_names = [f.name for f in msg.fields]
        has_separate_rgb = (
            "r" in field_names and "g" in field_names and "b" in field_names
        )

        if has_separate_rgb and msg.width > 0:
            # 转换为 rgb 打包格式
            converted_msg = self._convert_to_packed_rgb(msg)
            self.pc_pub.publish(converted_msg)
        else:
            self.pc_pub.publish(msg)

    @staticmethod
    def _convert_to_packed_rgb(msg: PointCloud2) -> PointCloud2:
        """将分开的 r,g,b 字段转换为打包的 rgb 字段。"""
        import numpy as np
        from sensor_msgs.msg import PointCloud2, PointField

        # 解析字段偏移
        field_offsets = {f.name: f.offset for f in msg.fields}

        n_points = msg.width * msg.height

        # 读取 x, y, z, r, g, b
        data = np.frombuffer(msg.data, dtype=np.uint8).reshape(n_points, -1)

        x = data[:, field_offsets["x"] : field_offsets["x"] + 4].view(np.float32)
        y = data[:, field_offsets["y"] : field_offsets["y"] + 4].view(np.float32)
        z = data[:, field_offsets["z"] : field_offsets["z"] + 4].view(np.float32)
        r = data[:, field_offsets["r"] : field_offsets["r"] + 4].view(np.float32)
        g = data[:, field_offsets["g"] : field_offsets["g"] + 4].view(np.float32)
        b = data[:, field_offsets["b"] : field_offsets["b"] + 4].view(np.float32)

        # 归一化到 [0, 1]
        r_norm = np.clip(r, 0, 1)
        g_norm = np.clip(g, 0, 1)
        b_norm = np.clip(b, 0, 1)

        # 如果是 [0, 255] 范围则归一化
        if r_norm.max() > 1.0:
            r_norm = r_norm / 255.0
            g_norm = g_norm / 255.0
            b_norm = b_norm / 255.0

        # 转换为 uint8 再打包为 ABGR 格式（RViz RGB8 使用）
        r_u8 = (r_norm * 255).astype(np.uint8)
        g_u8 = (g_norm * 255).astype(np.uint8)
        b_u8 = (b_norm * 255).astype(np.uint8)

        rgb_packed = (
            (r_u8.astype(np.uint32) << 16)
            | (g_u8.astype(np.uint32) << 8)
            | b_u8.astype(np.uint32)
        ).view(np.float32)

        # 构建新的点云数据 (x, y, z, rgb)
        new_data = np.column_stack([x, y, z, rgb_packed]).astype(np.float32)

        # 创建新的 PointCloud2 消息
        new_msg = PointCloud2()
        new_msg.header = msg.header
        new_msg.height = 1
        new_msg.width = n_points
        new_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        new_msg.point_step = 16
        new_msg.row_step = 16 * n_points
        new_msg.data = new_data.tobytes()
        new_msg.is_dense = True
        new_msg.is_bigendian = False

        return new_msg

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

        # 提取追踪 ID、类别和置信度
        track_id = det.id if det.id else None
        class_id = "Unknown"
        score = 0.0
        if det.results:
            class_id = det.results[0].hypothesis.class_id
            score = det.results[0].hypothesis.score

        if track_id:
            marker.text = f"#{track_id} {class_id} {score:.2f}"
        else:
            marker.text = f"{class_id} {score:.2f}"
        return marker

    def tracked_custom_callback(self, msg: TrackedObject3DArray):
        """TrackedObject3DArray -> MarkerArray（含质量字段文本）。"""
        marker_array = MarkerArray()

        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        for i, obj in enumerate(msg.objects):
            wire_marker = Marker()
            wire_marker.header = msg.header
            wire_marker.ns = "tracked_wire_boxes"
            wire_marker.id = i
            wire_marker.type = Marker.LINE_STRIP
            wire_marker.action = Marker.ADD

            center = obj.center
            size = obj.size
            sx, sy, sz = size.x / 2.0, size.y / 2.0, size.z / 2.0
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
            edges = [0, 1, 2, 3, 0, 4, 5, 6, 7, 4, 5, 1, 2, 6, 7, 3, 0]
            for idx in edges:
                p = Point()
                p.x = center.x + corners[idx][0]
                p.y = center.y + corners[idx][1]
                p.z = center.z + corners[idx][2]
                wire_marker.points.append(p)

            wire_marker.scale.x = 0.02
            if obj.graspable:
                wire_marker.color.r = 0.1
                wire_marker.color.g = 0.9
                wire_marker.color.b = 0.2
            else:
                wire_marker.color.r = 1.0
                wire_marker.color.g = 0.3
                wire_marker.color.b = 0.0
            wire_marker.color.a = 1.0
            marker_array.markers.append(wire_marker)

            text_marker = Marker()
            text_marker.header = msg.header
            text_marker.ns = "tracked_labels"
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = center.x
            text_marker.pose.position.y = center.y
            text_marker.pose.position.z = center.z + sz + 0.12
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.07
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            stale_tag = "STALE" if obj.is_stale else "fresh"
            grasp_tag = "graspable" if obj.graspable else "blocked"
            text_marker.text = (
                f"#{obj.tracking_id} {obj.label} {obj.score:.2f}\\n"
                f"occ={obj.occlusion_ratio:.2f} {stale_tag} {grasp_tag}"
            )
            marker_array.markers.append(text_marker)

        self.tracked_custom_pub.publish(marker_array)

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
            self._error(f"深度图伪彩色处理失败: {e}")


def main(args=None):
    """主入口函数。"""
    rclpy.init(args=args)
    node = RVizVisualizerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._info("收到中断信号")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
