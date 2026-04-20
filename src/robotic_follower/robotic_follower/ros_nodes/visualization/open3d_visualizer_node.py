#!/usr/bin/env python3
"""Open3D 可视化节点 - 实时 3D 渲染。

该节点提供基于 Open3D 的实时 3D 可视化界面，支持点云、RGB/深度图像
和检测结果的联合可视化。适用于调试和演示场景。

功能描述：
    - 点云 3D 渲染（支持 XYZRGB 格式）
    - RGB 图像 2D 显示
    - 深度图伪彩色显示
    - 3D 检测框可视化（线框边界框）
    - 目标类别和置信度标签显示
    - 实时数据更新（20Hz 刷新率）
    - 坐标系参考系显示
    - 动态接收检测器类别信息

订阅话题：
    - /camera/camera/depth/color/points (sensor_msgs/PointCloud2)
        处理后的点云数据（与实机 realsense2_camera 保持一致）
    - /perception/detections (vision_msgs/Detection3DArray)
        3D 目标检测结果
    - /camera/color/image_raw (sensor_msgs/Image)
        RGB 彩色图像
    - /camera/aligned_depth_to_color/image_raw (sensor_msgs/Image)
        深度图
    - /camera/color/camera_info (sensor_msgs/CameraInfo)
        相机内参（用于深度图转点云）
    - /perception/class_names_info (std_msgs/String)
        检测器类别信息（JSON 格式）

GUI 布局：
    - 左侧 70%: 3D 点云场景（Open3D SceneWidget）
    - 右侧 30%: 2D 图像面板
        - RGB 图像显示
        - 深度图伪彩色显示
        - 检测目标列表

注意：
    - 需要图形环境（DISPLAY/WAYLAND_DISPLAY）
    - 无头服务器建议使用 RViz2 可视化
    - ROS2 运行在后台线程，GUI 运行在主线程
"""

import contextlib
import json
import threading
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray

from robotic_follower.point_cloud.io.ros_converters import pointcloud2_to_numpy
from robotic_follower.util.wrapper import NodeWrapper


class Open3DVisualizerNode(NodeWrapper):
    """Open3D 可视化节点。"""

    def __init__(self):
        super().__init__("open3d_visualizer_node")

        self.bridge = CvBridge()

        # 数据缓存与锁
        self.data_lock = threading.Lock()
        self.current_points = None
        self.current_detections = []
        self.current_rgb_image = None
        self.current_depth_image = None
        self.current_camera_info = None
        self.is_new_data = False

        # 类别信息（从检测节点动态接收）
        self.class_names = []
        self.idx2class_name = {}
        self._class_names_received = False

        # 订阅话题
        self.pc_sub = self.create_subscription(
            PointCloud2, "/camera/camera/depth/color/points", self.pc_callback, 10
        )
        self.det_sub = self.create_subscription(
            Detection3DArray, "/perception/detections", self.det_callback, 10
        )
        self.img_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.img_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, "/camera/aligned_depth_to_color/image_raw", self.depth_callback, 10
        )
        self.info_sub = self.create_subscription(
            CameraInfo, "/camera/color/camera_info", self.camera_info_callback, 10
        )
        self.class_names_sub = self.create_subscription(
            String, "/perception/class_names_info", self.class_names_callback, 10
        )

        self._info("Open3D 可视化节点已启动，等待数据...")

    def class_names_callback(self, msg: String):
        """检测器类别信息回调。"""
        try:
            info = json.loads(msg.data)
            with self.data_lock:
                self.class_names = info.get("class_names", [])
                self.idx2class_name = info.get("idx2class_name", {})
                self._class_names_received = True
            self._info(f"已接收类别信息: {self.class_names}")
        except Exception as e:
            self._error(f"类别信息解析失败: {e}")

    def pc_callback(self, msg: PointCloud2):
        """点云回调。"""
        try:
            points = pointcloud2_to_numpy(msg)
            with self.data_lock:
                self.current_points = points
                self.is_new_data = True
        except Exception as e:
            self._error(f"点云解析失败: {e}")

    def img_callback(self, msg: Image):
        """RGB 图像回调。"""
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.data_lock:
                self.current_rgb_image = img
                self.is_new_data = True
        except Exception as e:
            self._error(f"图像解析失败: {e}")

    def depth_callback(self, msg: Image):
        """深度图回调。"""
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            with self.data_lock:
                self.current_depth_image = depth
                self.is_new_data = True
        except Exception as e:
            self._error(f"深度图解析失败: {e}")

    def camera_info_callback(self, msg: CameraInfo):
        """相机信息回调。"""
        with self.data_lock:
            self.current_camera_info = msg

    def det_callback(self, msg: Detection3DArray):
        """检测结果回调。"""
        try:
            detections = []
            for det_msg in msg.detections:
                pos = det_msg.bbox.center.position
                size = det_msg.bbox.size
                yaw = self._quaternion_to_yaw(det_msg.bbox.center.orientation)

                bbox = [pos.x, pos.y, pos.z, size.x, size.y, size.z, yaw]

                label = "unknown"
                score = 0.0
                track_id = None
                if len(det_msg.results) > 0:
                    label = det_msg.results[0].hypothesis.class_id
                    score = det_msg.results[0].hypothesis.score

                # 从 Detection3D.id 字段提取 track_id
                if det_msg.id:
                    track_id = det_msg.id

                detections.append(
                    {
                        "bbox": bbox,
                        "label": label,
                        "score": score,
                        "track_id": track_id,
                    }
                )

            with self.data_lock:
                self.current_detections = detections
                self.is_new_data = True
        except Exception as e:
            self._error(f"检测结果解析失败: {e}")

    @staticmethod
    def _quaternion_to_yaw(q) -> float:
        """四元数转 yaw 角。"""
        import math

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def run_ros_spin(node):
    """在一个独立的线程中运行 ROS2 spin。"""
    rclpy.spin(node)


def main(args=None):
    """主入口函数。"""
    import os

    if os.environ.get("DISPLAY") is None and os.environ.get("WAYLAND_DISPLAY") is None:
        print(
            "错误: 没有图形环境 (DISPLAY/WAYLAND_DISPLAY 未设置)，Open3D GUI 无法运行"
        )
        print("提示: 对于无头服务器，请使用 RViz2 进行可视化")
        return

    rclpy.init(args=args)
    node = Open3DVisualizerNode()

    # 启动后台 ROS 线程
    ros_thread = threading.Thread(target=run_ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    # 在主线程中初始化并运行 Open3D GUI
    import cv2
    import open3d as o3d
    import open3d.visualization.gui as gui
    import open3d.visualization.rendering as rendering

    from robotic_follower.visualization.visualizer import DetectionVisualizer

    app = gui.Application.instance
    try:
        app.initialize()
    except Exception as e:
        print(f"错误: Open3D GUI 初始化失败: {e}")
        node.destroy_node()
        rclpy.shutdown()
        return

    viz = DetectionVisualizer("Open3D Real-time Visualizer")

    # 构建窗体
    window = app.create_window("Real-time Perception Visualizer", 1600, 900)
    widget3d = gui.SceneWidget()
    widget3d.scene = rendering.Open3DScene(window.renderer)

    em = window.theme.font_size
    side_panel = gui.Vert(
        0, gui.Margins(int(0.5 * em), int(0.5 * em), int(0.5 * em), int(0.5 * em))
    )

    image_widget = gui.ImageWidget()
    side_panel.add_child(gui.Label("2D RGB View"))
    side_panel.add_child(image_widget)

    depth_image_widget = gui.ImageWidget()
    side_panel.add_child(gui.Label("2D Depth View"))
    side_panel.add_child(depth_image_widget)

    side_panel.add_child(gui.Label("Detected objects:"))
    detection_label = gui.Label("Waiting for detections...")
    side_panel.add_child(detection_label)

    window.add_child(widget3d)
    window.add_child(side_panel)

    node._depth_image_widget = depth_image_widget

    def on_layout(layout_context):
        r = window.content_rect
        panel_width = int(r.width * 0.3)
        widget3d.frame = gui.Rect(r.x, r.y, int(r.width * 0.7), r.height)
        side_panel.frame = gui.Rect(
            r.x + int(r.width * 0.7), r.y, panel_width, r.height
        )

    window.set_on_layout(on_layout)

    is_camera_setup = False
    current_labels = []

    def update_gui():
        nonlocal is_camera_setup, current_labels

        with node.data_lock:
            if not node.is_new_data:
                return
            node.is_new_data = False
            points = node.current_points
            dets = list(node.current_detections)
            rgb = node.current_rgb_image
            depth = node.current_depth_image
            cam_info = node.current_camera_info
            class_names = list(node.class_names)
            class_names_received = node._class_names_received

        if points is None:
            return

        # 刷新 2D 图像
        if rgb is not None:
            rgb_vis = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
            try:
                o3d_img = o3d.geometry.Image(rgb_vis)
                image_widget.update_image(o3d_img)
            except Exception as e:
                node._error(f"RGB ImageWidget update failed: {e}")

        if depth is not None:
            try:
                depth_norm = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
                depth_uint8 = depth_norm.astype(np.uint8)
                depth_colored = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_JET)
                depth_o3d_img = o3d.geometry.Image(depth_colored)
                depth_image_widget.update_image(depth_o3d_img)
            except Exception as e:
                node._error(f"Depth ImageWidget update failed: {e}")

        # 刷新 3D 场景
        widget3d.scene.clear_geometry()

        for lbl in current_labels:
            with contextlib.suppress(Exception):
                widget3d.remove_3d_label(lbl)
        current_labels.clear()

        detection_texts = []

        # 准备点云
        pcd = o3d.geometry.PointCloud()
        if points.shape[1] >= 6:
            pcd.points = o3d.utility.Vector3dVector(points[:, :3])
            pcd.colors = o3d.utility.Vector3dVector(points[:, 3:6])
        else:
            pcd.points = o3d.utility.Vector3dVector(points[:, :3])
            pcd.paint_uniform_color([0.5, 0.5, 0.5])

        mat_pcd = rendering.MaterialRecord()
        mat_pcd.shader = "defaultUnlit"
        widget3d.scene.add_geometry("Point Cloud", pcd, mat_pcd)

        # 坐标系
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        widget3d.scene.add_geometry("Coordinate Frame", coord_frame, mat_pcd)

        # 检测框
        for i, det in enumerate(dets):
            bbox_geom = viz._create_bbox_geometry(det)
            bbox_lineset = o3d.geometry.LineSet.create_from_oriented_bounding_box(
                bbox_geom
            )
            bbox_lineset.paint_uniform_color(bbox_geom.color)
            mat_bbox = rendering.MaterialRecord()
            mat_bbox.shader = "unlitLine"
            mat_bbox.line_width = 2.0
            widget3d.scene.add_geometry(f"BBox_{i}", bbox_lineset, mat_bbox)

            label = det.get("label", "unknown")
            track_id = det.get("track_id")

            # 将 label 解析为可读名称：
            # 如果 label 是数字字符串（类别索引），尝试查 class_names 表；
            # 否则直接使用字符串（已经是类名如 "ground", "cluster_0"）
            try:
                label_idx = int(label)
                if class_names_received and 0 <= label_idx < len(class_names):
                    label_name = class_names[label_idx]
                else:
                    label_name = f"Obj_{label_idx}"
            except (ValueError, TypeError):
                label_name = label

            score = det.get("score", 0.0)
            if track_id:
                text = f"#{track_id} {label_name}: {score:.2f}"
                detection_texts.append(f"#{track_id} {label_name} ({score:.2f})")
            else:
                text = f"{label_name}: {score:.2f}"
                detection_texts.append(f"[{i + 1}] {label_name} ({score:.2f})")

            bbox_arr = det.get("bbox", [0, 0, 0, 0, 0, 0, 0])
            bbox_center = np.array(bbox_arr[:3])
            bbox_height = bbox_arr[5]
            label_pos = bbox_center + np.array([0, 0, bbox_height / 2 + 0.1])
            lbl = widget3d.add_3d_label(label_pos, text)
            if lbl is not None:
                current_labels.append(lbl)

        if not detection_texts:
            detection_label.text = "No objects detected."
        else:
            detection_label.text = "\n".join(detection_texts)

        if not is_camera_setup:
            bounds = widget3d.scene.bounding_box
            widget3d.setup_camera(60.0, bounds, bounds.get_center())
            eye = np.array([0, -3, 0])
            lookat = np.array([0, 1, 0])
            up = np.array([0, 0, 1])
            widget3d.look_at(lookat, eye, up)
            is_camera_setup = True

    def background_poller():
        while True:
            time.sleep(0.05)  # 20fps check
            if node.is_new_data:
                gui.Application.instance.post_to_main_thread(window, update_gui)

    poller_thread = threading.Thread(target=background_poller, daemon=True)
    poller_thread.start()

    app.run()

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
