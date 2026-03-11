# -*- coding: utf-8 -*-
"""
感知节点

点云处理 + 3D检测的单节点
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Header
import numpy as np
import cv2
from pathlib import Path

try:
    # OpenCV到ROS2消息转换
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

from cognition.point_cloud.io import depth_to_pointcloud, numpy_to_pointcloud2
from cognition.point_cloud.filters import VoxelFilter, StatisticalFilter, PassthroughFilter
from cognition.point_cloud.features import compute_density
from cognition.detection.inference import Detection


class PerceptionNode(Node):
    """感知节点：点云处理 + 3D检测"""

    def __init__(self):
        super().__init__('perception_node')

        # 参数
        self.declare_parameter('enable_detection', True)
        self.declare_parameter('enable_density_calc', True)
        self.declare_parameter('model_config_path', '')
        self.declare_parameter('checkpoint_path', '')
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('min_depth', 0.0)
        self.declare_parameter('max_depth', 3.0)

        # 话题
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_callback,
            QoSProfile(depth=10, reliability=1)
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            QoSProfile(depth=10, reliability=1)
        )

        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            '/cognition/processed_pointcloud',
            QoSProfile(depth=5)
        )
        self.detections_pub = self.create_publisher(
            MarkerArray,
            '/cognition/detections',
            QoSProfile(depth=5)
        )

        # 状态
        self.camera_info = None
        self.cv_bridge = CvBridge() if CvBridge else None

        # 初始化滤波器
        self.voxel_filter = VoxelFilter(voxel_size=0.02)
        self.statistical_filter = StatisticalFilter(nb_neighbors=20, std_ratio=2.0)
        self.pass_filter = PassthroughFilter(axis_name='z', min_limit=0.0, max_limit=3.0)

        # 初始化检测器（延迟加载）
        self.detector = None
        self.detector_initialized = False

        self.get_logger().info("感知节点已启动")

    def depth_callback(self, msg: Image):
        """深度图像回调"""
        if self.camera_info is None:
            self.get_logger().warn("等待相机内参...")
            return

        try:
            # 转换深度图像
            if self.cv_bridge:
                depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            else:
                # 假设已经是对齐的uint16
                depth_array = np.frombuffer(msg.data, dtype=np.uint16)
                depth_image = depth_array.reshape(msg.height, msg.width)

            # 深度图转点云
            depth_scale = self.get_parameter('depth_scale').value
            points, _ = depth_to_pointcloud(
                depth_image,
                self.camera_info,
                depth_scale=depth_scale
            )

            if len(points) == 0:
                self.get_logger().warn("转换后的点云为空")
                return

            self.get_logger().info(f"点云点数: {len(points)}")

            # 点云处理管道
            processed_points = self._process_pointcloud(points)

            # 发布处理后的点云
            self._publish_pointcloud(processed_points, msg.header)

            # 3D检测
            enable_detection = self.get_parameter('enable_detection').value
            if enable_detection:
                self._detect_objects(processed_points, msg.header)

        except Exception as e:
            self.get_logger().error(f"处理深度图像失败: {e}")

    def camera_info_callback(self, msg: CameraInfo):
        """相机内参回调"""
        self.camera_info = msg
        self.get_logger().info("收到相机内参")

    def _process_pointcloud(self, points: np.ndarray) -> np.ndarray:
        """点云处理管道"""
        # 1. 体素滤波（下采样）
        points = self.voxel_filter.filter(points)
        self.get_logger().debug(f"体素滤波后点数: {len(points)}")

        # 2. 统计滤波（去噪）
        points = self.statistical_filter.filter(points)
        self.get_logger().debug(f"统计滤波后点数: {len(points)}")

        # 3. 直通滤波（Z轴范围）
        min_depth = self.get_parameter('min_depth').value
        max_depth = self.get_parameter('max_depth').value
        self.pass_filter.set_limits(min_depth, max_depth)
        points = self.pass_filter.filter(points)
        self.get_logger().debug(f"直通滤波后点数: {len(points)}")

        return points

    def _detect_objects(self, points: np.ndarray, header: Header):
        """3D目标检测"""
        # 初始化检测器（第一次检测时）
        if not self.detector_initialized:
            self._init_detector()

        if self.detector is None:
            return

        # 计算密度（如果启用）
        enable_density = self.get_parameter('enable_density_calc').value
        density = None
        if enable_density:
            self.get_logger().info("计算点云密度...")
            density = compute_density(
                points,
                kernel_type='gaussian',
                bandwidth=0.5,
                k_neighbors=50,
                norm_type='minmax'
            )
            self.get_logger().info("密度计算完成")

        # 执行检测
        self.get_logger().info("执行3D检测...")
        detections = self.detector.detect(points, density=density)
        self.get_logger().info(f"检测到 {len(detections)} 个物体")

        # 发布检测结果
        self._publish_detections(detections, header)

    def _init_detector(self):
        """初始化检测器"""
        model_config_path = self.get_parameter('model_config_path').value
        checkpoint_path = self.get_parameter('checkpoint_path').value

        if not model_config_path or not Path(model_config_path).exists():
            self.get_logger().warn("模型配置文件不存在，跳过检测")
            return

        try:
            self.detector = Detection(
                model_config_path=model_config_path,
                checkpoint_path=checkpoint_path if checkpoint_path else None
            )
            self.detector_initialized = True
            self.get_logger().info("检测器初始化成功")
        except Exception as e:
            self.get_logger().error(f"检测器初始化失败: {e}")

    def _publish_pointcloud(self, points: np.ndarray, header: Header):
        """发布处理后的点云"""
        msg = numpy_to_pointcloud2(points, header=header, frame_id='camera_link')

        self.pointcloud_pub.publish(msg)

    def _publish_detections(self, detections: list, header: Header):
        """发布检测结果"""
        marker_array = MarkerArray()
        marker_array.header = header
        marker_array.markers = []

        # 为每个检测创建包围盒标记
        for i, det in enumerate(detections):
            marker = Marker()
            marker.header = header
            marker.ns = 'detections'
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # 设置位置和姿态
            marker.pose.position.x = det.center[0]
            marker.pose.position.y = det.center[1]
            marker.pose.position.z = det.center[2]

            # 设置尺寸
            marker.scale.x = det.size[0]
            marker.scale.y = det.size[1]
            marker.scale.z = det.size[2]

            # 设置颜色
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            # 设置朝向（Z轴旋转）
            marker.pose.orientation.w = np.cos(det.heading / 2)
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = np.sin(det.heading / 2)

            # 生命周期
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 500000000  # 0.5秒

            marker_array.markers.append(marker)

        self.detections_pub.publish(marker_array)


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()
