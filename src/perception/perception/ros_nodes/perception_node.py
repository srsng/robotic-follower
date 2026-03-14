"""感知 ROS2 节点。"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose, BoundingBox3D
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from cv_bridge import CvBridge
import numpy as np
from typing import Optional
import yaml
import os

# 导入内部模块
from perception.point_cloud.io.converters import (
    depth_to_pointcloud,
    extract_camera_intrinsics_from_msg
)
from perception.point_cloud.io.ros_converters import numpy_to_pointcloud2
from perception.point_cloud.filters.filters import create_default_filter_pipeline
from perception.point_cloud.features.density import DensityCalculator
from perception.detection.inference.detector import create_detector_from_config
from perception.visualization.visualizer import DetectionVisualizer


class PerceptionNode(Node):
    """感知节点：点云处理 + 3D 目标检测。"""

    def __init__(self):
        super().__init__('perception_node')

        # 加载配置
        self.declare_parameter('config_file', '')
        self.declare_parameter('enable_visualization', True)
        config_file = self.get_parameter('config_file').value
        self.enable_viz = self.get_parameter('enable_visualization').value

        if config_file and os.path.exists(config_file):
            with open(config_file, 'r') as f:
                self.config = yaml.safe_load(f)
        else:
            self.config = self._get_default_config()

        # 初始化组件
        self.bridge = CvBridge()
        self.camera_intrinsics = None
        self.filter_pipeline = create_default_filter_pipeline(
            self.config.get('filtering', {})
        )
        self.density_calculator = DensityCalculator(
            bandwidth=self.config.get('density', {}).get('bandwidth', 0.05),
            kernel=self.config.get('density', {}).get('kernel', 'gaussian'),
            normalize=True
        )

        # 初始化检测器
        detector_config = self.config.get('detector', {})
        if detector_config:
            self.detector = create_detector_from_config(detector_config)
        else:
            self.get_logger().warn('未配置检测器，使用模拟检测')
            self.detector = None

        # 初始化可视化器
        if self.enable_viz:
            self.visualizer = DetectionVisualizer(window_name="Perception Node - 3D Detection")
            self.get_logger().info('可视化已启用')
        else:
            self.visualizer = None
            self.get_logger().info('可视化已禁用')

        # 订阅话题
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        # 发布话题
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            '/perception/processed_pointcloud',
            10
        )
        self.detections_pub = self.create_publisher(
            Detection3DArray,
            '/perception/detections',
            10
        )

        self.get_logger().info('感知节点已启动')

    def _get_default_config(self) -> dict:
        """获取默认配置。"""
        return {
            'filtering': {
                'voxel_size': 0.01,
                'statistical_nb_neighbors': 20,
                'statistical_std_ratio': 2.0,
                'passthrough_axis': 'z',
                'passthrough_min': 0.3,
                'passthrough_max': 2.0,
            },
            'density': {
                'bandwidth': 0.05,
                'kernel': 'gaussian',
                'use_inverse': True,
            },
            'detector': {
                'type': 'standard',
                'config_file': '',
                'checkpoint_file': '',
                'device': 'cuda:0',
                'score_threshold': 0.3,
            }
        }

    def camera_info_callback(self, msg: CameraInfo):
        """相机内参回调。"""
        if self.camera_intrinsics is None:
            self.camera_intrinsics = extract_camera_intrinsics_from_msg(msg)
            self.get_logger().info(f'已接收相机内参: {self.camera_intrinsics}')

    def depth_callback(self, msg: Image):
        """深度图回调。"""
        if self.camera_intrinsics is None:
            self.get_logger().warn('等待相机内参...')
            return

        try:
            # 1. 深度图转点云
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            points = depth_to_pointcloud(
                depth_image,
                self.camera_intrinsics,
                depth_scale=0.001,
                max_depth=10.0
            )

            if len(points) < 1000:
                self.get_logger().warn('点云点数过少，跳过处理')
                return

            # 2. 点云滤波
            filtered_points = self.filter_pipeline.filter(points)

            if len(filtered_points) < 50:
                self.get_logger().warn('滤波后点数过少，跳过检测')
                return

            # 3. 密度计算
            use_density = self.config.get('density', {}).get('use_inverse', True)
            if use_density:
                density = self.density_calculator.compute_inverse_density(filtered_points)
            else:
                density = None

            # 4. 3D 目标检测
            if self.detector is not None:
                detections = self.detector.detect(filtered_points, density)

                # 发布检测结果
                if detections:
                    detection_msg = self._create_detection_msg(detections, msg.header)
                    self.detections_pub.publish(detection_msg)
                    self.get_logger().info(f'检测到 {len(detections)} 个目标')

                    # 可视化检测结果
                    if self.enable_viz and self.visualizer is not None:
                        try:
                            self.visualizer.visualize(filtered_points, detections, block=False)
                        except Exception as viz_error:
                            self.get_logger().warn(f'可视化失败: {viz_error}')

            # 5. 发布处理后的点云
            pointcloud_msg = numpy_to_pointcloud2(
                filtered_points,
                frame_id='camera_depth_optical_frame',
                stamp=msg.header.stamp
            )
            self.pointcloud_pub.publish(pointcloud_msg)

        except Exception as e:
            self.get_logger().error(f'感知处理失败: {e}')

    def _create_detection_msg(self, detections: list, header) -> Detection3DArray:
        """创建检测消息。"""
        msg = Detection3DArray()
        msg.header = header
        msg.header.frame_id = 'camera_depth_optical_frame'

        for det in detections:
            detection = Detection3D()

            # 设置边界框
            bbox = det['bbox']  # [x, y, z, dx, dy, dz, yaw]
            detection.bbox.center.position = Point(x=bbox[0], y=bbox[1], z=bbox[2])
            detection.bbox.size = Vector3(x=bbox[3], y=bbox[4], z=bbox[5])

            # 设置朝向（从 yaw 转换为四元数）
            yaw = bbox[6]
            detection.bbox.center.orientation = self._yaw_to_quaternion(yaw)

            # 设置检测结果
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(det['label'])
            hypothesis.hypothesis.score = det['score']
            detection.results.append(hypothesis)

            msg.detections.append(detection)

        return msg

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        """将 yaw 角转换为四元数。"""
        import math
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return Quaternion(x=0.0, y=0.0, z=qz, w=qw)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
