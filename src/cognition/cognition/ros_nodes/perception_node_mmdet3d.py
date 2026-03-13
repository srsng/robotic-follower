# -*- coding: utf-8 -*-
"""
感知节点 - 使用 MMDetection3D API

点云处理 + 3D检测的单节点
"""

import sys
import os

# MMDetection3D 路径
MMD3D_PATH = os.path.expanduser('~/ws/py/mmdetection3d')
sys.path.insert(0, MMD3D_PATH)
sys.path.insert(0, os.path.join(MMD3D_PATH, 'projects/cognition_fusion'))

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
import numpy as np
import torch

# 点云处理工具
from cognition.point_cloud.io import depth_to_pointcloud, numpy_to_pointcloud2
from cognition.point_cloud.filters import VoxelFilter, StatisticalFilter, PassthroughFilter

# MMDetection3D 推理 API
try:
    from mmdet3d.apis import LidarDet3DInferencer
    MMDET3D_AVAILABLE = True
except ImportError as e:
    print(f"MMDetection3D 导入失败: {e}")
    MMDET3D_AVAILABLE = False


class MMDet3DPerceptionNode(Node):
    """感知节点：使用 MMDetection3D 进行 3D 检测"""

    def __init__(self):
        super().__init__('perception_node')

        # 参数
        self.declare_parameter('model_type', 'cognition_fusion')
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('min_depth', 0.0)
        self.declare_parameter('max_depth', 3.0)
        self.declare_parameter('voxel_size', 0.02)

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
        self._cv_bridge = None
        self.inferencer = None
        self.inferencer_initialized = False

        # 初始化滤波器
        voxel_size = self.get_parameter('voxel_size').value
        self.voxel_filter = VoxelFilter(voxel_size=voxel_size)
        self.statistical_filter = StatisticalFilter(nb_neighbors=20, std_ratio=2.0)
        self.pass_filter = PassthroughFilter(axis_name='z', min_limit=0.0, max_limit=3.0)

        self.get_logger().info("MMDetection3D 感知节点已启动")
        self.get_logger().info(f"MMDetection3D 可用: {MMDET3D_AVAILABLE}")
        self.get_logger().info(f"MMDetection3D 路径: {MMD3D_PATH}")

    def depth_callback(self, msg: Image):
        """深度图像回调"""
        if self.camera_info is None:
            self.get_logger().warn("等待相机内参...")
            return

        try:
            # 转换深度图像
            if self._cv_bridge is None:
                try:
                    from cv_bridge import CvBridge
                    self._cv_bridge = CvBridge()
                except ImportError:
                    self.get_logger().warn("cv_bridge 未安装，使用手动转换")

            if self._cv_bridge:
                depth_image = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            else:
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
                return

            self.get_logger().debug(f"点云点数: {len(points)}")

            # 点云处理管道
            processed_points = self._process_pointcloud(points)

            # 发布处理后的点云
            self._publish_pointcloud(processed_points, msg.header)

            # 3D检测
            if MMDET3D_AVAILABLE and self._init_inferencer():
                self._detect_objects(processed_points, msg.header)

        except Exception as e:
            self.get_logger().error(f"处理深度图像失败: {e}")
            import traceback
            traceback.print_exc()

    def camera_info_callback(self, msg: CameraInfo):
        """相机内参回调"""
        self.camera_info = msg
        self.get_logger().info("收到相机内参")

    def _process_pointcloud(self, points: np.ndarray) -> np.ndarray:
        """点云处理管道"""
        # 1. 体素滤波
        points = self.voxel_filter.filter(points)
        self.get_logger().debug(f"体素滤波后点数: {len(points)}")

        # 2. 统计滤波
        points = self.statistical_filter.filter(points)
        self.get_logger().debug(f"统计滤波后点数: {len(points)}")

        # 3. 直通滤波
        min_depth = self.get_parameter('min_depth').value
        max_depth = self.get_parameter('max_depth').value
        self.pass_filter.set_limits(min_depth, max_depth)
        points = self.pass_filter.filter(points)
        self.get_logger().debug(f"直通滤波后点数: {len(points)}")

        return points

    def _init_inferencer(self) -> bool:
        """初始化 MMDetection3D Inferencer"""
        if self.inferencer_initialized:
            return True

        if not MMDET3D_AVAILABLE:
            self.get_logger().error("MMDetection3D 不可用")
            return False

        model_type = self.get_parameter('model_type').value

        # 模型类型映射（使用配置名称，而非完整路径）
        config_map = {
            'cognition_fusion': 'configs/cognition_fusion_scannet.py',
            'votenet': 'configs/votenet_8xb8_scannet-3d.py',
            '3dssd': 'configs/3dssd_3x4_scannet-3d-18class.py',
        }

        config_name = config_map.get(model_type)
        if config_name is None:
            self.get_logger().error(f"未知的模型类型: {model_type}")
            return False

        # 构建配置文件路径
        config_path = os.path.join(MMD3D_PATH, config_name)
        
        if not os.path.exists(config_path):
            self.get_logger().error(f"配置文件不存在: {config_path}")
            return False

        try:
            # 检查 CUDA 可用性
            device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
            self.get_logger().info(f"使用设备: {device}")
            self.get_logger().info(f"加载配置: {config_name}")

            # 初始化 Inferencer
            self.inferencer = LidarDet3DInferencer(model_file=config_path, device=device)
            
            self.inferencer_initialized = True
            self.get_logger().info(f"MMDetection3D Inferencer 初始化成功: {model_type}")
            return True
        except Exception as e:
            self.get_logger().error(f"Inferencer 初始化失败: {e}")
            import traceback
            traceback.print_exc()
            return False

    def _detect_objects(self, points: np.ndarray, header: Header):
        """3D目标检测"""
        try:
            self.get_logger().debug("执行 3D 检测...")

            # LidarDet3DInferencer 期望字典格式，包含 'points' 键
            # 点云格式应为 (N, 4) 或 (N, 3)，其中 4 是 xyz+intensity
            N = points.shape[0]
            if points.shape[1] == 3:
                # 添加 intensity 通道（全零）
                intensity = np.zeros((N, 1), dtype=points.dtype)
                points_mmdet = np.hstack([points, intensity])
            elif points.shape[1] == 4:
                points_mmdet = points
            else:
                # 只取前4列
                points_mmdet = points[:, :4]

            # 执行推理
            inputs = {'points': points_mmdet}
            result = self.inferencer(inputs, show=False)

            # 转换结果为 ROS2 消息
            self._publish_detections(result, header)

        except Exception as e:
            self.get_logger().error(f"检测失败: {e}")
            import traceback
            traceback.print_exc()

    def _publish_pointcloud(self, points: np.ndarray, header: Header):
        """发布处理后的点云"""
        msg = numpy_to_pointcloud2(points, header=header, frame_id='camera_link')
        self.pointcloud_pub.publish(msg)

    def _publish_detections(self, result, header: Header):
        """将 MMDetection3D 结果转换为 ROS2 MarkerArray"""
        marker_array = MarkerArray()
        marker_array.header = header
        marker_array.markers = []

        # MMDetection3D 返回的是 Det3DDataSample 列表
        # Det3DDataSample 包含 pred_instances_3d 属性
        try:
            if hasattr(result, 'pred_instances_3d'):
                pred_instances = result.pred_instances_3d
                
                # 获取 3D 边界框
                if hasattr(pred_instances, 'bboxes_3d'):
                    bboxes = pred_instances.bboxes_3d.tensor  # (N, 7) 格式: [x, y, z, w, h, l, yaw, label]
                    scores = pred_instances.scores if hasattr(pred_instances, 'scores') else None
                    labels = pred_instances.labels if hasattr(pred_instances, 'labels') else None
                    
                    num_detections = len(bboxes)
                    for i in range(num_detections):
                        bbox = bboxes[i]
                        score = float(scores(scores[i]) if scores is not None else 0.5
                        label = int(labels[i]) if labels is not None else 0
                        
                        if score < 0.05:  # 置信度过滤
                            continue
                        
                        marker = Marker()
                        marker.header = header
                        marker.ns = 'detections'
                        marker.id = i
                        marker.type = Marker.CUBE
                        marker.action = Marker.ADD
                        
                        # 设置位置（center）
                        marker.pose.position.x = float(bbox[0])
                        marker.pose.position.y = float(bbox[1])
                        marker.pose.position.z = float(bbox[2])
                        
                        # 设置尺寸
                        marker.scale.x = float(bbox[3])
                        marker.scale.y = float(bbox[4])
                        marker.scale.z = float(bbox[5])
                        
                        # 设置颜色
                        self._set_marker_color(marker, label)
                        
                        # 设置朝向
                        yaw = float(bbox[6])
                        marker.pose.orientation.w = np.cos(yaw / 2)
                        marker.pose.orientation.x = 0.0
                        marker.pose.orientation.y = 0.0
                        marker.pose.orientation.z = np.sin(yaw / 2)
                        
                        marker_array.markers.append(marker)

            self.detections_pub.publish(marker_array)
            self.get_logger().debug(f"发布 {len(marker_array.markers)} 个检测结果")
            
        except Exception as e:
            self.get_logger().error(f"结果转换失败: {e}")
            import traceback
            traceback.print_exc()

    def _set_marker_color(self, marker: Marker, class_id: int):
        """设置标记颜色（基于类别）"""
        # 预定义的颜色
        colors = [
            (1.0, 0.0, 0.0),  # 红色
            (0.0, 1.0, 0.0),  # 绿色
            (0.0, 0.0, 1.0),  # 蓝色
            (1.0, 1.0, 0.0),  # 黄色
            (1.0, 0.0, 1.0),  # 紫色
            (0.0, 1.0, 1.0),  # 青色
            (1.0, 0.5, 0.0),  # 橙色
            (0.5, 0.0, 1.0),  # 零色
        ]
        
        color = colors[class_id % len(colors)]
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 0.7


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    node = MMDet3DPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()
