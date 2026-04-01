#!/usr/bin/env python3
"""3D 目标检测节点。

该节点接收处理后的点云，执行 3D 目标检测并发布检测结果。

功能描述：
    - 订阅处理后的点云
    - 执行 3D 目标检测（VoteNet 等模型）
    - 发布检测结果（边界框、类别、置信度）
    - 从配置文件加载检测器参数

订阅话题：
    - /camera/camera/depth/color/points (sensor_msgs/PointCloud2)
        处理后的点云数据

发布话题：
    - /perception/detections (vision_msgs/Detection3DArray)
        3D 目标检测结果

参数：
    - config_file (string, 默认 "model/config/votenet_config.yaml")
        配置文件路径

使用示例：
    ros2 run robotic_follower detection_node
    ros2 run robotic_follower detection_node --ros-args -p config_file:=/path/to/config.yaml
"""

import os
import warnings

import rclpy
from geometry_msgs.msg import Point, Quaternion, Vector3
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import Detection3D, Detection3DArray

from robotic_follower.detection.inference.detector import create_detector_from_config
from robotic_follower.point_cloud.io.ros_converters import pointcloud2_to_numpy


class DetectionNode(Node):
    """3D 目标检测节点。"""

    DEFAULT_CONFIG = "~/ros2_ws/install/robotic_follower/share/robotic_follower/model/config/votenet_config.yaml"

    def __init__(self):
        super().__init__("detection_node")

        # 过滤底层库已知的无害警告
        warnings.filterwarnings(
            "ignore", message="Unable to import Axes3D", category=UserWarning
        )
        warnings.filterwarnings(
            "ignore",
            message="Unnecessary conv bias before batch/instance norm",
            category=UserWarning,
        )
        warnings.filterwarnings(
            "ignore",
            message="The torch.cuda.*DtypeTensor constructors are no longer recommended",
            category=UserWarning,
        )

        # 获取包路径
        from ament_index_python.packages import get_package_share_directory

        try:
            self.package_path = get_package_share_directory("robotic_follower")
        except Exception:
            self.package_path = os.path.dirname(
                os.path.dirname(
                    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
                )
            )

        # 参数
        self.declare_parameter("config_file", self.DEFAULT_CONFIG)

        config_file = os.path.expanduser(self.get_parameter("config_file").value)
        config_file = self._resolve_path(config_file)

        # 加载配置
        self.detector = None
        if os.path.exists(config_file):
            self._load_from_config(config_file)
        else:
            self.get_logger().error(f"配置文件不存在: {config_file}")

        # 订阅话题
        self.declare_parameter("pointcloud_topic", "/camera/camera/depth/color/points")
        pointcloud_topic = str(self.get_parameter("pointcloud_topic").value)
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            pointcloud_topic,
            self.pointcloud_callback,
            10,
        )

        # 发布话题
        self.detections_pub = self.create_publisher(
            Detection3DArray, "/perception/detections", 10
        )

        # 无检测器时定时打印错误
        self.error_timer = (
            self.create_timer(1.0, self._print_error_if_no_detector)
            if self.detector is None or self.detector.model is None
            else None
        )

        self.get_logger().info("3D 检测节点已启动")

    def _print_error_if_no_detector(self):
        """无检测器时打印错误。"""
        if self.detector is None or self.detector.model is None:
            self.get_logger().error("检测器未加载，无法执行检测")

    def _resolve_path(self, path: str) -> str:
        """解析路径为绝对路径。"""
        if os.path.isabs(path):
            return path
        return os.path.join(self.package_path, path)

    def _load_from_config(self, config_file: str):
        """从配置文件加载检测器。"""
        import yaml

        with open(config_file) as f:
            config = yaml.safe_load(f)

        detector_config = config.get("detector", {})
        if not detector_config:
            self.get_logger().error("配置文件中无 detector 配置")
            return

        detector_config["config_file"] = os.path.expanduser(
            detector_config["config_file"]
        )
        detector_config["checkpoint_file"] = os.path.expanduser(
            detector_config["checkpoint_file"]
        )

        self.get_logger().info(f"config_file: {detector_config['config_file']}")
        self.get_logger().info(f"checkpoint_file: {detector_config['checkpoint_file']}")

        self.detector = create_detector_from_config(detector_config)
        if self.detector and self.detector.model is not None:
            self.get_logger().info("已加载检测模型")
        else:
            self.get_logger().error("检测模型未加载")

    def pointcloud_callback(self, msg: PointCloud2):
        """点云回调。"""
        self.get_logger().debug(f"收到点云消息: {msg.width * msg.height} 点")
        if self.detector is None or self.detector.model is None:
            self.get_logger().warn("检测器未就绪")
            return

        try:
            points = pointcloud2_to_numpy(msg)
            self.get_logger().debug(f"收到点云: shape={points.shape}, dtype={points.dtype}")

            # 只取 XYZ（忽略 RGB），mmdet3d VoteNet 只接受 3 通道输入
            if points.shape[1] > 3:
                points = points[:, :3]
                self.get_logger().debug(f"提取 XYZ: shape={points.shape}")

            if len(points) < 100:
                self.get_logger().warn("点云点数过少，跳过检测")
                return

            detections = self.detector.detect(points)
            self.get_logger().debug(f"检测到 {len(detections)} 个目标")
            if detections:
                detection_msg = self._create_detection_msg(detections, msg.header)
                self.detections_pub.publish(detection_msg)
                self.get_logger().debug("已发布检测结果")

        except Exception as e:
            self.get_logger().error(f"检测失败: {e}")

    def _create_detection_msg(self, detections: list, header) -> Detection3DArray:
        """创建检测消息。"""
        msg = Detection3DArray()
        msg.header = header
        msg.header.frame_id = "camera_depth_optical_frame"

        for det in detections:
            detection = Detection3D()
            bbox = det["bbox"]
            # 修复：mmdet3d 中 SUNRGBD 数据集的 z 坐标是底部中心，而 ROS2 Detection3D 和 Open3D 期望几何中心
            # 因此需要将 z 加上高度的一半 (dz / 2)
            detection.bbox.center.position = Point(x=bbox[0], y=bbox[1], z=bbox[2] + bbox[5] / 2.0)
            detection.bbox.size = Vector3(x=bbox[3], y=bbox[4], z=bbox[5])
            detection.bbox.center.orientation = self._yaw_to_quaternion(bbox[6])

            from vision_msgs.msg import ObjectHypothesisWithPose

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(det["label"])
            hypothesis.hypothesis.score = det["score"]
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
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到中断信号")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
