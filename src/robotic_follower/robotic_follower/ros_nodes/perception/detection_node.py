#!/usr/bin/env python3
"""3D 目标检测节点。

该节点接收处理后的点云，执行 3D 目标检测并发布检测结果。

功能描述：
    - 订阅处理后的点云
    - 可选：将点云从 source_frame 变换到 target_frame
    - 执行 3D 目标检测（VoteNet 等模型）
    - 发布检测结果（边界框、类别、置信度）
    - 从配置文件加载检测器参数

订阅话题：
    - /camera/camera/depth/color/points (sensor_msgs/PointCloud2)
        处理后的点云数据

发布话题：
    - /perception/detections (vision_msgs/Detection3DArray)
        3D 目标检测结果（坐标系：target_frame）
    - /perception/class_names_info (std_msgs/String)
        检测器类别信息（JSON 格式），包含 class_names、idx2class_name、
        class_name2idx、ignore_class_idx、ignore_class_names。
        仅在 detector.ready == True 时首次收到点云后发布一次。

TF 依赖：
    - 当 source_frame != target_frame 时，需要 source_frame → target_frame 的变换路径
    - 由 robot_state_publisher（根据组合 URDF）和 hand_eye_calibration 共同提供

参数：
    - config_file (string, 默认 "model/config/votenet_config.yaml")
        配置文件路径
    - source_frame (string, 默认 "camera_depth_optical_frame")
        点云源坐标系
    - target_frame (string, 默认 "camera_depth_optical_frame")
        点云目标坐标系，默认为源坐标系（即不做变换）

        perception_real.launch.py 中设置为 "base_link" 以变换到基座坐标系

使用示例：
    ros2 run robotic_follower detection_node
    ros2 run robotic_follower detection_node --ros-args -p target_frame:=base_link
"""

import json
import os
import warnings

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Quaternion, Vector3
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection3D, Detection3DArray

from robotic_follower.detection.inference import create_from_config
from robotic_follower.detection.inference.__base__ import Detector
from robotic_follower.point_cloud.io.ros_converters import pointcloud2_to_numpy
from robotic_follower.util.wrapper import NodeWrapper


def filter_warnings():
    """过滤底层库已知的无害警告"""
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


class DetectionNode(NodeWrapper):
    """3D 目标检测节点。"""

    detector: Detector | None

    # DEFAULT_CONFIG = "~/ros2_ws/src/robotic_follower/model/config/votenet_config.yaml"
    # DEFAULT_CONFIG = (
    #     "~/ros2_ws/src/robotic_follower/model/config/density_votenet_config.yaml"
    # )
    DEFAULT_CONFIG = "~/ros2_ws/src/robotic_follower/model/config/ground_cluster.yaml"
    # DEFAULT_CONFIG = "~/ros2_ws/src/robotic_follower/model/config/density_votenet_config_mini.yaml"
    # DEFAULT_CONFIG = "~/ros2_ws/src/robotic_follower/model/config/density_votenet_to_scene-70c.yaml"

    def __init__(self):
        super().__init__("detection_node")
        filter_warnings()
        # TF 初始化
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 参数
        config_file = self.declare_and_get_parameter("config_file", self.DEFAULT_CONFIG)
        config_file = self._resolve_path(config_file)
        self.target_frame = self.declare_and_get_parameter(
            "target_frame", "camera_depth_optical_frame"
        )
        self.source_frame = self.declare_and_get_parameter(
            "source_frame", "camera_depth_optical_frame"
        )

        # 加载配置
        self.detector = None
        if os.path.exists(config_file):
            self._load_from_config(config_file)
        else:
            self._error(f"配置文件不存在: {config_file}")

        # 订阅话题
        pointcloud_topic = self.declare_and_get_parameter(
            "pointcloud_topic", "/camera/camera/depth/color/points"
        )
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
        self.class_names_pub = self.create_publisher(
            String, "/perception/class_names_info", 10
        )

        # 发布 class_names 信息（仅在 detector 就绪后发布一次）
        self._published_class_names = False

        # 无检测器时定时打印错误
        self.error_timer = (
            self.create_timer(2.0, self._print_error_if_no_detector)
            if self.detector is None or not self.detector.ready
            else None
        )

        self._info(f"3D 检测节点已启动，变换目标: {self.target_frame}")

    def _print_error_if_no_detector(self):
        """无检测器时打印错误。"""
        if self.detector is None or not self.detector.ready:
            self._error("检测器未加载，无法执行检测")

    def _publish_class_names_info(self):
        """发布 class_names 信息到话题。"""
        if self.detector is None or not self.detector.ready:
            return

        info = {
            "class_names": list(self.detector.class_names),
            "idx2class_name": {
                str(k): v for k, v in self.detector.idx2class_name.items()
            },
            "class_name2idx": self.detector.class_name2idx,
            "ignore_class_idx": list(self.detector.ignore_class_idx),
            "ignore_class_names": list(self.detector.ignore_class_names),
        }
        msg = String()
        msg.data = json.dumps(info)
        self.class_names_pub.publish(msg)
        self._info("已发布 class_names 信息")

    def _resolve_path(self, path: str) -> str:
        """解析路径为绝对路径。"""
        path = os.path.expanduser(path)

        if os.path.isabs(path):
            return path
        try:
            from ament_index_python.packages import get_package_share_directory

            pkg_path = get_package_share_directory("robotic_follower")
        except Exception:
            pkg_path = os.path.dirname(
                os.path.dirname(
                    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
                )
            )
        return os.path.join(pkg_path, path)

    def _load_from_config(self, config_file: str):
        """从配置文件加载检测器。"""
        import yaml

        with open(config_file) as f:
            config = yaml.safe_load(f)

        detector_config = config.get("detector", {})
        if not detector_config:
            self._error("配置文件中无 detector 配置")
            return

        if "type" not in detector_config:
            self._fatal("配置文件中无 type 定义")
            return

        self.detector = create_from_config(detector_config, parent_node=self)

        if self.detector is not None and self.detector.ready:
            self._info("已加载检测模型")
        else:
            self._error("检测模型未加载")

    def _transform_pointcloud(self, points: np.ndarray, stamp) -> np.ndarray | None:
        """将点云从 source_frame 变换到 target_frame。

        Args:
            points: 点云数据 (N, 3)
            stamp: 点云时间戳

        Returns:
            变换后的点云数据，如果 source_frame == target_frame 则直接返回原始点云
        """
        # 源目标和目标相同时，跳过变换
        if self.source_frame == self.target_frame:
            return points

        try:
            # 查询变换，支持多跳转
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                stamp,
                timeout=rclpy.duration.Duration(seconds=0.5),  # type: ignore
            )
        except Exception as e:
            self._warn(f"TF 变换查询失败: {e}")
            return None

        # 提取旋转矩阵和平移向量
        q = transform.transform.rotation
        t = transform.transform.translation

        # 四元数转旋转矩阵
        q_norm = np.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        q = np.array([q.x, q.y, q.z, q.w]) / q_norm

        # 旋转矩阵 (from tf2)
        R = np.array(
            [
                [
                    1 - 2 * (q[2] ** 2 + q[3] ** 2),
                    2 * (q[1] * q[2] - q[0] * q[3]),
                    2 * (q[1] * q[3] + q[0] * q[2]),
                ],
                [
                    2 * (q[1] * q[2] + q[0] * q[3]),
                    1 - 2 * (q[1] ** 2 + q[3] ** 2),
                    2 * (q[2] * q[3] - q[0] * q[1]),
                ],
                [
                    2 * (q[1] * q[3] - q[0] * q[2]),
                    2 * (q[2] * q[3] + q[0] * q[1]),
                    1 - 2 * (q[1] ** 2 + q[2] ** 2),
                ],
            ]
        )
        translation = np.array([t.x, t.y, t.z])

        # 应用变换
        transformed = (R @ points.T).T + translation
        return transformed

    def pointcloud_callback(self, msg: PointCloud2):
        """点云回调。"""
        self._debug(f"收到点云消息: {msg.width * msg.height} 点")
        if self.detector is None or not self.detector.ready:
            self._warn("检测器未就绪")
            return

        # 发布 class_names 信息（仅在首次就绪时发布一次）
        if not self._published_class_names:
            self._publish_class_names_info()
            self._published_class_names = True

        try:
            points = pointcloud2_to_numpy(msg)
            self._debug(f"收到点云: shape={points.shape}, dtype={points.dtype}")

            # 只取 XYZ（忽略 RGB），mmdet3d VoteNet 只接受 3 通道输入
            if points.shape[1] > 3:
                self._debug(f"提取 XYZ: shape={points.shape} -> (N, 3)")
                points = points[:, :3]

            if len(points) < 100:
                self._warn("点云点数过少，跳过检测")
                return

            # 坐标变换：camera_depth_optical_frame → base_link
            transformed_points = self._transform_pointcloud(points, msg.header.stamp)
            if transformed_points is None:
                self._warn("点云变换失败，跳过本帧")
                return

            detections = self.detector.detect(transformed_points)
            self._debug(f"检测到 {len(detections)} 个目标")
            if detections:
                detection_msg = self._create_detection_msg(detections, msg.header)
                self.detections_pub.publish(detection_msg)
                self._debug("已发布检测结果")

        except Exception as e:
            self._error(f"检测失败: {e}")

    def _create_detection_msg(self, detections: list, header) -> Detection3DArray:
        """创建检测消息。"""
        msg = Detection3DArray()
        msg.header = header
        msg.header.frame_id = self.target_frame

        for det in detections:
            detection = Detection3D()
            bbox = det["bbox"]
            # 所有检测器输出的 bbox 格式均为 [x, y, z, dx, dy, dz, yaw]，z 为几何中心
            detection.bbox.center.position = Point(x=bbox[0], y=bbox[1], z=bbox[2])
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
        node._info("收到中断信号")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
