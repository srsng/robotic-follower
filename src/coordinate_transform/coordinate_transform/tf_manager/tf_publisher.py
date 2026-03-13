"""
TF发布器，负责静态和动态变换
"""
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
import yaml
import os


class TFPublisher:
    """TF发布器，负责静态和动态变换"""

    def __init__(self, node: Node):
        self.node = node
        self.static_broadcaster = StaticTransformBroadcaster(node)
        self.dynamic_broadcaster = TransformBroadcaster(node)

    @staticmethod
    def create_transform_stamped(parent: str, child: str,
                                translation: list, rotation: list,
                                time: Time = None) -> TransformStamped:
        """
        创建变换消息

        Args:
            parent: 父坐标系
            child: 子坐标系
            translation: 平移 [x, y, z]
            rotation: 旋转 [x, y, z, w] (四元数)
            time: 时间戳

        Returns:
            TransformStamped
        """
        transform = TransformStamped()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.header.stamp = time if time else Time()

        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]

        transform.transform.rotation.x = rotation[0]
        transform.transform.rotation.y = rotation[1]
        transform.transform.rotation.z = rotation[2]
        transform.transform.rotation.w = rotation[3]

        return transform

    def publish_static_transform(self, parent: str, child: str,
                               translation: list, rotation: list):
        """发布静态变换"""
        transform = self.create_transform_stamped(
            parent, child, translation, rotation
        )
        self.static_broadcaster.sendTransform(transform)

    def publish_dynamic_transform(self, transform: TransformStamped):
        """发布动态变换"""
        self.dynamic_broadcaster.sendTransform(transform)

    def load_and_publish_hand_eye_transform(self, calibration_file: str,
                                           parent_frame: str = "end_effector",
                                           child_frame: str = "camera_link"):
        """
        从手眼标定结果文件加载并发布手眼变换

        Args:
            calibration_file: 标定结果文件路径
            parent_frame: 父坐标系名称
            child_frame: 子坐标系名称

        Returns:
            bool: 是否成功加载和发布
        """
        # 展开路径中的 ~
        calibration_file = os.path.expanduser(calibration_file)

        if not os.path.exists(calibration_file):
            self.node.get_logger().error(
                f"手眼标定文件不存在: {calibration_file}"
            )
            return False

        try:
            with open(calibration_file, 'r') as f:
                calib_data = yaml.safe_load(f)

            # 从标定结果中提取变换
            translation = calib_data.get('translation_vector', [0.0, 0.0, 0.0])
            quaternion = calib_data.get('quaternion', [0.0, 0.0, 0.0, 1.0])

            # 发布静态TF
            self.publish_static_transform(
                parent_frame, child_frame, translation, quaternion
            )

            self.node.get_logger().info(
                f"成功加载并发布手眼变换: {parent_frame} -> {child_frame}"
            )
            return True

        except Exception as e:
            self.node.get_logger().error(
                f"加载手眼标定文件失败: {e}"
            )
            return False
