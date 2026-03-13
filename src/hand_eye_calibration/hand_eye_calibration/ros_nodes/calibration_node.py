"""手眼标定ROS2节点。"""

import sys
import json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String, Float64MultiArray
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3, PoseStamped
from cv_bridge import CvBridge

# 导入内部组件
from hand_eye_calibration.calibration import (
    CalibrationManager,
    CalibrationConfig,
)
from hand_eye_calibration.camera import BoardDetector
from hand_eye_calibration.calibration import (
    ExtrinsicCalibrator,
    CalibrationValidator,
)


class CalibrationNode(Node):
    """手眼标定ROS2节点。"""

    def __init__(self):
        super().__init__('hand_eye_calibration_node')

        # 加载配置
        self.config = self._get_default_config()

        # 创建标定管理器
        manager_config = CalibrationConfig(
            board_type=self.config['calibration']['board_type'],
            board_cols=self.config['calibration']['board_cols'],
            board_rows=self.config['calibration']['board_rows'],
            circle_diameter=self.config['calibration']['circle_diameter'],
            min_samples=self.config['calibration']['min_samples'],
            max_samples=self.config['calibration']['max_samples'],
        )
        self.manager = CalibrationManager(manager_config, self)

        # 标定器
        self.calibrator = ExtrinsicCalibrator()
        self.manager.calibrator = self.calibrator

        # 验证器
        self.validator = CalibrationValidator(
            max_error=self.config['validation']['max_error'],
        )
        self.manager.validator = self.validator

        # 标定板检测器
        self.board_detector = BoardDetector(
            board_type=self.config['calibration']['board_type'],
            board_cols=self.config['calibration']['board_cols'],
            board_rows=self.config['calibration']['board_rows'],
            circle_diameter=self.config['calibration']['circle_diameter'],
        )

        # CV桥接
        self.bridge = CvBridge()

        # 状态变量
        self.camera_matrix = None
        self.dist_coeffs = None
        self.current_image = None
        self.current_robot_pose = None

        # 创建话题
        self._create_topics()

        # 创建订阅
        self._create_subscriptions()

        # 创建TF广播器
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.dynamic_tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("手眼标定节点已启动")
        self.get_logger().info("等待相机图像和机器人位姿...")

    def _get_default_config(self) -> dict:
        """获取默认配置。"""
        return {
            'calibration': {
                'board_type': 'circles_asymmetric',
                'board_cols': 4,
                'board_rows': 5,
                'circle_diameter': 0.020,
                'min_samples': 15,
                'max_samples': 50,
            },
            'validation': {
                'max_error': 0.01,
            },
            'output': {
                'save_path': 'results/calibration.yaml',
                'tf_parent_frame': 'end_effector',
                'tf_child_frame': 'camera_link',
            },
        }

    def _create_topics(self):
        """创建发布话题。"""
        # 标定状态
        self.status_pub = self.create_publisher(
            String,
            '/hand_eye_calibration/status',
            10
        )

        # 标定结果
        self.result_pub = self.create_publisher(
            Float64MultiArray,
            '/hand_eye_calibration/result',
            10
        )

    def _create_subscriptions(self):
        """创建订阅话题。"""
        # 相机图像
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self._image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # 相机信息
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self._camera_info_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # 机器人位姿
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot/pose',
            self._pose_callback,
            10
        )

        # 添加标定样本
        self.add_sample_sub = self.create_subscription(
            String,
            '/hand_eye_calibration/add_sample',
            self._add_sample_callback,
            10
        )

        # 执行标定
        self.execute_sub = self.create_subscription(
            String,
            '/hand_eye_calibration/execute',
            self._execute_callback,
            10
        )

        # 重置标定
        self.reset_sub = self.create_subscription(
            String,
            '/hand_eye_calibration/reset',
            self._reset_callback,
            10
        )

    def _camera_info_callback(self, msg: CameraInfo):
        """相机信息回调。"""
        # 提取相机内参
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        self.get_logger().info("收到相机内参")

    def _image_callback(self, msg: Image):
        """图像回调。"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {e}")

    def _pose_callback(self, msg: PoseStamped):
        """位姿回调。"""
        # 从ROS位姿转换为4x4齐次变换矩阵
        pos = msg.pose.position
        orient = msg.pose.orientation

        # 平移向量
        t = np.array([pos.x, pos.y, pos.z])

        # 四元数转旋转矩阵
        R = self._quaternion_to_matrix(orient.x, orient.y, orient.z, orient.w)

        # 构建4x4齐次变换矩阵
        pose = np.eye(4)
        pose[:3, :3] = R
        pose[:3, 3] = t

        self.current_robot_pose = pose

    def _quaternion_to_matrix(
        self,
        x: float,
        y: float,
        z: float,
        w: float
    ) -> np.ndarray:
        """四元数转旋转矩阵。"""
        # 归一化四元数
        norm = np.sqrt(x*x + y*y + z*z + w*w)
        if norm == 0:
            return np.eye(3)
        x, y, z, w = x/norm, y/norm, z/norm, w/norm

        xx, yy, zz = x*x, y*y, z*z
        xy, xz, yz = x*y, x*z, y*z
        xw, yw, zw = x*w, y*w, z*w

        R = np.array([
            [1 - 2*(yy + zz), 2*(xy - zw), 2*(xz + yw)],
            [2*(xy + zw), 1 - 2*(xx + zz), 2*(yz - xw)],
            [2*(xz - yw), 2*(yz + xw), 1 - 2*(xx + yy)]
        ])

        return R

    def _add_sample_callback(self, msg: String):
        """添加标定样本回调。"""
        if self.current_robot_pose is None:
            self.get_logger().warn("未收到机器人位姿")
            return

        if self.current_image is None:
            self.get_logger().warn("未收到相机图像")
            return

        if self.camera_matrix is None or self.dist_coeffs is None:
            self.get_logger().warn("相机内参未收到")
            return

        # 如果处于空闲状态，自动开始收集
        status = self.manager.get_status()
        if status['state'] == 'idle':
            self.manager.start_calibration()

        # 检测标定板
        camera_pose = self.board_detector.detect_with_camera_matrix(
            self.current_image,
            self.camera_matrix,
            self.dist_coeffs
        )

        if camera_pose is None:
            self.get_logger().warn("标定板检测失败")
            return

        # 添加样本
        success = self.manager.add_sample(
            self.current_robot_pose,
            self.current_image,
            camera_pose
        )

        if success:
            status = self.manager.get_status()
            self.get_logger().info(
                f"已添加第 {status['sample_count']} 个样本 "
                f"(需要至少 {status['min_samples']} 个)"
            )
        else:
            self.get_logger().warn("添加样本失败")

        # 发布状态
        self._publish_status()

    def _execute_callback(self, msg: String):
        """执行标定回调。"""
        try:
            result = self.manager.execute_calibration()

            # 发布手眼变换TF
            self._publish_transform(result)

            # 保存结果
            save_path = self.config['output']['save_path']
            self.manager.save_results(save_path)

            # 发布结果数据
            result_msg = Float64MultiArray()
            result_msg.data = [
                result['error'],
                result['translation_vector'][0],
                result['translation_vector'][1],
                result['translation_vector'][2],
                result['quaternion'][0],
                result['quaternion'][1],
                result['quaternion'][2],
                result['quaternion'][3],
            ]
            self.result_pub.publish(result_msg)

            self.get_logger().info(f"标定完成，误差: {result['error']:.6f}m")
            self.get_logger().info(f"标定结果已保存到 {save_path}")

        except Exception as e:
            self.get_logger().error(f"执行标定失败: {e}")

        self._publish_status()

    def _reset_callback(self, msg: String):
        """重置标定回调。"""
        success = self.manager.reset()
        if success:
            self.get_logger().info("标定已重置")
        self._publish_status()

    def _publish_status(self):
        """发布标定状态。"""
        status = self.manager.get_status()
        status_str = json.dumps(status)
        msg = String()
        msg.data = status_str
        self.status_pub.publish(msg)

    def _publish_transform(self, result: dict):
        """发布手眼变换TF。"""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.config['output']['tf_parent_frame']
        transform.child_frame_id = self.config['output']['tf_child_frame']

        transform.transform.translation.x = result['translation_vector'][0]
        transform.transform.translation.y = result['translation_vector'][1]
        transform.transform.translation.z = result['translation_vector'][2]

        transform.transform.rotation.x = result['quaternion'][0]
        transform.transform.rotation.y = result['quaternion'][1]
        transform.transform.rotation.z = result['quaternion'][2]
        transform.transform.rotation.w = result['quaternion'][3]

        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().info("已发布手眼变换TF")


def main(args=None):
    """主函数。"""
    rclpy.init(args=args)
    node = CalibrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
