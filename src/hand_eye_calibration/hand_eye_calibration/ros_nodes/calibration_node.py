"""手眼标定ROS2节点。"""

import sys
import json
import os
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String, Float64MultiArray
from tf2_ros import TransformBroadcaster
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
from hand_eye_calibration.robot import RobotInterface


class CalibrationNode(Node):
    """手眼标定ROS2节点。"""

    def __init__(self):
        super().__init__('hand_eye_calibration_node')

        # 获取启动参数
        self.declare_parameter('auto_load_calibration', False)
        self.declare_parameter('calibration_file', 'results/calibration.yaml')
        self.auto_load = self.get_parameter('auto_load_calibration').value
        self.calibration_file = self.get_parameter('calibration_file').value

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

        # 机器人接口（用于主动控制）
        self.robot_interface = RobotInterface(
            node=self,
            base_frame=self.config.get('robot', {}).get('base_frame', 'base_link'),
            end_effector_frame=self.config.get('robot', {}).get('end_effector_frame', 'link6_1_1'),
            group_name=self.config.get('robot', {}).get('group_name', 'dummy_arm'),
        )

        # 创建话题
        self._create_topics()

        # 创建订阅
        self._create_subscriptions()

        # 创建TF广播器（使用动态TF发布器以支持实时更新）
        self.tf_broadcaster = TransformBroadcaster(self)

        # 当前标定结果（用于周期性发布TF）
        self.current_calibration_result = None

        # TF发布定时器（10Hz）
        self.tf_timer = self.create_timer(0.1, self._publish_calibration_tf)

        # 尝试自动加载标定结果
        if self.auto_load:
            self._try_load_calibration()

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
            'robot': {
                'base_frame': 'base_link',
                'end_effector_frame': 'link6_1_1',
                'group_name': 'dummy_arm',
            },
            'output': {
                'save_path': 'results/calibration.yaml',
                'tf_parent_frame': 'link6_1_1',  # URDF中实际存在的末端执行器坐标系
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

        # 机器人控制（移动到目标位姿）
        self.move_to_pose_sub = self.create_subscription(
            PoseStamped,
            '/hand_eye_calibration/move_to_pose',
            self._move_to_pose_callback,
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
        norm = np.sqrt(x**2 + y**2 + z**2 + w**2)
        if norm == 0:
            return np.eye(3)
        x, y, z, w = x/norm, y/norm, z/norm, w/norm

        xx, yy, zz = x**2, y**2, z**2
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

            # 保存当前标定结果用于周期性发布TF
            self.current_calibration_result = result

        except Exception as e:
            self.get_logger().error(f"执行标定失败: {e}")

        self._publish_status()

    def _reset_callback(self, msg: String):
        """重置标定回调。"""
        success = self.manager.reset()
        if success:
            self.get_logger().info("标定已重置")
            self.current_calibration_result = None  # 清除标定结果
        self._publish_status()

    def _move_to_pose_callback(self, msg: PoseStamped):
        """移动到目标位姿（用于标定）。"""
        try:
            position = [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            ]
            orientation = [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            ]

            # 假设 robot_interface 已在节点中初始化
            # 需要在 __init__ 中创建 robot_interface 实例
            if hasattr(self, 'robot_interface') and self.robot_interface:
                success = self.robot_interface.move_to_pose(position, orientation)
                if success:
                    self.get_logger().info('已移动到目标位姿')
                else:
                    self.get_logger().warn('移动失败')
            else:
                self.get_logger().warn('robot_interface 未初始化')

        except Exception as e:
            self.get_logger().error(f'移动失败: {e}')

    def _publish_status(self):
        """发布标定状态。"""
        status = self.manager.get_status()
        status_str = json.dumps(status)
        msg = String()
        msg.data = status_str
        self.status_pub.publish(msg)

    def _publish_calibration_tf(self):
        """周期性发布手眼变换TF。"""
        if self.current_calibration_result is None:
            return

        result = self.current_calibration_result
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

    def _try_load_calibration(self):
        """尝试加载已保存的标定结果。"""
        if not os.path.exists(self.calibration_file):
            self.get_logger().warn(f"标定文件不存在: {self.calibration_file}")
            return

        try:
            with open(self.calibration_file, 'r', encoding='utf-8') as f:
                result = yaml.safe_load(f)

            # 验证结果格式
            required_keys = ['translation_vector', 'quaternion']
            if not all(key in result for key in required_keys):
                self.get_logger().error("标定文件格式错误")
                return

            # 转换列表为numpy数组
            if isinstance(result['translation_vector'], list):
                result['translation_vector'] = np.array(result['translation_vector'])
            if isinstance(result['quaternion'], list):
                result['quaternion'] = np.array(result['quaternion'])

            self.current_calibration_result = result
            self.get_logger().info(f"已加载标定结果: {self.calibration_file}")
            self.get_logger().info(
                f"手眼变换: t=[{result['translation_vector'][0]:.4f}, "
                f"{result['translation_vector'][1]:.4f}, "
                f"{result['translation_vector'][2]:.4f}]"
            )

        except Exception as e:
            self.get_logger().error(f"加载标定结果失败: {e}")


def main(args=None):
    """主函数。"""
    rclpy.init(args=args)
    node = CalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到中断信号，退出")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
