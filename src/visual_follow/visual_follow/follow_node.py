"""视觉跟随协调器 ROS2 节点。"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
import tf2_ros
import yaml
import os
from typing import Optional


class VisualFollowNode(Node):
    """视觉跟随协调器节点：开环控制流程。"""

    def __init__(self):
        super().__init__('visual_follow_node')

        # 加载配置
        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').value

        if config_file and os.path.exists(config_file):
            with open(config_file, 'r') as f:
                self.config = yaml.safe_load(f)
        else:
            self.config = self._get_default_config()

        # TF 监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 目标坐标系
        self.camera_frame = self.config.get('frames', {}).get('camera_frame', 'camera_depth_optical_frame')
        self.base_frame = self.config.get('frames', {}).get('base_frame', 'base_link')

        # MoveIt2 接口（延迟初始化）
        self.moveit_interface = None
        self._init_moveit()

        # 订阅检测结果
        self.detection_sub = self.create_subscription(
            Detection3DArray,
            '/perception/detections',
            self.detection_callback,
            10
        )

        # 控制参数
        self.min_confidence = self.config.get('control', {}).get('min_confidence', 0.5)
        self.target_offset_z = self.config.get('control', {}).get('target_offset_z', 0.1)

        self.get_logger().info('视觉跟随协调器已启动')

    def _get_default_config(self) -> dict:
        """获取默认配置。"""
        return {
            'frames': {
                'camera_frame': 'camera_depth_optical_frame',
                'base_frame': 'base_link',
            },
            'control': {
                'min_confidence': 0.5,
                'target_offset_z': 0.1,  # 目标上方偏移（米）
                'planning_time': 5.0,
                'max_velocity_scaling': 0.3,
                'max_acceleration_scaling': 0.3,
            },
            'moveit': {
                'group_name': 'arm',
                'end_effector_link': 'end_effector',
            }
        }

    def _init_moveit(self):
        """初始化 MoveIt2 接口。"""
        try:
            # 尝试导入 MoveIt2 Python 接口
            # 注意：这里使用简化的接口，实际可能需要 pymoveit2 或其他库
            self.get_logger().info('MoveIt2 接口初始化（简化版本）')
            # TODO: 实际集成 MoveIt2 Python API
            self.moveit_interface = None
        except Exception as e:
            self.get_logger().warn(f'MoveIt2 初始化失败: {e}，使用模拟模式')
            self.moveit_interface = None

    def detection_callback(self, msg: Detection3DArray):
        """检测结果回调。"""
        if len(msg.detections) == 0:
            self.get_logger().info('未检测到目标')
            return

        # 选择置信度最高的检测结果
        best_detection = max(
            msg.detections,
            key=lambda d: d.results[0].hypothesis.score if d.results else 0.0
        )

        if not best_detection.results:
            return

        score = best_detection.results[0].hypothesis.score
        if score < self.min_confidence:
            self.get_logger().info(f'检测置信度过低: {score:.2f}')
            return

        # 提取目标位置
        target_pose_camera = PoseStamped()
        target_pose_camera.header = msg.header
        target_pose_camera.pose.position = best_detection.bbox.center.position
        target_pose_camera.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.get_logger().info(
            f'检测到目标 (置信度: {score:.2f}): '
            f'x={target_pose_camera.pose.position.x:.3f}, '
            f'y={target_pose_camera.pose.position.y:.3f}, '
            f'z={target_pose_camera.pose.position.z:.3f}'
        )

        # 坐标转换到机器人基座坐标系
        target_pose_base = self.transform_to_base_frame(target_pose_camera)

        if target_pose_base is None:
            self.get_logger().warn('坐标转换失败')
            return

        # 添加偏移（目标上方）
        target_pose_base.pose.position.z += self.target_offset_z

        self.get_logger().info(
            f'目标位置（基座坐标系）: '
            f'x={target_pose_base.pose.position.x:.3f}, '
            f'y={target_pose_base.pose.position.y:.3f}, '
            f'z={target_pose_base.pose.position.z:.3f}'
        )

        # 执行运动规划和控制
        self.move_to_target(target_pose_base)

    def transform_to_base_frame(self, pose_stamped: PoseStamped) -> Optional[PoseStamped]:
        """将位姿转换到机器人基座坐标系。"""
        try:
            # 等待 TF 变换可用
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                pose_stamped.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # 执行坐标转换
            pose_transformed = do_transform_pose(pose_stamped.pose, transform)

            result = PoseStamped()
            result.header.frame_id = self.base_frame
            result.header.stamp = self.get_clock().now().to_msg()
            result.pose = pose_transformed

            return result

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'TF 查询失败: {e}')
            return None

    def move_to_target(self, target_pose: PoseStamped):
        """移动到目标位置。"""
        if self.moveit_interface is None:
            self.get_logger().info('MoveIt2 未初始化，模拟运动规划')
            self._simulate_motion(target_pose)
            return

        # TODO: 实际调用 MoveIt2 进行运动规划和执行
        # 1. 设置目标位姿
        # 2. 规划轨迹
        # 3. 执行轨迹
        self.get_logger().info('执行运动规划...')

    def _simulate_motion(self, target_pose: PoseStamped):
        """模拟运动（用于测试）。"""
        self.get_logger().info(
            f'[模拟] 移动到目标位置: '
            f'({target_pose.pose.position.x:.3f}, '
            f'{target_pose.pose.position.y:.3f}, '
            f'{target_pose.pose.position.z:.3f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = VisualFollowNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
