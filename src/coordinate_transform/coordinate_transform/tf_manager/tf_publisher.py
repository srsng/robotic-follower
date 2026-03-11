"""
TF 发布器

提供静态和动态 TF 变换的发布功能。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Pose
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from typing import List, Tuple, Optional
import threading

from coordinate_transform.config.frame_names import FrameNames


class TFPublisher:
    """
    TF 发布器

    功能：
    - 发布静态 TF 变换
    - 发布动态 TF 变换
    - 支持定时更新
    """

    def __init__(self, node: Node):
        """
        初始化 TF 发布器

        Args:
            node: ROS2 节点
        """
        self.node = node
        self._static_broadcaster = StaticTransformBroadcaster(node)
        self._dynamic_broadcaster = TransformBroadcaster(node)
        self._timer = None
        self._dynamic_transforms = {}
        self._lock = threading.Lock()

    # ==================== 韦态变换 ====================

    def publish_static_transform(self, transform: TransformStamped) -> None:
        """
        发布静态变换（只发布一次）

        Args:
            transform: 变换消息
        """
        self._static_broadcaster.sendTransform(transform)
        self.node.get_logger().info(
            f"发布静态 TF: {transform.header.frame_id} -> {transform.child_frame_id}"
        )

    def publish_static_transforms(self, transforms: List[TransformStamped]) -> None:
        """
        批量发布静态变换

        Args:
            transforms: 变换消息列表
        """
        self._static_broadcaster.sendTransform(transforms)
        self.node.get_logger().info(f"发布 {len(transforms)} 个静态 TF 变换")

    @staticmethod
    def create_static_transform(
        parent_frame: str,
        child_frame: str,
        translation: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        rotation: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),  # (x, y, z, w)
    ) -> TransformStamped:
        """
        创建静态变换

        Args:
            parent_frame: 父坐标系
            child_frame: 子坐标系
            translation: 平移 (x, y, z)
            rotation: 四元数 (x, y, z, w)

        Returns:
            TransformStamped: 变换消息
        """
        transform = TransformStamped()
        transform.header.stamp = rclpy.time.Time().to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame

        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]

        transform.transform.rotation.x = rotation[0]
        transform.transform.rotation.y = rotation[1]
        transform.transform.rotation.z = rotation[2]
        transform.transform.rotation.w = rotation[3]

        return transform

    @staticmethod
    def create_static_transform_from_pose(
        parent_frame: str,
        child_frame: str,
        pose: Pose,
    ) -> TransformStamped:
        """
        从位姿创建静态变换

        Args:
            parent_frame: 父坐标系
            child_frame: 子坐标系
            pose: 位姿

        Returns:
            TransformStamped: 变换消息
        """
        transform = TransformStamped()
        transform.header.stamp = rclpy.time.Time().to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation = pose.position
        transform.transform.rotation = pose.orientation
        return transform

    # ==================== 动态变换 ====================

    def update_dynamic_transform(self, transform: TransformStamped) -> None:
        """
        更新动态变换

        Args:
            transform: 变换消息
        """
        with self._lock:
            self._dynamic_transforms[transform.child_frame_id] = transform

    def remove_dynamic_transform(self, child_frame: str) -> None:
        """
        移除动态变换

        Args:
            child_frame: 子坐标系名称
        """
        with self._lock:
            if child_frame in self._dynamic_transforms:
                del self._dynamic_transforms[child_frame]
                self.node.get_logger().info(f"移除动态 TF: {child_frame}")

    def start_dynamic_publishing(self, period: float = 0.1) -> None:
        """
        启动动态变换定时发布

        Args:
            period: 发布周期（秒）
        """
        if self._timer is not None:
            self.node.get_logger().warn("动态发布已启动，无需重复启动")
            return

        self._timer = self.node.create_timer(period, self._publish_dynamic_transforms)
        self.node.get_logger().info(f"启动动态 TF 发布，周期: {period}s")

    def stop_dynamic_publishing(self) -> None:
        """停止动态变换发布"""
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
            self.node.get_logger().info("停止动态 TF 发布")

    def _publish_dynamic_transforms(self) -> None:
        """发布所有动态变换"""
        with self._lock:
            transforms = list(self._dynamic_transforms.values())

        if transforms:
            self._dynamic_broadcaster.sendTransform(transforms)

    # ==================== 工具坐标系创建 ====================

    def create_tool_frame(
        parent_frame: str = FrameNames.LINK6,
        tool_frame: str = FrameNames.TOOL_FRAME,
        tcp_offset: Tuple[float, float, float] = (0.0, 0.0, 0.1),
    ) -> TransformStamped:
        """
        创建工具坐标系（TCP）

        Args:
            parent_frame: 父坐标系（通常是末端执行器）
            tool_frame: 工具坐标系名称
            tcp_offset: TCP 偏移量 (x, y, z) 米

        Returns:
            TransformStamped: 工具坐标系变换
        """
        return self.create_static_transform(
            parent_frame=parent_frame,
            child_frame=tool_frame,
            translation=tcp_offset,
        )

    def create_gripper_frame(
        parent_frame: str = FrameNames.LINK6,
        gripper_frame: str = FrameNames.GRIPPER_FRAME,
        gripper_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> TransformStamped:
        """
        创建夹爪坐标系

        Args:
            parent_frame: 父坐标系
            gripper_frame: 夹爪坐标系名称
            gripper_offset: 夹爪偏移量 (x, y, z) 米

        Returns:
            TransformStamped: 夹爪坐标系变换
        """
        return self.create_static_transform(
            parent_frame=parent_frame,
            child_frame=gripper_frame,
            translation=gripper_offset,
        )

    # ==================== 手眼标定相关的 TF 发布 ====================

    def publish_hand_eye_transform(
        parent_frame: str = FrameNames.LINK6,
        camera_frame: str = FrameNames.D435_COLOR_OPTICAL,
        hand_eye_transform: Optional[TransformStamped] = None,
    ) -> None:
        """
        发布手眼标定变换

        注意：这是静态变换，由手眼标定结果确定

        Args:
            parent_frame: 机械臂末端坐标系
            camera_frame: 相机光学坐标系
            hand_eye_transform: 手眼变换（如果为从标定文件加载）
        """
        if hand_eye_transform is None:
            # 默认无偏移
            transform = self.create_static_transform(
                parent_frame=parent_frame,
                child_frame=camera_frame,
            )
        else:
            transform = hand_eye_transform
            transform.header.frame_id = parent_frame
            transform.child_frame_id = camera_frame

        self.publish_static_transform(transform)
        self.node.get_logger().info(f"发布手眼变换: {parent_frame} -> {camera_frame}")
