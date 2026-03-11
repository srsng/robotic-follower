"""
TF 树管理器

提供 TF 树监听和坐标变换查询功能。
支持点、位姿、向量等多种几何元素的坐标转换。
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
    Transform,
    TransformStamped,
    Vector3,
)
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener
from typing import Optional, Tuple, List

from coordinate_transform.config.frame_names import FrameNames, TFTransformTimeout


class TFTreeManager:
    """
    TF 树管理器

    功能：
    - 监听 TF 树
    - 查询坐标系间的变换
    - 转换点、位姿、向量等几何元素
    - 检查变换是否可用
    """

    def __init__(self, node: Node, buffer_time: float = 10.0):
        """
        初始化 TF 树管理器

        Args:
            node: ROS2 节点
            buffer_time: TF 缓冲时间（秒）
        """
        self.node = node
        self._tfbuffer = Buffer()
        self._tf_listener = TransformListener(self._tfbuffer, node)
        self._buffer_time = buffer_time

    # ==================== 基础变换查询 ====================

    def lookup_transform(
        self,
        target_frame: str,
        source_frame: str,
        timeout: float = TFTransformTimeout.DEFAULT_TIMEOUT,
        time: Optional[rclpy.time.Time] = None,
    ) -> TransformStamped:
        """
        查询坐标系变换

        Args:
            target_frame: 目标坐标系
            source_frame: 源坐标系
            timeout: 超时时间（秒）
            time: 变换时间，None 表示最新变换

        Returns:
            TransformStamped: 变换消息

        Raises:
            TransformException: 变换不可用
        """
        try:
            if time is None:
                time = rclpy.time.Time()
            transform = self._tfbuffer.lookup_transform(
                target_frame, source_frame, time, timeout=rclpy.duration.Duration(seconds=timeout)
            )
            return transform
        except TransformException as ex:
            self.node.get_logger().error(
                f"无法获取变换 {source_frame} -> {target_frame}: {ex}"
            )
            raise

    def can_transform(
        self,
        target_frame: str,
        source_frame: str,
        timeout: float = TFTransformTimeout.SHORT_TIMEOUT,
    ) -> bool:
        """
        检查变换是否可用

        Args:
            target_frame: 目标坐标系
            source_frame: 源坐标系
            timeout: 超时时间（秒）

        Returns:
            bool: 变换是否可用
        """
        try:
            self.lookup_transform(target_frame, source_frame, timeout)
            return True
        except TransformException:
            return False

    def wait_for_transform(
        self,
        target_frame: str,
        source_frame: str,
        timeout: float = TFTransformTimeout.LONG_TIMEOUT,
    ) -> bool:
        """
        等待变换可用

        Args:
            target_frame: 目标坐标系
            source_frame: 源坐标系
            timeout: 超时时间（秒）

        Returns:
            bool: 是否成功等待到变换
        """
        start_time = self.node.get_clock().now()
        while (self.node.get_clock().now() - start_time).nanoseconds / 1e9 < timeout:
            if self.can_transform(target_frame, source_frame, timeout=TFTransformTimeout.SHORT_TIMEOUT):
                return True
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return False

    # ==================== 点转换 ====================

    def transform_point(
        self,
        target_frame: str,
        source_frame: str,
        point: Point,
        timeout: float = TFTransformTimeout.DEFAULT_TIMEOUT,
    ) -> Point:
        """
        转换点坐标

        Args:
            target_frame: 目标坐标系
            source_frame: 源坐标系
            point: 源坐标系中的点
            timeout: 超时时间（秒）

        Returns:
            Point: 目标坐标系中的点
        """
        point_stamped = self._create_point_stamped(source_frame, point)
        transform = self.lookup_transform(target_frame, source_frame, timeout)
        return self._do_transform_point(point_stamped, transform)

    def transform_point3d(self, target_frame: str, source_frame: str, point: Tuple[float, float, float],
                         timeout: float = TFTransformTimeout.DEFAULT_TIMEOUT) -> Tuple[float, float, float]:
        """
        转换 3D 点坐标（元组形式）

        Args:
            target_frame: 目标坐标系
            source_frame: 源坐标系
            point: (x, y, z) 元组
            timeout: 超时时间（秒）

        Returns:
            Tuple[float, float, float]: 目标坐标系的 (x, y, z)
        """
        point_msg = Point(x=point[0], y=point[1], z=point[2])
        transformed = self.transform_point(target_frame, source_frame, point_msg, timeout)
        return (transformed.x, transformed.y, transformed.z)

    # ==================== 位姿转换 ====================

    def transform_pose(
        self,
        target_frame: str,
        source_frame: str,
        pose: Pose,
        timeout: float = TFTransformTimeout.DEFAULT_TIMEOUT,
    ) -> Pose:
        """
        转换位姿

        Args:
            target_frame: 目标坐标系
            source_frame: 源坐标系
            pose: 源坐标系中的位姿
            timeout: 超时时间（秒）

        Returns:
            Pose: 目标坐标系中的位姿
        """
        pose_stamped = self._create_pose_stamped(source_frame, pose)
        transform = self.lookup_transform(target_frame, source_frame, timeout)
        return self._do_transform_pose(pose_stamped, transform)

    def transform_pose_stamped(
        self,
        target_frame: str,
        pose_stamped: PoseStamped,
        timeout: float = TFTransformTimeout.DEFAULT_TIMEOUT,
    ) -> PoseStamped:
        """
        转换带时间戳的位姿

        Args:
            target_frame: 目标坐标系
            pose_stamped: 带时间戳的位姿
            timeout: 超时时间（秒）

        Returns:
            PoseStamped: 目标坐标系中的位姿
        """
        transform = self.lookup_transform(target_frame, pose_stamped.header.frame_id, timeout)
        result_pose = self._do_transform_pose(pose_stamped, transform)
        result_stamped = PoseStamped()
        result_stamped.header = transform.header
        result_stamped.pose = result_pose
        return result_stamped

    # ==================== 向量转换 ====================

    def transform_vector(
        self,
        target_frame: str,
        source_frame: str,
        vector: Vector3,
        timeout: float = TFTransformTimeout.DEFAULT_TIMEOUT,
    ) -> Vector3:
        """
        转换向量（只旋转，不改变长度）

        Args:
            target_frame: 目标坐标系
            source_frame: 源坐标系
            vector: 源坐标系中的向量
            timeout: 超时时间（秒）

        Returns:
            Vector3: 目标坐标系中的向量
        """
        transform = self.lookup_transform(target_frame, source_frame, timeout)
        rotation = transform.transform.rotation
        return self._rotate_vector(vector, rotation)

    # ==================== TF 树信息 ====================

    def get_all_frames(self) -> List[str]:
        """
        获取所有可用的坐标系

        Returns:
            List[str]: 坐标系名称列表
        """
        return self._tfbuffer.all_frames_as_string().split("\n")

    def print_tf_tree(self) -> None:
        """打印 TF 树结构"""
        frames = self.get_all_frames()
        self.node.get_logger().info("TF 树结构:")
        for frame in frames:
            if frame.strip():
                self.node.get_logger().info(f"  {frame}")

    # ==================== 私有辅助方法 ====================

    def _create_point_stamped(self, frame_id: str, point: Point) -> PoseStamped:
        """创建带时间戳的点（用 PoseStamped 表示）"""
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.header.stamp = self.node.get_clock().now().to_msg()
        pose_stamped.pose.position = point
        pose_stamped.pose.orientation.w = 1.0
        return pose_stamped

    def _create_pose_stamped(self, frame_id: str, pose: Pose) -> PoseStamped:
        """创建带时间戳的位姿"""
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.header.stamp = self.node.get_clock().now().to_msg()
        pose_stamped.pose = pose
        return pose_stamped

    def _do_transform_point(self, point_stamped: PoseStamped, transform: TransformStamped) -> Point:
        """执行点变换"""
        p = point_stamped.pose.position
        t = transform.transform.translation
        q = transform.transform.rotation

        # 应用旋转和平移
        rotated = self._rotate_point(p, q)
        return Point(x=rotated.x + t.x, y=rotated.y + t.y, z=rotated.z + t.z)

    def _do_transform_pose(self, pose_stamped: PoseStamped, transform: TransformStamped) -> Pose:
        """执行位姿变换"""
        position = self._do_transform_point(pose_stamped, transform)
        orientation = self._multiply_quaternions(
            transform.transform.rotation, pose_stamped.pose.orientation
        )
        return Pose(position=position, orientation=orientation)

    def _rotate_point(self, point: Point, quaternion: Quaternion) -> Point:
        """用四元数旋转点"""
        # 四元数: q = (w, x, y, z)
        w, x, y, z = quaternion.w, quaternion.x, quaternion.y, quaternion.z

        # 旋转公式: p' = q * p * q^-1
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z

        px = point.x
        py = point.y
        pz = point.z

        # 旋转后的坐标
        rx = (1 - 2 * (yy + zz)) * px + (2 * (xy - wz)) * py + (2 * (xz + wy)) * pz
        ry = (2 * (xy + wz)) * px + (1 - 2 * (xx + zz)) * py + (2 * (yz - wx)) * pz
        rz = (2 * (xz - wy)) * px + (2 * (yz + wx)) * py + (1 - 2 * (xx + yy)) * pz

        return Point(x=rx, y=ry, z=rz)

    def _rotate_vector(self, vector: Vector3, quaternion: Quaternion) -> Vector3:
        """用四元数旋转向量量"""
        point = Point(x=vector.x, y=vector.y, z=vector.z)
        rotated = self._rotate_point(point, quaternion)
        return Vector3(x=rotated.x, y=rotated.y, z=rotated.z)

    def _multiply_quaternions(self, q1: Quaternion, q2: Quaternion) -> Quaternion:
        """四元数乘法: q = q1 * q2"""
        w1, x1, y1, z1 = q1.w, q1.x, q1.y, q1.z
        w2, x2, y2, z2 = q2.w, q2.x, q2.y, q2.z

        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

        # 归一化
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        if norm > 0:
            w, x, y, z = w / norm, x / norm, y / norm, z / norm

        return Quaternion(w=w, x=x, y=y, z=z)

    @staticmethod
    def quaternion_to_euler(q: Quaternion) -> Tuple[float, float, float]:
        """
        四元数转欧拉角（ZYX 顺序）

        Args:
            q: 四元数

        Returns:
            Tuple[float, float, float]: (roll, pitch, yaw) 弧度
        """
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return (roll, pitch, yaw)

    @staticmethod
    def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
        """
        欧拉角转四元数（ZYX 顺序）

        Args:
            roll: 绕 X 轴旋转（弧度）
            pitch: 绕 Y 轴旋转（弧度）
            yaw: 绕 Z 轴旋转（弧度）

        Returns:
            Quaternion: 四元数
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return Quaternion(w=w, x=x, y=y, z=z)
