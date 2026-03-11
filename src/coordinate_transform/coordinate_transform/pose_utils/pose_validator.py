"""
位姿验证器

提供关节位置、位姿在工作空间内、可达性等验证功能。
"""

import math
import numpy as np
from typing import List, Tuple, Optional
from geometry_msgs.msg import Pose, Point, Quaternion


class PoseValidator:
    """
    位姿验证器

    功能：
    - 验证关节位置是否在限制范围内
    - 验证位姿是否在工作空间内
    - 验证四元数是否有效
    - 计算位姿之间的距离
    """

    @staticmethod
    def validate_joint_positions(
        joints: List[float],
        limits: List[Tuple[float, float]],
        tolerance: float = 1e-6,
    ) -> Tuple[bool, int]:
        """
        验证关节位置是否在限制范围内

        Args:
            joints: 关节位置列表
            limits: 关节限制列表，每个元素为 (min, max)
            tolerance: 容差

        Returns:
            Tuple[bool, int]: (是否有效, 违反限制的关节索引，-1 表示全部有效)
        """
        if len(joints) != len(limits):
            return (False, -1)

        for i, (joint, (min_val, max_val)) in enumerate(zip(joints, limits)):
            if joint < min_val - tolerance or joint > max_val + tolerance:
                return (False, i)

        return (True, -1)

    @staticmethod
    def validate_pose_within_workspace(
        pose: Pose,
        workspace_bounds: Optional[dict] = None,
    ) -> bool:
        """
        验证位姿是否在工作空间内

        Args:
            pose: 待验证的位姿
            workspace_bounds:工作空间边界，格式：
                {
                    'x': (min_x, max_x),
                    'y': (min_y, max_y),
                    'z': (min_z, max_z)
                }

        Returns:
            bool: 位姿是否在工作空间内
        """
        if workspace_bounds is None:
            # 默认工作空间范围（可根据机械臂规格调整）
            workspace_bounds = {
                'x': (-1.0, 1.0),
                'y': (-1.0, 1.0),
                'z': (0.0, 1.5),
            }

        x, y, z = pose.position.x, pose.position.y, pose.position.z

        if not (workspace_bounds['x'][0] <= x <= workspace_bounds['x'][1]):
            return False
        if not (workspace_bounds['y'][0] <= y <= workspace_bounds['y'][1]):
            return False
        if not (workspace_bounds['z'][0] <= z <= workspace_bounds['z'][1]):
            return False

        return True

    @staticmethod
    def validate_quaternion(quaternion: Quaternion, tolerance: float = 1e-6) -> bool:
        """
        验证四元数是否有效（归一化）

        Args:
            quaternion: 四元数
            tolerance: 归一化容差

        Returns:
            bool: 四元数是否有效
        """
        norm = math.sqrt(
            quaternion.x ** 2 +
            quaternion.y ** 2 +
            quaternion.z ** 2 +
            quaternion.w ** 2
        )

        return abs(norm - 1.0) < tolerance

    @staticmethod
    def normalize_quaternion(quaternion: Quaternion) -> Quaternion:
        """
        归一化四元数

        Args:
            quaternion: 四元数

        Returns:
            Quaternion: 归一化后的四元数
        """
        norm = math.sqrt(
            quaternion.x ** 2 +
            quaternion.y ** 2 +
            quaternion.z ** 2 +
            quaternion.w ** 2
        )

        if norm < 1e-10:
            return Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)

        return Quaternion(
            w=quaternion.w / norm,
            x=quaternion.x / norm,
            y=quaternion.y / norm,
            z=quaternion.z / norm,
        )

    @staticmethod
    def calculate_position_distance(pose1: Pose, pose2: Pose) -> float:
        """
        计算两个位姿之间的位置距离

        Args:
            pose1: 位姿 1
            pose2: 位姿 2

        Returns:
            float: 欧氏距离（米）
        """
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        dz = pose1.position.z - pose2.position.z

        return math.sqrt(dx * dx + dy * dy + dz * dz)

    @staticmethod
    def calculate_orientation_distance(
        pose1: Pose,
        pose2: Pose,
        method: str = 'quaternion',
    ) -> float:
        """
        计算两个位姿之间的姿态距离

        Args:
            pose1: 位姿 1
            pose2: 位姿 2
            method: 计算方法，'quaternion' 或 'euler'

        Returns:
            float: 姿态距离（弧度）
        """
        q1 = np.array([
            pose1.orientation.w,
            pose1.orientation.x,
            pose1.orientation.y,
            pose1.orientation.z,
        ])
        q2 = np.array([
            pose2.orientation.w,
            pose2.orientation.x,
            pose2.orientation.y,
            pose2.orientation.z,
        ])

        if method == 'quaternion':
            # 计算四元数距离（旋转角度）
            dot = np.abs(np.dot(q1, q2))
            dot = min(dot, 1.0)
            return 2 * math.acos(dot)

        elif method == 'euler':
            # 欧拉角距离
            euler1 = PoseValidator.quaternion_to_euler(pose1.orientation)
            euler2 = PoseValidator.quaternion_to_euler(pose2.orientation)

            diff = sum(abs(a - b) for a, b in zip(euler1, euler2))
            return diff

        return 0.0

    @staticmethod
    def quaternion_to_euler(
        quaternion: Quaternion,
        order: str = 'zyx',
    ) -> Tuple[float, float, float]:
        """
        四元数转欧拉角

        Args:
            quaternion: 四元数 (w, x, y, z)
            order: 旋转顺序，默认 'zyx'

        Returns:
            Tuple[float, float, float]: 欧拉角 (弧度)
        """
        w, x, y, z = quaternion.w, quaternion.x, quaternion.y, quaternion.z

        if order == 'zyx':
            # Roll (X-axis rotation)
            sinr_cosp = 2.0 * (w * x + y * z)
            cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
            roll = math.atan2(sinr_cosp, cosr_cosp)

            # Pitch (Y-axis rotation)
            sinp = 2.0 * (w * y - z * x)
            if abs(sinp) >= 1.0:
                pitch = math.copysign(math.pi / 2, sinp)
            else:
                pitch = math.asin(sinp)

            # Yaw (Z-axis rotation)
            siny_cosp = 2.0 * (w * z + x * y)
            cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            return (roll, pitch, yaw)

        return (0.0, 0.0, 0.0)

    @staticmethod
    def euler_to_quaternion(
        roll: float,
        pitch: float,
        yaw: float,
        order: str = 'zyx',
    ) -> Quaternion:
        """
        欧拉角转四元数

        Args:
            roll: 绕 X 轴旋转（弧度）
            pitch: 绕 Y 轴旋转（弧度）
            yaw: 绕 Z 轴旋转（弧度）
            order: 旋转顺序，默认 'zyx'

        Returns:
            Quaternion: 四元数 (w, x, y, z)
        """
        if order == 'zyx':
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

        return Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)

    @staticmethod
    def is_pose_similar(
        pose1: Pose,
        pose2: Pose,
        position_tolerance: float = 0.01,
        orientation_tolerance: float = 0.05,
    ) -> bool:
        """
        判断两个位姿是否相似

        Args:
            pose1: 位姿 1
            pose2: 位姿 2
            position_tolerance: 位置容差（米）
            orientation_tolerance: 姿态容容差（弧度）

        Returns:
            bool: 位姿是否相似
        """
        pos_dist = PoseValidator.calculate_position_distance(pose1, pose2)
        ori_dist = PoseValidator.calculate_orientation_distance(pose1, pose2)

        return (
            pos_dist <= position_tolerance and
            ori_dist <= orientation_tolerance
        )
