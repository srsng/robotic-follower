"""
位姿计算器

提供相对位姿计算、位姿组合、变换矩阵计算等功能。
"""

import math
import numpy as np
from typing import List, Tuple
from geometry_msgs.msg import Pose, Point, Quaternion


class PoseCalculator:
    """
    位姿计算器

    功能：
    - 计算相对位姿
    - 组合位姿变换
    - 计算变换矩阵
    - 位姿求逆
    """

    @staticmethod
    def calculate_relative_pose(
        base_pose: Pose,
        target_pose: Pose,
    ) -> Pose:
        """
        计算目标位姿相对于基准位姿的相对位姿

        Args:
            base_pose: 基准位姿
            target_pose: 目标位姿

        Returns:
            Pose: 相对位姿 (base -> target)
        """
        # 获取变换矩阵
        base_matrix = PoseCalculator.pose_to_matrix(base_pose)
        target_matrix = PoseCalculator.pose_to_matrix(target_pose)

        # 计算相对变换: T_relative = T_base^(-1) * T_target
        base_inv = np.linalg.inv(base_matrix)
        relative_matrix = np.dot(base_inv, target_matrix)

        return PoseCalculator.matrix_to_pose(relative_matrix)

    @staticmethod
    def apply_relative_pose(
        base_pose: Pose,
        relative_pose: Pose,
    ) -> Pose:
        """
        将相对位姿应用到基准位姿

        Args:
            base_pose: 基准位姿
            relative_pose: 相对位姿

        Returns:
            Pose: 结果位姿
        """
        base_matrix = PoseCalculator.pose_to_matrix(base_pose)
        relative_matrix = PoseCalculator.pose_to_matrix(relative_pose)

        result_matrix = np.dot(base_matrix, relative_matrix)
        return PoseCalculator.matrix_to_pose(result_matrix)

    @staticmethod
    def pose_to_matrix(pose: Pose) -> np.ndarray:
        """
        位姿转 4x4 齐次变换矩阵

        Args:
            pose: 位姿

        Returns:
            np.ndarray: 4x4 齐次变换矩阵
        """
        q = pose.orientation
        t = pose.position

        # 四元数转旋转矩阵
        w, x, y, z = q.w, q.x, q.y, q.z
        rotation = np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w,     2*x*z + 2*y*w,     0.0],
            [2*x*y + 2*z*w,     1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w,     0.0],
            [2*x*z - 2*y*w,     2*y*z + 2*x*w,     1 - 2*x*x - 2*y*y, 0.0],
            [0.0,               0.0,               0.0,               1.0],
        ])

        # 平移
        rotation[0, 3] = t.x
        rotation[1, 3] = t.y
        rotation[2, 3] = t.z

        return rotation

    @staticmethod
    def matrix_to_pose(matrix: np.ndarray) -> Pose:
        """
        4x4 齐次变换矩阵转位姿

        Args:
            matrix: 4x4 齐次变换矩阵

        Returns:
            Pose: 位姿
        """
        # 提取平移
        position = Point(
            x=float(matrix[0, 3]),
            y=float(matrix[1, 3]),
            z=float(matrix[2, 3]),
        )

        # 提取旋转矩阵并转四元数
        rotation = matrix[:3, :3]
        orientation = PoseCalculator.rotation_matrix_to_quaternion(rotation)

        return Pose(position=position, orientation=orientation)

    @staticmethod
    def rotation_matrix_to_quaternion(rotation: np.ndarray) -> Quaternion:
        """
        3x3 旋转矩阵转四元数

        Args:
            rotation: 3x3 旋转矩阵

        Returns:
            Quaternion: 四元数 (w, x, y, z)
        """
        trace = np.trace(rotation)

        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            w = 0.25 * s
            x = (rotation[2, 1] - rotation[1, 2]) / s
            y = (rotation[0, 2] - rotation[2, 0]) / s
            z = (rotation[1, 0] - rotation[0, 1]) / s
        elif (rotation[0, 0] > rotation[1, 1]) and (rotation[0, 0] > rotation[2, 2]):
            s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            w = (rotation[2, 1] - rotation[1, 2]) / s
            x = 0.25 * s
            y = (rotation[0, 1] + rotation[1, 0]) / s
            z = (rotation[0, 2] + rotation[2, 0]) / s
        elif rotation[1, 1] > rotation[2, 2]:
            s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            w = (rotation[0, 2] - rotation[2, 0]) / s
            x = (rotation[0, 1] + rotation[1, 0]) / s
            y = 0.25 * s
            z = (rotation[1, 2] + rotation[2, 1]) / s
        else:
            s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            w = (rotation[1, 0] - rotation[0, 1]) / s
            x = (rotation[0, 2] + rotation[2, 0]) / s
            y = (rotation[1, 2] + rotation[2, 1]) / s
            z = 0.25 * s

        # 归一化
        norm = math.sqrt(w*w + x*x + y*y + z*z)
        if norm > 0:
            w, x, y, z = w/norm, x/norm, y/norm, z/norm

        return Quaternion(w=w, x=x, y=y, z=z)

    @staticmethod
    def quaternion_to_rotation_matrix(quat: Quaternion) -> np.ndarray:
        """
        四元数转 3x3 旋转矩阵

        Args:
            quat: 四元数 (w, x, y, z)

        Returns:
            np.ndarray: 3x3 旋转矩阵
        """
        w, x, y, z = quat.w, quat.x, quat.y, quat.z

        return np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [2*x*y + 2*z*w,     1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w,     2*y*z + 2*x*w,     1 - 2*x*x - 2*y*y],
        ])

    @staticmethod
    def inverse_pose(pose: Pose) -> Pose:
        """
        求位姿的逆（对应变换矩阵的逆）

        Args:
            pose: 位姿

        Returns:
            Pose: 逆位姿
        """
        matrix = PoseCalculator.pose_to_matrix(pose)
        inv_matrix = np.linalg.inv(matrix)
        return PoseCalculator.matrix_to_pose(inv_matrix)

    @staticmethod
    def compose_poses(pose1: Pose, pose2: Pose) -> Pose:
        """
        组合两个位姿变换（相当于矩阵乘法）

        Args:
            pose1: 位姿 1
            pose2: 位姿 2

        Returns:
            Pose: 组合后的位姿
        """
        matrix1 = PoseCalculator.pose_to_matrix(pose1)
        matrix2 = PoseCalculator.pose_to_matrix(pose2)

        result = np.dot(matrix1, matrix2)
        return PoseCalculator.matrix_to_pose(result)

    @staticmethod
    def multiply_quaternions(q1: Quaternion, q2: Quaternion) -> Quaternion:
        """
        四元数乘法

        Args:
            q1: 四元数 1
            q2: 四元数 2

        Returns:
            Quaternion: 乘积四元数
        """
        w1, x1, y1, z1 = q1.w, q1.x, q1.y, q1.z
        w2, x2, y2, z2 = q2.w, q2.x, q2.y, q2.z

        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2

        # 归一化
        norm = math.sqrt(w*w + x*x + y*y + z*z)
        if norm > 0:
            w, x, y, z = w/norm, x/norm, y/norm, z/norm

        return Quaternion(w=w, x=x, y=y, z=z)

    @staticmethod
    def conjugate_quaternion(q: Quaternion) -> Quaternion:
        """
        四元数共轭

        Args:
            q: 四元数

        Returns:
            Quaternion: 共轭四元数
        """
        return Quaternion(w=q.w, x=-q.x, y=-q.y, z=-q.z)
