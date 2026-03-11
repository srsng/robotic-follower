"""
位姿插值器

提供位姿插值功能，支持线性插值和样条插值。
"""

import math
import numpy as np
from typing import List, Tuple
from geometry_msgs.msg import Pose, Point, Quaternion
from scipy import interpolate


class PoseInterpolation:
    """
    位姿插值器

    功能：
    - 线性位姿插值
    - 样条位姿插值
    - 基于速度的位姿轨迹生成
    """

    @staticmethod
    def linear_interpolate(
        pose1: Pose,
        pose2: Pose,
        t: float,
    ) -> Pose:
        """
        线性位姿插值

        Args:
            pose1: 起始位姿
            pose2: 目标位姿
            t: 插值参数 [0, 1]

        Returns:
            Pose: 插值后的位姿
        """
        t = max(0.0, min(1.0, t))

        # 位置线性插值
        position = Point(
            x=pose1.position.x + t * (pose2.position.x - pose1.position.x),
            y=pose1.position.y + t * (pose2.position.y - pose1.position.y),
            z=pose1.position.z + t * (pose2.position.z - pose1.position.z),
        )

        # 姿态球面线性插值
        orientation = PoseInterpolation._slerp(
            pose1.orientation,
            pose2.orientation,
            t,
        )

        return Pose(position=position, orientation=orientation)

    @staticmethod
    def _slerp(
        q1: Quaternion,
        q2: Quaternion,
        t: float,
    ) -> Quaternion:
        """
        四元数球面线性插值 (SLERP)

        Args:
            q1: 起始四元数
            q2: 目标四元数
            t: 插值参数 [0, 1]

        Returns:
            Quaternion: 插值后的四元数
        """
        # 转换为 numpy 数组
        q1_np = np.array([q1.w, q1.x, q1.y, q1.z])
        q2_np = np.array([q2.w, q2.x, q2.y, q2.z])

        # 计算点积
        dot = np.dot(q1_np, q2_np)

        # 如果点积为负，反转一个四元数以确保最短路径
        if dot < 0.0:
            q2_np = -q2_np
            dot = -dot

        # 如果四元数相同，直接返回
        if dot > 0.9995:
            result = q1_np + t * (q2_np - q1_np)
            norm = np.linalg.norm(result)
            if norm < 1e-10:
                return Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
            result = result / norm
            return Quaternion(w=result[0], x=result[1], y=result[2], z=result[3])

        # 计算 SLERP
        theta_0 = math.acos(dot)
        sin_theta_0 = math.sin(theta_0)

        theta = theta_0 * t
        sin_theta = math.sin(theta)

        s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0

        result = s0 * q1_np + s1 * q2_np
        result = result / np.linalg.norm(result)

        return Quaternion(w=result[0], x=result[1], y=result[2], z=result[3])

    @staticmethod
    def generate_linear_trajectory(
        start_pose: Pose,
        end_pose: Pose,
        num_steps: int,
    ) -> List[Pose]:
        """
        生成线性位姿轨迹

        Args:
            start_pose: 起始位姿
            end_pose: 目标位姿
            num_steps: 轨迹点数

        Returns:
            List[Pose]: 位姿轨迹列表
        """
        if num_steps < 2:
            return [start_pose]

        trajectory = []
        for i in range(num_steps):
            t = i / (num_steps - 1)
            trajectory.append(PoseInterpolation.linear_interpolate(start_pose, end_pose, t))

        return trajectory

    @staticmethod
    def spline_interpolate(
        poses: List[Pose],
        num_steps: int,
        method: str = 'cubic',
    ) -> List[Pose]:
        """
        样条位姿插值

        Args:
            poses: 关键位姿列表
            num_steps: 总轨迹点数
            method: 插值方法，'linear' 或 'cubic'

        Returns:
            List[Pose]: 插值后的位姿轨迹
        """
        if len(poses) < 2:
            return poses.copy()

        # 提取位置和姿态
        positions = [(p.position.x, p.position.y, p.position.z) for p in poses]
        orientations = [PoseInterpolation._quaternion_to_euler(p.orientation) for p in poses]

        # 样条插值位置
        if method == 'cubic' and len(poses) >= 3:
            x_spline = interpolate.CubicSpline(range(len(positions)), [p[0] for p in positions])
            y_spline = interpolate.CubicSpline(range(len(positions)), [p[1] for p in positions])
            z_spline = interpolate.CubicSpline(range(len(positions)), [p[2] for p in positions])
        else:
            x_spline = interpolate.interp1d(
                range(len(positions)), [p[0] for p in positions], kind='linear'
            )
            y_spline = interpolate.interp1d(
                range(len(positions)), [p[1] for p in positions], kind='linear'
            )
            z_spline = interpolate.interp1d(
                range(len(positions)), [p[2] for p in positions], kind='linear'
            )

        # 样条插值姿态（使用线性插值避免欧拉角不连续）
        roll_spline = interpolate.interp1d(
            range(len(orientations)), [o[0] for o in orientations], kind='linear'
        )
        pitch_spline = interpolate.interp1d(
            range(len(orientations)), [o[1] for o in orientations], kind='linear'
        )
        yaw_spline = interpolate.interp1d(
            range(len(orientations)), [o[2] for o in orientations], kind='linear'
        )

        # 生成轨迹
        trajectory = []
        segment_length = (len(poses) - 1) / (num_steps - 1)

        for i in range(num_steps):
            t = i * segment_length
            t = min(t, len(poses) - 1 - 1e-10)

            # 插值位置
            position = Point(
                x=float(x_spline(t)),
                y=float(y_spline(t)),
                z=float(z_spline(t)),
            )

            # 插值姿态
            roll = float(roll_spline(t))
            pitch = float(pitch_spline(t))
            yaw = float(yaw_spline(t))
            orientation = PoseInterpolation._euler_to_quaternion(roll, pitch, yaw)

            trajectory.append(Pose(position=position, orientation=orientation))

        return trajectory

    @staticmethod
    def _quaternion_to_euler(quat: Quaternion) -> Tuple[float, float, float]:
        """四元数转欧拉角"""
        w, x, y, z = quat.w, quat.x, quat.y, quat.z

        # Roll (X-axis)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (Y-axis)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (Z-axis)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return (roll, pitch, yaw)

    @staticmethod
    def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
        """欧拉角转四元数"""
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

    @staticmethod
    def generate_trajectory_with_velocity(
        start_pose: Pose,
        end_pose: Pose,
        max_velocity: float,
        time_step: float = 0.01,
    ) -> List[Pose]:
        """
        基于最大速度生成位姿轨迹

        Args:
            start_pose: 起始位姿
            end_pose: 目标位姿
            max_velocity: 最大速度（米/秒）
            time_step: 时间步长（秒）

        Returns:
            List[Pose]: 位姿轨迹列表
        """
        # 计算直线距离
        dx = end_pose.position.x - start_pose.position.x
        dy = end_pose.position.y - start_pose.position.y
        dz = end_pose.position.z - start_pose.position.z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        # 计算所需时间
        total_time = distance / max_velocity if max_velocity > 0 else 1.0
        num_steps = max(2, int(total_time / time_step))

        return PoseInterpolation.generate_linear_trajectory(start_pose, end_pose, num_steps)
