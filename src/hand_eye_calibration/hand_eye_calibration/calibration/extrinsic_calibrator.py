"""手眼外参标定器（AX=XB求解）。"""

import cv2
import numpy as np
from scipy import spatial
from typing import List, Tuple


class ExtrinsicCalibrator:
    """手眼外参标定器（AX=XB求解）。"""

    def calibrate(
        self,
        robot_poses: List[np.ndarray],
        camera_poses: List[np.ndarray]
    ) -> dict:
        """执行手眼标定。

        Args:
            robot_poses: 机械臂位姿列表（4x4齐次变换矩阵）
            camera_poses: 相机位姿列表（4x4齐次变换矩阵）

        Returns:
            标定结果字典，包含：
            - rotation_matrix: 3x3旋转矩阵
            - translation_vector: 3x1平移向量
            - quaternion: [x, y, z, w]四元数
            - error: 标定误差
        """
        if len(robot_poses) != len(camera_poses):
            raise ValueError("机械臂位姿和相机位姿数量不一致")

        if len(robot_poses) < 3:
            raise ValueError("至少需要3个位姿才能进行标定")

        # 构建位姿变换对
        A_motions = []
        B_motions = []

        for i in range(len(robot_poses) - 1):
            # 计算机械臂位姿变换 A
            R1_a, t1_a = self._decompose_pose(robot_poses[i])
            R2_a, t2_a = self._decompose_pose(robot_poses[i + 1])
            R_a = R2_a @ R1_a.T
            t_a = t2_a - R_a @ t1_a
            A_motions.append((R_a, t_a))

            # 计算相机位姿变换 B
            R1_b, t1_b = self._decompose_pose(camera_poses[i])
            R2_b, t2_b = self._decompose_pose(camera_poses[i + 1])
            R_b = R2_b @ R1_b.T
            t_b = t2_b - R_b @ t1_b
            B_motions.append((R_b, t_b))

        # 转换为cv格式
        R_gripper2base = [cv2.Rodrigues(r)[0] for r, t in A_motions]
        t_gripper2base = [t for r, t in A_motions]
        R_target2cam = [cv2.Rodrigues(r)[0] for r, t in B_motions]
        t_target2cam = [t for r, t in B_motions]

        # 使用cv2.calibrateHandEye求解
        # 尝试不同的方法，选择误差最小的
        methods = [
            cv2.CALIB_HAND_EYE_TSAI,
            cv2.CALIB_HAND_EYE_PARK,
            cv2.CALIB_HAND_EYE_HORAUD,
            cv2.CALIB_HAND_EYE_ANDREFF,
            cv2.CALIB_HAND_EYE_DANIILIDIS,
        ]

        best_error = float('inf')
        best_R = None
        best_t = None

        for method in methods:
            try:
                R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
                    R_gripper2base=R_gripper2base,
                    t_gripper2base=t_gripper2base,
                    R_target2cam=R_target2cam,
                    t_target2cam=t_target2cam,
                    method=method
                )

                # 计算误差
                error = self._compute_calibration_error(
                    R_cam2gripper, t_cam2gripper,
                    A_motions, B_motions
                )

                if error < best_error:
                    best_error = error
                    best_R = R_cam2gripper
                    best_t = t_cam2gripper
            except cv2.error:
                continue

        if best_R is None:
            raise ValueError("所有标定方法均失败")

        # 转换为四元数
        quaternion = self._rotation_to_quaternion(best_R)

        return {
            'rotation_matrix': best_R,
            'translation_vector': best_t,
            'quaternion': quaternion,
            'error': float(best_error)
        }

    def _decompose_pose(self, pose: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """分解位姿为旋转和平移。

        Args:
            pose: 4x4齐次变换矩阵

        Returns:
            (R, t): 3x3旋转矩阵，3x1平移向量
        """
        R = pose[:3, :3]
        t = pose[:3, 3]
        return R, t

    def _rotation_to_quaternion(self, R: np.ndarray) -> np.ndarray:
        """旋转矩阵转四元数。

        使用scipy.spatial.transform.Rotation（社区成熟方案）

        Args:
            R: 3x3旋转矩阵

        Returns:
            四元数 [x, y, z, w]
        """
        quaternion = spatial.transform.Rotation.from_matrix(R).as_quat()
        return quaternion

    def _compute_calibration_error(
        self,
        R: np.ndarray,
        t: np.ndarray,
        A_motions: List[Tuple[np.ndarray, np.ndarray]],
        B_motions: List[Tuple[np.ndarray, np.ndarray]]
    ) -> float:
        """计算标定误差：验证AX=XB。

        Args:
            R: 手眼旋转矩阵
            t: 手眼平移向量
            A_motions: 机械臂位姿变换列表
            B_motions: 相机位姿变换列表

        Returns:
            平均误差
        """
        errors = []

        for (R_a, t_a), (R_b, t_b) in zip(A_motions, B_motions):
            # 构建4x4齐次变换矩阵
            def make_homo(R, t):
                M = np.eye(4)
                M[:3, :3] = R
                M[:3, 3] = t
                return M

            A = make_homo(R_a, t_a)
            B = make_homo(R_b, t_b)
            X = make_homo(R, t)

            # 验证 AX ≈ XB
            AX = A @ X
            XB = X @ B
            error = np.linalg.norm(AX - XB)
            errors.append(error)

        return np.mean(errors)
