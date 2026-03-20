"""手眼标定验证器。"""

from typing import Any

import numpy as np


class CalibrationValidator:
    """标定结果验证器。"""

    def __init__(
        self,
        max_error: float = 0.01,  # 1cm
        reprojection_threshold: float = 0.05,  # 5 pixels
    ):
        """初始化验证器。

        Args:
            max_error: 最大允许误差（米）
        """
        self.max_error = max_error
        self.reprojection_threshold = reprojection_threshold

    def validate(self, result: dict[str, Any], samples: list) -> dict[str, Any]:
        """验证标定结果。

        Args:
            result: 标定结果
            samples: 标定样本列表

        Returns:
            验证结果字典
        """
        R = result["rotation_matrix"]
        t = result["translation_vector"]

        # 计算各个样本的误差
        errors = []
        for i, sample in enumerate(samples):
            robot_pose = sample.robot_pose
            camera_pose = sample.camera_pose

            # 验证 AX=XB
            error = self._verify_hand_eye(R, t, robot_pose, camera_pose)
            errors.append(error)

        validation_result = {
            "max_error": float(max(errors)),
            "mean_error": float(np.mean(errors)),
            "std_error": float(np.std(errors)),
            "passed": max(errors) < self.max_error,
        }

        return validation_result

    def _verify_hand_eye(
        self,
        R: np.ndarray,
        t: np.ndarray,
        robot_pose: np.ndarray,
        camera_pose: np.ndarray,
    ) -> float:
        """验证手眼变换矩阵。

        验证方法：重投影误差（Eye-in-hand 场景）
        - robot_pose: gripper 在 base 下的位姿 (gripper2base)
        - camera_pose: marker 在 camera 下的位姿 (marker2camera)
        - X: camera 在 gripper 下的位姿 (camera2gripper)

        验证关系：robot_pose @ X ≈ camera_pose（当 camera_pose 为 marker2camera 时）
        即 gripper2base @ camera2gripper ≈ marker2base
        而 marker2camera 是已知的，所以 camera_pose @ X.T ≈ marker2base

        简化验证：检查 camera_pose @ X.T 和 robot_pose 的位置一致性

        Args:
            R: 手眼旋转矩阵
            t: 手眼平移向量
            robot_pose: 机械臂位姿 (4x4 gripper2base)
            camera_pose: 相机位姿 (4x4 marker2camera)

        Returns:
            误差值（米）
        """
        # 构建手眼变换矩阵 X (camera2gripper)
        X = np.eye(4)
        X[:3, :3] = R
        X[:3, 3] = t.flatten() if t.ndim > 1 else t

        # 验证：camera_pose @ X.T ≈ robot_pose（Eye-in-hand）
        # 即 marker2camera @ camera2gripper.T ≈ gripper2base
        # 简化比较位置差异
        predicted = camera_pose @ X.T
        error = np.linalg.norm(predicted[:3, 3] - robot_pose[:3, 3])

        return float(error)
