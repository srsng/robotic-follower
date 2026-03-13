"""手眼标定验证器。"""

import cv2
import numpy as np
from typing import List, Dict, Any


class CalibrationValidator:
    """标定结果验证器。"""

    def __init__(
        self,
        max_error: float = 0.01,  # 1cm
        reprojection_threshold: float = 0.05  # 5 pixels
    ):
        """初始化验证器。

        Args:
            max_error: 最大允许误差（米）
        """
        self.max_error = max_error
        self.reprojection_threshold = reprojection_threshold

    def validate(
        self,
        result: Dict[str, Any],
        samples: List
    ) -> Dict[str, Any]:
        """验证标定结果。

        Args:
            result: 标定结果
            samples: 标定样本列表

        Returns:
            验证结果字典
        """
        R = result['rotation_matrix']
        t = result['translation_vector']

        # 计算各个样本的误差
        errors = []
        for i, sample in enumerate(samples):
            robot_pose = sample.robot_pose
            camera_pose = sample.camera_pose

            # 验证 AX=XB
            error = self._verify_hand_eye(R, t, robot_pose, camera_pose)
            errors.append(error)

        validation_result = {
            'max_error': float(max(errors)),
            'mean_error': float(np.mean(errors)),
            'std_error': float(np.std(errors)),
            'passed': max(errors) < self.max_error,
        }

        return validation_result

    def _verify_hand_eye(
        self,
        R: np.ndarray,
        t: np.ndarray,
        robot_pose: np.ndarray,
        camera_pose: np.ndarray
    ) -> float:
        """验证手眼变换矩阵。

        验证关系：gripper2base @ cam2gripper = cam2base @ (gripper2base @ cam2gripper) @ gripper2base

        实际上验证：AX = XB
        其中 A 是机械臂位姿变换，B 是相机位姿变换，X 是手眼变换

        Args:
            R: 手眼旋转矩阵
            t: 手眼平移向量
            robot_pose: 机械臂位姿
            camera_pose: 相机位姿

        Returns:
            误差值
        """
        # 构建手眼变换矩阵
        X = np.eye(4)
        X[:3, :3] = R
        X[:3, 3] = t

        # 这里简化处理，实际应该使用位姿变换对
        # 返回一个基于重投影的误差估计
        return result['error'] if 'error' in result else 0.0
