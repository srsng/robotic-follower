"""标定验证器"""

import numpy as np
from typing import Tuple


class CalibrationValidator:
    """标定结果验证器

    用于验证相机标定结果的质量
    """

    @staticmethod
    def validate_reprojection_error(error: float, threshold: float = 0.5) -> bool:
        """验证重投影误差

        Args:
            error: 重投影误差
            threshold: 误差阈值（像素）

        Returns:
            bool: 是否通过验证
        """
        return error < threshold

    @staticmethod
    def validate_focal_length(fx: float, fy: float, ratio_threshold: float = 0.1) -> bool:
        """验证焦距比例

        Args:
            fx: x方向焦距
            fy: y方向焦距
            ratio_threshold: 比例阈值

        Returns:
            bool: 是否通过验证
        """
        # 检查焦距是否为正
        if fx <= 0 or fy <= 0:
            return False

        # 检查两个焦距是否接近（方形像素）
        ratio = abs(fx - fy) / max(fx, fy)
        return ratio < ratio_threshold

    @staticmethod
    def validate_principal_point(cx: float, cy: float,
                                  width: int, height: int,
                                  margin: float = 0.2) -> bool:
        """验证光心位置

        Args:
            cx: x方向光心坐标
            cy: y方向光心坐标
            width: 图像宽度
            height: 图像高度
            margin: 允许的边缘比例

        Returns:
            bool: 是否通过验证
        """
        # 光心应该在图像中心附近
        center_x = width / 2.0
        center_y = height / 2.0

        offset_x = abs(cx - center_x) / width
        offset_y = abs(cy - center_y) / height

        return offset_x < margin and offset_y < margin

    @staticmethod
    def validate_distortion_coeffs(distortion: np.ndarray,
                                    max_k: float = 2.0,
                                    max_p: float = 0.2) -> bool:
        """验证畸变系数

        Args:
            distortion: 畸变系数数组 [k1, k2, p1, p2, k3]
            max_k: 径向畸变系数最大值
            max_p: 切向畸变系数最大值

        Returns:
            bool: 是否通过验证
        """
        if len(distortion) < 5:
            return False

        k1, k2, p1, p2, k3 = distortion[0:5]

        # 检查径向畸变
        if abs(k1) > max_k or abs(k2) > max_k or abs(k3) > max_k:
            return False

        # 检查切向畸变
        if abs(p1) > max_p or abs(p2) > max_p:
            return False

        return True

    @staticmethod
    def validate_camera_matrix(matrix: np.ndarray,
                                image_size: Tuple[int, int]) -> bool:
        """验证相机内参矩阵

        Args:
            matrix: 3x3 相机内参矩阵
            image_size: 图像尺寸 (width, height)

        Returns:
            bool: 是否通过验证
        """
        if matrix.shape != (3, 3):
            return False

        # 检查矩阵是否为归一化形式
        if abs(matrix[2, 2] - 1.0) > 0.01:
            return False

        # 提取参数
        fx = matrix[0, 0]
        fy = matrix[1, 1]
        cx = matrix[0, 2]
        cy = matrix[1, 2]

        # 验证焦距
        if not CalibrationValidator.validate_focal_length(fx, fy):
            return False

        # 验证光心
        width, height = image_size
        if not CalibrationValidator.validate_principal_point(cx, cy, width, height):
            return False

        return True

    @staticmethod
    def generate_validation_report(camera_matrix: np.ndarray,
                                    distortion: np.ndarray,
                                    reprojection_error: float,
                                    image_size: Tuple[int, int]) -> dict:
        """生成验证报告

        Args:
            camera_matrix: 相机内参矩阵
            distortion: 畸变系数
            reprojection_error: 重投影误差
            image_size: 图像尺寸

        Returns:
            dict: 验证报告
        """
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]

        report = {
            'reprojection_error': {
                'value': reprojection_error,
                'valid': CalibrationValidator.validate_reprojection_error(reprojection_error)
            },
            'focal_length': {
                'fx': fx,
                'fy': fy,
                'ratio': abs(fx - fy) / max(fx, fy),
                'valid': CalibrationValidator.validate_focal_length(fx, fy)
            },
            'principal_point': {
                'cx': cx,
                'cy': cy,
                'center_x': image_size[0] / 2.0,
                'center_y': image_size[1] / 2.0,
                'valid': CalibrationValidator.validate_principal_point(cx, cy, image_size[0], image_size[1])
            },
            'distortion': {
                'k1': distortion[0] if len(distortion) > 0 else 0.0,
                'k2': distortion[1] if len(distortion) > 1 else 0.0,
                'p1': distortion[2] if len(distortion) > 2 else 0.0,
                'p2': distortion[3] if len(distortion) > 3 else 0.0,
                'k3': distortion[4] if len(distortion) > 4 else 0.0,
                'valid': CalibrationValidator.validate_distortion_coeffs(distortion)
            },
            'overall_valid': (
                CalibrationValidator.validate_reprojection_error(reprojection_error) and
                CalibrationValidator.validate_focal_length(fx, fy) and
                CalibrationValidator.validate_principal_point(cx, cy, image_size[0], image_size[1]) and
                CalibrationValidator.validate_distortion_coeffs(distortion)
            )
        }

        return report
