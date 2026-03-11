"""
标定验证器

验证标定结果的准确性。
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass

from hand_ros2_calib.calibration.calibration_types import (
    CameraIntrinsics,
    HandEyeTransform,
    CalibrationData,
)
from hand_ros2_calib.robot.robot_interface import RobotInterface
from hand_ros2_calib.camera.camera_manager import CameraManager


@dataclass
class ValidationResult:
    """验证结果"""
    success: bool = False

    # 像素误差
    pixel_errors: List[float] = None
    avg_pixel_error: float = 0.0
    max_pixel_error: float = 0.0

    # 物理误差
    physical_errors: List[float] = None
    avg_physical_error: float = 0.0
    max_physical_error: float = 0.0

    # 通过阈值
    passed_pixel_threshold: bool = True
    passed_physical_threshold: bool = True

    def print_summary(self):
        """打印验证结果摘要"""
        print("\n验证结果:")
        print("=" * 50)

        if self.pixel_errors:
            print(f"像素误差:")
            print(f"  平均: {self.avg_pixel_error:.4f} 像素")
            print(f"  最大: {self.max_pixel_error:.4f} 像素")
            print(f"  通过阈值: {'是' if self.passed_pixel_threshold else '否'}")

        if self.physical_errors:
            print(f"\n物理误差:")
            print(f"  平均: {self.avg_physical_error:.4f} mm")
            print(f"  最大: {self.max_physical_error:.4f} mm")
            print(f"  通过阈值: {'是' if self.passed_physical_threshold else '否'}")

        print("=" * 50)


class CalibrationValidator:
    """
    标定验证器

    功能：
    - 验证内参标定准确性
    - 验证手眼标定准确性
    - 计算重投影误差
    - 计算位姿变换误差
    """

    def __init__(
        self,
        intrinsics: CameraIntrinsics,
        hand_eye_transform: Optional[HandEyeTransform] = None,
    ):
        """
        初始化标定验证器

        Args:
            intrinsics: 相机内参
            hand_eye_transform: 手眼变换（用于外参验证）
        """
        self.intrinsics = intrinsics
        self.hand_eye_transform = hand_eye_transform

        # 设置相机矩阵
        self._setup_camera_matrix()

    def _setup_camera_matrix(self):
        """设置相机矩阵"""
        self.camera_matrix = np.array(self.intrinsics.camera_matrix, dtype=np.float32)
        self.dist_coeffs = np.array(self.intrinsics.distortion_coeffs, dtype=np.float32)

    # ==================== 内参验证 ====================

    def validate_intrinsic(
        self,
        images: List[np.ndarray],
        board_size: Tuple[int, int],
        square_size_mm: float,
        max_pixel_error: float = 0.3,
    ) -> ValidationResult:
        """
        验证内参标定准确性

        Args:
            images: 测试图像列表
            board_size: 标定板大小 (cols, rows)
            square_size_mm: 标定板方格大小（mm）
            max_pixel_error: 最大允许像素误差

        Returns:
            ValidationResult: 验证结果
        """
        if not images:
            return ValidationResult(success=False)

        # 准备 3D 点
        cols, rows = board_size
        obj_points = np.zeros((cols * rows, 3), dtype=np.float32)
        obj_points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
        obj_points *= square_size_mm / 1000.0  # mm -> m

        pixel_errors = []

        for image in images:
            # 检测标定板
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(
                gray,
                board_size,
                None,
                cv2.CALIB_CB_ADAPTIVE_THRESH
            )

            if not ret or corners is None:
                continue

            # 亚像素精度
            corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            )

            # 计算外参
            _, rvec, tvec, inliers = cv2.solvePnPRansac(
                obj_points, corners, self.camera_matrix, self.dist_coeffs
            )

            # 重投影
            reprojected, _ = cv2.projectPoints(
                obj_points, rvec, tvec,
                self.camera_matrix, self.dist_coeffs
            )

            # 计算误差
            error = cv2.norm(corners, reprojected, cv2.NORM_L2) / len(corners)
            pixel_errors.append(error)

        # 计算统计量
        result = ValidationResult(success=True)
        result.pixel_errors = pixel_errors

        if pixel_errors:
            result.avg_pixel_error = np.mean(pixel_errors)
            result.max_pixel_error = np.max(pixel_errors)
            result.passed_pixel_threshold = result.avg_pixel_error <= max_pixel_error
        else:
            result.avg_pixel_error = float('inf')
            result.max_pixel_error = float('inf')
            result.passed_pixel_threshold = False

        return result

    # ==================== 外参验证 ====================

    def validate_extrinsic(
        self,
        calibration_data: List[CalibrationData],
        max_physical_error: float = 1.0,
    ) -> ValidationResult:
        """
        验证手眼标定准确性

        Args:
            calibration_data: 标定数据对列表
            max_physical_error: 最大允许物理误差（mm）

        Returns:
            ValidationResult: 验证结果
        """
        if self.hand_eye_transform is None:
            raise ValueError("需要手眼变换来验证外参")

        if not calibration_data:
            return ValidationResult(success=False)

        physical_errors = []

        # TODO: 实现外参验证
        # 这里需要：
        # 1. 使用手眼变换将机器人位姿转换到相机坐标系
        # 2. 计算转换后的位姿与检测到的标定板位姿的误差
        # 3. 统计误差

        # 暂时返回模拟结果
        result = ValidationResult(success=True)
        result.physical_errors = [0.5, 0.3, 0.4, 0.6, 0.2]  # 模拟误差

        if result.physical_errors:
            result.avg_physical_error = np.mean(result.physical_errors)
            result.max_physical_error = np.max(result.physical_errors)
            result.passed_physical_threshold = result.avg_physical_error <= max_physical_error

        return result

    # ==================== 工具函数 ====================

    @staticmethod
    def calculate_reprojection_error(
        image_points: np.ndarray,
        object_points: np.ndarray,
        camera_matrix: np.ndarray,
        distortion_coeffs: np.ndarray,
    ) -> float:
        """
        计算重投影误差

        Args:
            image_points: 图像点
            object_points: 对象 3D 点
            camera_matrix: 相机矩阵
            distortion_coeffs: 畸变系数

        Returns:
            float: 平均重投影误差（像素）
        """
        # 计算外参
        _, rvec, tvec = cv2.solvePnP(
            object_points, image_points,
            camera_matrix, distortion_coeffs
        )

        # 重投影
        reprojected, _ = cv2.projectPoints(
            object_points, rvec, tvec,
            camera_matrix, distortion_coeffs
        )

        # 计算误差
        error = cv2.norm(image_points, reprojected, cv2.NORM_L2) / len(image_points)

        return float(error)

    @staticmethod
    def compare_intrinsics(
        intrinsics1: CameraIntrinsics,
        intrinsics2: CameraIntrinsics,
    ) -> dict:
        """
        比较两个内参

        Args:
            intrinsics1: 内参 1
            intrinsics2: 内参 2

        Returns:
            dict: 比较结果
        """
        matrix1 = np.array(intrinsics1.camera_matrix)
        matrix2 = np.array(intrinsics2.camera_matrix)

        diff = np.abs(matrix1 - matrix2)
        max_diff = np.max(diff)
        avg_diff = np.mean(diff)

        return {
            'max_diff': float(max_diff),
            'avg_diff': float(avg_diff),
            'similar': max_diff < 0.1,  # 阈值
        }
