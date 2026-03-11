"""
内参标定器

执行相机内参标定，集成 hand_eyes_calibration C++ 库。
"""

import cv2
import numpy as np
import os
import time
from typing import List, Tuple, Optional
import xml.etree.ElementTree as ET

from hand_ros2_calib.calibration.calibration_types import (
    CalibrationConfig,
    CameraIntrinsics,
    PatternType,
)


class IntrinsicCalibrator:
    """
    内参标定器

    功能：
    - 检测标定板
    - 收集标定图像
    - 执行内参标定
    - 保存/加载标定结果
    """

    def __init__(self, config: CalibrationConfig):
        """
        初始化内参标定器

        Args:
            config: 标定配置
        """
        self.config = config
        self.images = []
        self.corners_list = []

        # 设置 OpenCV 标定板检测参数
        self._setup_pattern_criteria()

    def _setup_pattern_criteria(self):
        """设置标定板检测参数"""
        # 终止条件
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # 标定板尺寸
        self.board_size = (self.config.board_cols, self.config.board_rows)

        # 世界坐标点（标定板平面为 Z=0）
        self.object_points = np.zeros(
            (self.config.board_cols * self.config.board_rows, 3),
            dtype=np.float32
        )
        self.object_points[:, :2] = np.mgrid[
            0:self.config.board_cols,
            0:self.config.board_rows
        ].T.reshape(-1, 2)
        self.object_points *= self.config.square_size_mm / 1000.0  # mm -> m

    def detect_board(
        self,
        image: np.ndarray,
        visualize: bool = False,
    ) -> Tuple[bool, Optional[np.ndarray]]:
        """
        检测图像中的标定板

        Args:
            image: 输入图像
            visualize: 是否可视化检测结果

        Returns:
            Tuple[bool, Optional[np.ndarray]]: (是否检测成功, 角点坐标)
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        corners = None
        detected = False

        if self.config.pattern_type == PatternType.CHESSBOARD:
            # 棋盘格检测
            ret, corners = cv2.findChessboardCorners(
                gray,
                self.board_size,
                None,
                cv2.CALIB_CB_ADAPTIVE_THRESH |
                cv2.CALIB_CB_FAST_CHECK |
                cv2.CALIB_CB_NORMALIZE_IMAGE
            )

            if ret and corners is not None:
                # 亚像素精度优化
                corners = cv2.cornerSubPix(
                    gray,
                    corners,
                    (11, 11),
                    (-1, -1),
                    self.criteria
                )
                detected = True

        elif self.config.pattern_type == PatternType.CIRCLES_ASYMMETRIC:
            # 非对称圆点检测
            ret, centers = cv2.findCirclesGrid(
                gray,
                self.board_size,
                cv2.CALIB_CB_ASYMMETRIC_GRID
            )
            if ret and centers is not None:
                corners = centers
                detected = True

        elif self.config.pattern_type == PatternType.CIRCLES_SYMMETRIC:
            # 对称圆点检测
            ret, centers = cv2.findCirclesGrid(
                gray,
                self.board_size,
                cv2.CALIB_CB_CLUSTERING
            )
            if ret and centers is not None:
                corners = centers
                detected = True

        # 可视化
        if visualize and detected:
            vis_image = image.copy()
            cv2.drawChessboardCorners(vis_image, self.board_size, corners, True)
            cv2.imshow('标定板检测', vis_image)
            cv2.waitKey(100)

        return (detected, corners)

    def add_image(self, image: np.ndarray, corners: np.ndarray) -> bool:
        """
        添加标定图像

        Args:
            image: 标定图像
            corners: 检测到的角点

        Returns:
            bool: 是否成功添加
        """
        if len(self.images) >= self.config.intrinsic_num_images:
            return False

        self.images.append(image)
        self.corners_list.append(corners)
        return True

    def calibrate(self) -> Optional[CameraIntrinsics]:
        """
        执行内参标定

        Returns:
            Optional[CameraIntrinsics]: 标定结果，失败返回 None
        """
        if len(self.images) < 3:
            print(f"错误：至少需要 3 张标定图像，当前只有 {len(self.images)} 张")
            return None

        print(f"开始内参标定，使用 {len(self.images)} 张图像...")

        # 准备标定数据
        obj_points = [self.object_points] * len(self.corners_list)

        # 执行标定
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            obj_points,
            self.corners_list,
            (self.images[0].shape[1], self.images[0].shape[0]),
            None,
            None,
            flags=cv2.CALIB_FIX_K4 | cv2.CALIB_FIX_K5
        )

        if not ret:
            print("内参标定失败")
            return None

        # 计算重投影误差
        mean_error = 0
        for i, corners in enumerate(self.corners_list):
            # 投影 3D 点到图像平面
            reprojected, _ = cv2.projectPoints(
                obj_points[i],
                rvecs[i],
                tvecs[i],
                camera_matrix,
                dist_coeffs
            )
            error = cv2.norm(corners, reprojected, cv2.NORM_L2) / len(corners)
            mean_error += error

        rms_error = mean_error / len(self.corners_list)
        print(f"内参标定完成，RMS 误差: {rms_error:.4f} 像素")

        # 返回结果
        return CameraIntrinsics(
            camera_matrix=camera_matrix.tolist(),
            distortion_coeffs=dist_coeffs.tolist(),
            image_width=self.images[0].shape[1],
            image_height=self.images[0].shape[0],
        ), rms_error

    def save_to_xml(
        self,
        intrinsics: CameraIntrinsics,
        output_path: str,
    ) -> bool:
        """
        保存内参到 XML 文件（兼容 hand_eyes_calibration 格式）

        Args:
            intrinsics: 内参
            output_path: 输出文件路径

        Returns:
            bool: 是否成功保存
        """
        try:
            root = ET.Element("CalibrationResult")

            # 相机矩阵
            matrix_elem = ET.SubElement(root, "CameraMatrix")
            for i, row in enumerate(intrinsics.camera_matrix):
                for j, val in enumerate(row):
                    elem = ET.SubElement(matrix_elem, f"m{i}{j}")
                    elem.text = str(val)

            # 畸变系数
            dist_elem = ET.SubElement(root, "DistortionCoefficients")
            for i, val in enumerate(intrinsics.distortion_coeffs):
                elem = ET.SubElement(dist_elem, f"k{i}")
                elem.text = str(val)

            # 图像尺寸
            size_elem = ET.SubElement(root, "ImageSize")
            ET.SubElement(size_elem, "width").text = str(intrinsics.image_width)
            ET.SubElement(size_elem, "height").text = str(intrinsics.image_height)

            # 保存文件
            tree = ET.ElementTree(root)
            tree.write(output_path, encoding="utf-8", xml_declaration=True)

            print(f"内参已保存到: {output_path}")
            return True

        except Exception as e:
            print(f"保存内参失败: {e}")
            return False

    @staticmethod
    def load_from_xml(input_path: str) -> Optional[CameraIntrinsics]:
        """
        从 XML 文件加载内参

        Args:
            input_path: 输入文件路径

        Returns:
            Optional[CameraIntrinsics]: 内参，失败返回 None
        """
        try:
            tree = ET.parse(input_path)
            root = tree.getroot()

            # 读取相机矩阵
            camera_matrix = []
            matrix_elem = root.find("CameraMatrix")
            for i in range(3):
                row = []
                for j in range(3):
                    elem = matrix_elem.find(f"m{i}{j}")
                    row.append(float(elem.text))
                camera_matrix.append(row)

            # 读取畸变系数
            distortion_coeffs = []
            dist_elem = root.find("DistortionCoefficients")
            i = 0
            while True:
                elem = dist_elem.find(f"k{i}")
                if elem is None:
                    break
                distortion_coeffs.append(float(elem.text))
                i += 1

            # 读取图像尺寸
            size_elem = root.find("ImageSize")
            width = int(size_elem.find("width").text)
            height = int(size_elem.find("height").text)

            return CameraIntrinsics(
                camera_matrix=camera_matrix,
                distortion_coeffs=distortion_coeffs,
                image_width=width,
                image_height=height,
            )

        except Exception as e:
            print(f"加载内参失败: {e}")
            return None

    def clear(self):
        """清除已收集的图像"""
        self.images = []
        self.corners_list = []
