"""相机内参标定器"""

import os
import yaml
import numpy as np
from typing import List, Dict, Optional


class IntrinsicCalibrator:
    """相机内参标定器

    用于加载和验证相机标定结果
    """

    def __init__(self, config: dict):
        """初始化标定器

        Args:
            config: 标定配置字典
        """
        self.config = config
        self.calibration_file = None
        self.camera_info = None

    def load_calibration(self, file_path: str) -> bool:
        """加载标定文件

        Args:
            file_path: 标定文件路径 (.yaml)

        Returns:
            bool: 加载是否成功
        """
        if not os.path.exists(file_path):
            return False

        try:
            with open(file_path, 'r') as f:
                data = yaml.safe_load(f)

            self.camera_info = {
                'image_width': data.get('image_width', 640),
                'image_height': data.get('image_height', 480),
                'camera_matrix': {
                    'data': data.get('camera_matrix', {}).get('data', [])
                },
                'distortion_coefficients': {
                    'data': data.get('distortion_coefficients', {}).get('data', [])
                },
                'rectification_matrix': {
                    'data': data.get('rectification_matrix', {}).get('data', [])
                },
                'projection_matrix': {
                    'data': data.get('projection_matrix', {}).get('data', [])
                }
            }

            self.calibration_file = file_path
            return True

        except Exception as e:
            print(f"加载标定文件失败: {e}")
            return False

    def save_calibration(self, data: Dict, file_path: str) -> bool:
        """保存标定结果

        Args:
            data: 标定数据字典
            file_path: 保存文件路径

        Returns:
            bool: 保存是否成功
        """
        try:
            # 确保目录存在
            os.makedirs(os.path.dirname(file_path), exist_ok=True)

            with open(file_path, 'w') as f:
                yaml.dump(data, f, default)

            self.calibration_file = file_path
            return True

        except Exception as e:
            print(f"保存标定文件失败: {e}")
            return False

    def get_camera_matrix(self) -> Optional[np.ndarray]:
        """获取相机内参矩阵

        Returns:
            np.ndarray: 3x3 相机内参矩阵
        """
        if not self.camera_info:
            return None

        data = self.camera_info.get('camera_matrix', {}).get('data', [])
        if len(data) < 9:
            return None

        return np.array(data).reshape(3, 3)

    def get_distortion_coefficients(self) -> Optional[np.ndarray]:
        """获取畸变系数

        Returns:
            np.ndarray: 畸变系数数组
        """
        if not self.camera_info:
            return None

        data = self.camera_info.get('distortion_coefficients', {}).get('data', [])
        if len(data) == 0:
            return None

        return np.array(data)

    def get_reprojection_error(self) -> float:
        """获取重投影误差

        Returns:
            float: 重投影误差
        """
        if not self.camera_info:
            return float('inf')

        return self.camera_info.get('reprojection_error', float('inf'))

    def is_calibrated(self) -> bool:
        """检查是否已标定

        Returns:
            bool: 是否已标定
        """
        return self.calibration_file is not None and self.camera_info is not None

    def get_default_calibration_dir(self) -> str:
        """获取默认标定文件目录

        Returns:
            str: 标定文件目录路径
        """
        return os.path.expanduser('~/.ros/camera_info/')
