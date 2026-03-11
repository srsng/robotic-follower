"""
文件 I/O 工具

提供标定数据的保存和加载功能。
"""

import os
import cv2
import numpy as np
import xml.etree.ElementTree as ET
from typing import List, Optional
from dataclasses import dataclass


@dataclass
class CalibrationSession:
    """标定会话信息"""
    start_time: str
    end_time: str
    calibration_type: str
    num_images: int
    num_poses: int
    success: bool = False


class FileIO:
    """
    文件 I/O 工具

    功能：
    - 保存/加载标定结果
    - 保存/加载标定图像
    - 导出标定报告
    """

    @staticmethod
    def save_image(
        image: np.ndarray,
        filepath: str,
        compress: bool = True,
    ) -> bool:
        """
        保存图像

        Args:
            image: 图像
            filepath: 文件路径
            compress: 是否压缩

        Returns:
            bool: 是否成功保存
        """
        try:
            os.makedirs(os.path.dirname(filepath), exist_ok=True)
            ext = os.path.splitext(filepath)[1].lower()

            if compress and ext in ['.jpg', '.jpeg']:
                params = [cv2.IMWRITE_JPEG_QUALITY, 90]
            elif compress and ext == '.png':
                params = [cv2.IMWRITE_PNG_COMPRESSION, 5]
            else:
                params = []

            cv2.imwrite(filepath, image, params)
            return True

        except Exception as e:
            print(f"保存图像失败: {e}")
            return False

    @staticmethod
    def load_image(filepath: str) -> Optional[np.ndarray]:
        """
        加载图像

        Args:
            filepath: 文件路径

        Returns:
            Optional[np.ndarray]: 图像，失败返回 None
        """
        try:
            if not os.path.exists(filepath):
                print(f"文件不存在: {filepath}")
                return None

            image = cv2.imread(filepath)
            if image is None:
                print(f"无法读取图像: {filepath}")
                return None

            return image

        except Exception as e:
            print(f"加载图像失败: {e}")
            return None

    @staticmethod
    def load_images_from_directory(
        directory: str,
        pattern: str = "*.png",
    ) -> List[Tuple[str, np.ndarray]]:
        """
        从目录加载所有图像

        Args:
            directory: 目录路径
            pattern: 文件匹配模式

        Returns:
            List[Tuple[str, np.ndarray]]: (文件路径, 图像）列表
        """
        results = []

        try:
            import glob

            filepath_list = glob.glob(os.path.join(directory, pattern))

            for filepath in sorted(filepath_list):
                image = FileIO.load_image(filepath)
                if image is not None:
                    results.append((filepath, image))

        except Exception as e:
            print(f"加载图像目录失败: {e}")

        return results

    @staticmethod
    def save_yaml(data: dict, filepath: str) -> bool:
        """
        保存 YAML 文件

        Args:
            data: 数据字典
            filepath: 文件路径

        Returns:
            bool: 是否成功保存
        """
        try:
            import yaml

            os.makedirs(os.path.dirname(filepath), exist_ok=True)
            with open(filepath, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, allow_unicode=True)

            return True

        except Exception as e:
            print(f"保存 YAML 失败: {e}")
            return False

    @staticmethod
    def load_yaml(filepath: str) -> Optional[dict]:
        """
        加载 YAML 文件

        Args:
            filepath: 文件路径

        Returns:
            Optional[dict]: 数据字典，失败返回 None
        """
        try:
            import yaml

            if not os.path.exists(filepath):
                return None

            with open(filepath, 'r') as f:
                return yaml.safe_load(f)

        except Exception as e:
            print(f"加载 YAML 失败: {e}")
            return None

    @staticmethod
    def create_session_report(
        session: CalibrationSession,
        output_path: str,
    ) -> bool:
        """
        创建标定会话报告

        Args:
            session: 标定会话信息
            output_path: 输出文件路径

        Returns:
            bool: 是否成功创建
        """
        try:
            root = ET.Element("CalibrationSession")

            # 会话信息
            ET.SubElement(root, "StartTime").text = session.start_time
            ET.SubElement(root, "EndTime").text = session.end_time
            ET.SubElement(root, "CalibrationType").text = session.calibration_type
            ET.SubElement(root, "NumImages").text = str(session.num_images)
            ET.SubElement(root, "NumPoses").text = str(session.num_poses)
            ET.SubElement(root, "Success").text = str(session.success)

            # 保存文件
            tree = ET.ElementTree(root)
            tree.write(output_path, encoding="utf-8", xml_declaration=True)

            return True

        except Exception as e:
            print(f"创建会话报告失败: {e}")
            return False

    @staticmethod
    def ensure_directory(directory: str) -> bool:
        """
        确保目录存在

        Args:
            directory: 目录路径

        Returns:
            bool: 是否成功
        """
        try:
            os.makedirs(directory, exist_ok=True)
            return True
        except Exception as e:
            print(f"创建目录失败: {e}")
            return False

    @staticmethod
    def get_file_size(filepath: str) -> int:
        """
        获取文件大小

        Args:
            filepath: 文件路径

        Returns:
            int: 文件大小（字节），文件不存在返回 0
        """
        try:
            if os.path.exists(filepath):
                return os.path.getsize(filepath)
        except Exception:
            pass

        return 0

    @staticmethod
    def get_directory_size(directory: str) -> int:
        """
        获取目录总大小

        Args:
            directory: 目录路径

        Returns:
            int: 目录大小（字节）
        """
        try:
            total_size = 0

            for dirpath, dirnames, filenames in os.walk(directory):
                for filename in filenames:
                    filepath = os.path.join(dirpath, filename)
                    total_size += os.path.getsize(filepath)

            return total_size

        except Exception:
            return 0
