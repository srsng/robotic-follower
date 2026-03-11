"""
坐标转换器

提供相机坐标系、机器人坐标系、像素坐标系之间的转换功能。
"""

import math
import numpy as np
from typing import List, Tuple, Optional, Dict
from geometry_msgs.msg import Point, Pose, PoseStamped
from sensor_msgs.msg import CameraInfo

from coordinate_transform.config.frame_names import FrameNames


class CoordinateConverter:
    """
    坐标转换器

    功能：
    - 像素坐标 <-> 相机光学坐标系
    - 相机光学坐标系 <-> 机器人坐标系
    - 像素坐标 -> 机器人坐标系（一步转换）
    - 深度图像处理
    """

    def __init__(self, tf_manager=None):
        """
        初始化坐标转换器

        Args:
            tf_manager: TFTreeManager 实例（可选，用于坐标系转换）
        """
        self.tf_manager = tf_manager

    # ==================== 相机内参相关 ====================

    @staticmethod
    def set_camera_intrinsics(camera_info: CameraInfo) -> None:
        """
        设置相机内参

        Args:
            camera_info: 相机信息消息
        """
        CoordinateConverter._camera_matrix = np.array(camera_info.k).reshape(3, 3)
        CoordinateConverter._distortion_coeffs = np.array(camera_info.d)

        # 提取内参
        fx = camera_info.k[0]
        fy = camera_info.k[4]
        cx = camera_info.k[2]
        cy = camera_info.k[5]

        CoordinateConverter._intrinsics = {
            'fx': fx,
            'fy': fy,
            'cx': cx,
            'cy': cy,
        }

    @staticmethod
    def get_camera_intrinsics() -> Dict[str, float]:
        """
        获取相机内参

        Returns:
            Dict[str, float]: 内参字典
        """
        if not hasattr(CoordinateConverter, '_intrinsics'):
            return {}

        return CoordinateConverter._intrinsics

    # ==================== 像素 <-> 相机光学坐标系 ====================

    @staticmethod
    def pixel_to_camera_optical(
        pixel: Tuple[int, int],
        depth: float,
        intrinsics: Optional[Dict[str, float]] = None,
    ) -> Tuple[float, float, float]:
        """
        像素坐标转相机光学坐标系 3D 点

        Args:
            pixel: (u, v) 像素坐标
            depth: 深度值（米）
            intrinsics: 相机内参，如果为 None 使用已设置的内参

        Returns:
            Tuple[float, float, float]: 相机光学坐标系 (x, y, z)
        """
        u, v = pixel

        if intrinsics is None:
            if not hasattr(CoordinateConverter, '_intrinsics'):
                raise ValueError("相机内参未设置，请先调用 set_camera_intrinsics() 或提供 intrinsics")
            intrinsics = CoordinateConverter._intrinsics

        fx, fy, cx, cy = intrinsics['fx'], intrinsics['fy'], intrinsics['cx'], intrinsics['cy']

        # 根据针孔相机模型计算 3D 坐标
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth

        return (x, y, z)

    @staticmethod
    def camera_optical_to_pixel(
        point: Tuple[float, float, float],
        intrinsics: Optional[Dict[str, float]] = None,
    ) -> Tuple[int, int]:
        """
        相机光学坐标系 3D 点转像素坐标

        Args:
            point: (x, y, z) 相机光学坐标系坐标
            intrinsics: 相机内参

        Returns:
            Tuple[int, int]: (u, v) 像素坐标
        """
        x, y, z = point

        if z <= 0:
            raise ValueError("z 坐标必须为正值")

        if intrinsics is None:
            if not hasattr(CoordinateConverter, '_intrinsics'):
                raise ValueError("相机内参未设置")
            intrinsics = CoordinateConverter._intrinsics

        fx, fy, cx, cy = intrinsics['fx'], intrinsics['fy'], intrinsics['cx'], intrinsics['cy']

        u = int(round(fx * x / z + cx))
        v = int(round(fy * y / z + cy))

        return (u, v)

    # ==================== 相机光学 -> 机器人坐标系 ====================

    def camera_optical_to_robot(
        point: Tuple[float, float, float],
        target_frame: str = FrameNames.BASE_LINK,
        source_frame: str = FrameNames.D435_COLOR_OPTICAL,
    ) -> Tuple[float, float, float]:
        """
        相机光学坐标系转机器人坐标系

        Args:
            point: 相机光学坐标系 (x, y, z)
            target_frame: 目标坐标系（默认机器人基座）
            source_frame: 源坐标系（默认相机光学坐标系）

        Returns:
            Tuple[float, float, float]: 机器人坐标系 (x, y, z)
        """
        if self.tf_manager is None:
            raise ValueError("需要 TFTreeManager 实例来执行坐标系转换")

        point_msg = Point(x=point[0], y=point[1], z=point[2])
        transformed = self.tf_manager.transform_point(target_frame, source_frame, point_msg)

        return (transformed.x, transformed.y, transformed.z)

    def robot_to_camera_optical(
        point: Tuple[float, float, float],
        target_frame: str = FrameNames.D435_COLOR_OPTICAL,
        source_frame: str = FrameNames.BASE_LINK,
    ) -> Tuple[float, float, float]:
        """
        机器人坐标系转相机光学坐标系

        Args:
            point: 机器人坐标系 (x, y, z)
            target_frame: 目标坐标系（默认相机光学坐标系）
            source_frame: 源坐标系（默认机器人基座）

        Returns:
            Tuple[float, float, float]: 相机光学坐标系 (x, y, z)
        """
        if self.tf_manager is None:
            raise ValueError("需要 TFTreeManager 实例来执行坐标系转换")

        point_msg = Point(x=point[0], y=point[1], z=point[2])
        transformed = self.tf_manager.transform_point(target_frame, source_frame, point_msg)

        return (transformed.x, transformed.y, transformed.z)

    # ==================== 像素 -> 机器人坐标系（一步转换）====================

    def pixel_to_robot(
        pixel: Tuple[int, int],
        depth: float,
        intrinsics: Optional[Dict[str, float]] = None,
        target_frame: str = FrameNames.BASE_LINK,
        source_frame: str = FrameNames.D435_COLOR_OPTICAL,
    ) -> Tuple[float, float, float]:
        """
        像素坐标直接转机器人坐标系

        这相当于：pixel -> camera_optical -> robot

        Args:
            pixel: (u, v) 像素坐标
            depth: 深度值（米）
            intrinsics: 相机内参
            target_frame: 目标坐标系
            source_frame: 源坐标系

        Returns:
            Tuple[float, float, float]: 机器人坐标系 (x, y, z)
        """
        # 步骤 1：像素 -> 相机光学坐标系
        camera_point = CoordinateConverter.pixel_to_camera_optical(
            pixel, depth, intrinsics
        )

        # 步骤 2：相机光学坐标系 -> 机器人坐标系
        robot_point = self.camera_optical_to_robot(
            camera_point, target_frame, source_frame
        )

        return robot_point

    # ==================== 批量转换 ====================

    @staticmethod
    def pixels_to_camera_optical(
        pixels: List[Tuple[int, int]],
        depths: List[float],
        intrinsics: Optional[Dict[str, float]] = None,
    ) -> List[Tuple[float, float, float]]:
        """
        批量像素坐标转相机光学坐标系

        Args:
            pixels: 像素坐标列表 [(u1, v1), (u2, v2), ...]
            depths: 深度值列表 [depth1, depth2, ...]
            intrinsics: 相机内参

        Returns:
            List[Tuple[float, float, float]]: 相机光学坐标系点列表
        """
        if len(pixels) != len(depths):
            raise ValueError("像素坐标和深度值数量不匹配")

        points = []
        for pixel, depth in zip(pixels, depths):
            points.append(
                CoordinateConverter.pixel_to_camera_optical(pixel, depth, intrinsics)
            )

        return points

    # ==================== 深度图像处理 ====================

    @staticmethod
    def get_depth_at_pixel(
        depth_image: np.ndarray,
        pixel: Tuple[int, int],
    ) -> float:
        """
        从深度图像中获取指定像素的深度值

        Args:
            depth_image: 深度图像（numpy 数组）
            pixel: (u, v) 像素坐标

        Returns:
            float: 深度值（米）
        """
        u, v = pixel
        height, width = depth_image.shape

        # 边界检查
        if u < 0 or u >= width or v < 0 or v >= height:
            raise ValueError(f"像素坐标 ({u}, {v}) 超出图像范围 ({width}x{height})")

        depth = float(depth_image[v, u])

        # 检查是否为无效深度值
        if depth <= 0 or depth != depth:  # NaN or invalid
            raise ValueError(f"像素 ({u}, {v}) 的深度值无效: {depth}")

        return depth

    @staticmethod
    def get_depth_at_pixel_safe(
        depth_image: np.ndarray,
        pixel: Tuple[int, int],
        default_depth: float = 0.0,
    ) -> float:
        """
        安全地从深度图像中获取指定像素的深度值

        Args:
            depth_image: 深度图像
            pixel: (u, v) 像素坐标
            default_depth: 默认深度值（当获取失败时返回）

        Returns:
            float: 深度值（米），失败时返回 default_depth
        """
        try:
            return CoordinateConverter.get_depth_at_pixel(depth_image, pixel)
        except (ValueError, IndexError):
            return default_depth

    # ==================== 像素边界检查 ====================

    @staticmethod
    def is_pixel_valid(
        pixel: Tuple[int, int],
        image_width: int,
        image_height: int,
        margin: int = 0,
    ) -> bool:
        """
        检查像素坐标是否在图像边界内

        Args:
            pixel: (u, v) 像素坐标
            image_width: 图像宽度
            image_height: 图像高度
            margin: 边界留白（像素）

        Returns:
            bool: 像素是否有效
        """
        u, v = pixel
        return (
            u >= margin and
            u < image_width - margin and
            v >= margin and
            v < image_height - margin
        )

    @staticmethod
    def get_valid_pixels_in_rect(
        rect: Tuple[int, int, int, int],  # (x, y, width, height)
        image_width: int,
        image_height: int,
        margin: int = 0,
    ) -> List[Tuple[int, int]]:
        """
        获取矩形区域内所有有效像素

        Args:
            rect: (x, y, width, height) 矩形区域
            image_width: 图像宽度
            image_height: 图像高度
            margin: 边界留白

        Returns:
            List[Tuple[int, int]]: 有效像素列表
        """
        x, y, w, h = rect
        pixels = []

        for v in range(y, y + h):
            for u in range(x, x + w):
                if CoordinateConverter.is_pixel_valid(
                    (u, v), image_width, image_height, margin
                ):
                    pixels.append((u, v))

        return pixels
