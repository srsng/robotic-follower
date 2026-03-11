# -*- coding: utf-8 -*-
"""
ROS2消息转换

提供NumPy数组与ROS2消息之间的转换
"""

import numpy as np
from typing import Tuple, Optional
import struct

try:
    from sensor_msgs.msg import PointCloud2, PointField
    from sensor_msgs.msg import Image, CameraInfo
    from std_msgs.msg import Header
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    PointCloud2 = None
    PointField = None
    Image = None
    CameraInfo = None
    Header = None


def numpy_to_pointcloud2(
    points: np.ndarray,
    header: Optional = None,
    frame_id: str = "map",
    colors: Optional[np.ndarray] = None
) -> Optional:
    """
    将NumPy数组转换为ROS2 PointCloud2消息

    Args:
        points: 点云坐标 (N, 3)
        header: 消息头
        frame_id: 坐标系ID
        colors: 颜色 (N, 3)，可选

    Returns:
        PointCloud2消息，如果ROS未安装则返回None
    """
    if not ROS_AVAILABLE:
        return None

    num_points = len(points)

    # 创建消息头
    if header is None:
        header = Header()
        header.frame_id = frame_id

    # 创建PointCloud2消息
    cloud_msg = PointCloud2()
    cloud_msg.header = header
    cloud_msg.height = 1
    cloud_msg.width = num_points
    cloud_msg.is_bigendian = False
    cloud_msg.is_dense = True

    # 定义字段
    cloud_msg.fields = [
        PointField(name='x', offset=0, datatype=7, count=1),
        PointField(name='y', offset=4, datatype=7, count=1),
        PointField(name='z', offset=8, datatype=7, count=1),
    ]

    if colors is not None:
        cloud_msg.fields.extend([
            PointField(name='r', offset=12, datatype=7, count=1),
            PointField(name='g', offset=16, datatype=7, count=1),
            PointField(name='b', offset=20, datatype=7, count=1),
        ])

    # 计算点步长和大小
    point_step = 12 if colors is None else 24
    cloud_msg.point_step = point_step
    cloud_msg.row_step = point_step * num_points
    cloud_msg.data = (num_points * point_step).to_bytes()

    # 填充数据
    for i in range(num_points):
        x, y, z = points[i]
        # 写入坐标 (float32)
        cloud_msg.data[i*point_step:(i*point_step+12)] = struct.pack('3f', x, y, z)

        if colors is not None:
            r, g, b = colors[i]
            # 写入颜色 (uint8)
            cloud_msg.data[i*point_step+12:(i*point_step+24)] = struct.pack('3B', r, g, b)

    return cloud_msg


def pointcloud2_to_numpy(cloud_msg) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """
    将ROS2 PointCloud2消息转换为NumPy数组

    Args:
        cloud_msg: PointCloud2消息

    Returns:
        points: 点云坐标 (N, 3)
        colors: 颜色 (N, 3)，如果有颜色则返回，否则返回None
    """
    if not ROS_AVAILABLE:
        return np.array([]), None

    # 提取字段信息
    field_names = [f.name for f in cloud_msg.fields]
    has_colors = all(c in field_names for c in ['r', 'g', 'b'])

    # 解析数据
    points = []
    colors_list = []

    for i in range(cloud_msg.height * cloud_msg.width):
        offset = i * cloud_msg.point_step

        # 提取坐标
        x = struct.unpack('f', cloud_msg.data[offset:offset+4])[0]
        y = struct.unpack('f', cloud_msg.data[offset+4:offset+8])[0]
        z = struct.unpack('f', cloud_msg.data[offset+8:offset+12])[0]
        points.append([x, y, z])

        # 提取颜色
        if has_colors:
            r = cloud_msg.data[offset+12]
            g = cloud_msg.data[offset+13]
            b = cloud_msg.data[offset+14]
            colors_list.append([r, g, b])

    points = np.array(points, dtype=np.float32)
    colors = np.array(colors_list, dtype=np.uint8) if has_colors else None

    return points, colors


def depth_to_pointcloud(
    depth_image: np.ndarray,
    camera_info: CameraInfo,
    depth_scale: float = 0.001,
    invalid_depth_value: float = 0.0,
    rgb_image: Optional[np.ndarray] = None
) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """
    将深度图像转换为点云

    Args:
        depth_image: 深度图像 (H, W)
        camera_info: 相机内参
        depth_scale: 深度缩放因子
        invalid_depth_value: 无效深度值
        rgb_image: RGB图像 (H, W, 3)，可选

    Returns:
        points: 点云坐标 (N, 3)
        colors: 颜色 (N, 3)，如果有RGB图像则返回
    """
    height, width = depth_image.shape

    # 提取相机内参
    fx = camera_info.k[0]
    fy = camera_info.k[4]
    cx = camera_info.k[2]
    cy = camera_info.k[5]

    # 创建像素坐标网格
    v, u = np.meshgrid(np.arange(height), np.arange(width), indexing='ij')

    # 转换深度值
    depth = depth_image.astype(np.float32) * depth_scale

    # 移除无效深度
    valid_mask = (depth > 0) & (depth != invalid_depth_value)

    # 计算点云坐标
    z = depth[valid_mask]
    x = (u[valid_mask] - cx) * z / fx
    y = (v[valid_mask] - cy) * z / fy

    points = np.stack([x, y, z], axis=1)

    # 提取颜色
    colors = None
    if rgb_image is not None:
        colors = rgb_image[valid_mask].astype(np.uint8)

    return points, colors


def pointcloud_to_depth(
    points: np.ndarray,
    camera_info: CameraInfo,
    image_size: Tuple[int, int] = None
) -> np.ndarray:
    """
    将点云投影回深度图像

    Args:
        points: 点云坐标 (N, 3)
        camera_info: 相机内参
        image_size: 输出图像尺寸 (H, W)，默认使用相机参数

    Returns:
        depth_image: 深度图像 (H, W)
    """
    # 提取相机内参
    fx = camera_info.k[0]
    fy = camera_info.k[4]
    cx = camera_info.k[2]
    cy = camera_info.k[5]

    if image_size is None:
        image_size = (int(camera_info.height), int(camera_info.width))

    # 创建深度图像
    depth_image = np.zeros(image_size, dtype=np.float32)

    # 投影点云到图像平面
    for x, y, z in points:
        if z <= 0:
            continue

        u = int(x * fx / z + cx)
        v = int(y * fy / z + cy)

        # 检查边界
        if 0 <= u < image_size[1] and 0 <= v < image_size[0]:
            depth_image[v, u] = z

    return depth_image
