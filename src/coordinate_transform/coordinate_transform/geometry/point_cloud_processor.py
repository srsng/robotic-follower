"""
点云处理器

提供点云滤波、降采样、特征提取等功能。
"""

import numpy as np
from typing import List, Tuple, Optional
from sensor_msgs.msg import PointCloud2, PointField
import struct


class PointCloudProcessor:
    """
    点云处理器

    功能：
    - 点云滤波（距离、范围）
    - 点云降采样（体素）
    - 统计滤波
    - 点云转换（ROS2 <-> numpy）
    - 特征提取（质心、包围盒）
    """

    @staticmethod
    def pointcloud2_to_numpy(pointcloud: PointCloud2) -> np.ndarray:
        """
        将 ROS2 PointCloud2 转换为 numpy 数组

        Args:
            pointcloud: ROS2 点云消息

        Returns:
            np.ndarray: Nx3 numpy 数组 [x, y, z]
        """
        points = []

        # 解析点云数据
        field_names = [field.name for field in pointcloud.fields]
        point_step = pointcloud.point_step
        row_step = pointcloud.row_step
        data = pointcloud.data

        # 查找 x, y, z 字段偏移
        x_offset = pointcloud.fields[field_names.index('x')].offset
        y_offset = pointcloud.fields[field_names.index('y')].offset
        z_offset = pointcloud.fields[field_names.index('z')].offset

        for row in range(pointcloud.height):
            row_data = data[row * row_step : (row + 1) * row_step]

            for col in range(pointcloud.width):
                point_data = row_data[col * point_step : (col + 1) * point_step]

                # 提取 x, y, z
                x = struct.unpack('f', point_data[x_offset:x_offset+4])[0]
                y = struct.unpack('f', point_data[y_offset:y_offset+4])[0]
                z = struct.unpack('f', point_data[z_offset:z_offset+4])[0]

                # 检查是否为 NaN 或 Inf
                if not (np.isnan(x) or np.isnan(y) or np.isnan(z) or
                        np.isinf(x) or np.isinf(y) or np.isinf(z)):
                    points.append([x, y, z])

        return np.array(points, dtype=np.float32)

    @staticmethod
    def numpy_to_pointcloud2(points: np.ndarray, frame_id: str = "camera_optical") -> PointCloud2:
        """
        将 numpy 数组转换为 ROS2 PointCloud2

        Args:
            points: Nx3 numpy 数组
            frame_id: 坐标系名称

        Returns:
            PointCloud2: ROS2 点云消息
        """
        pointcloud = PointCloud2()

        # 设置头信息
        pointcloud.header.frame_id = frame_id
        pointcloud.height = 1
        pointcloud.width = len(points)

        # 设置字段
        pointcloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        pointcloud.is_bigendian = False
        pointcloud.point_step = 12  # 3 * 4 bytes
        pointcloud.row_step = pointcloud.point_step * pointcloud.width

        # 填充数据
        for point in points:
            pointcloud.data.extend(struct.pack('fff', point[0], point[1], point[2]))

        return pointcloud

    @staticmethod
    def filter_by_distance(
        pointcloud: PointCloud2,
        min_distance: float = 0.1,
        max_distance: float = 10.0,
    ) -> PointCloud2:
        """
        按距离滤波点云

        Args:
            pointcloud: 输入点云
            min_distance: 最小距离（米）
            max_distance: 最大距离（米）

        Returns:
            PointCloud2: 滤波后的点云
        """
        points = PointCloudProcessor.pointcloud2_to_numpy(pointcloud)

        # 计算距离并滤波
        distances = np.linalg.norm(points, axis=1)
        mask = (distances >= min_distance) & (distances <= max_distance)
        filtered_points = points[mask]

        return PointCloudProcessor.numpy_to_pointcloud2(
            filtered_points,
            pointcloud.header.frame_id,
        )

    @staticmethod
    def filter_by_range(
        pointcloud: PointCloud2,
        x_range: Tuple[float, float] = (-10.0, 10.0),
        y_range: Tuple[float, float] = (-10.0, 10.0),
        z_range: Tuple[float, float] = (-10.0, 10.0),
    ) -> PointCloud2:
        """
        按坐标范围滤波点云

        Args:
            pointcloud: 输入点云
            x_range: X 轴范围 (min, max)
            y_range: Y 轴范围 (min, max)
            z_range: Z 轴范围 (min, max)

        Returns:
            PointCloud2: 滤波后的点云
        """
        points = PointCloudProcessor.pointcloud2_to_numpy(pointcloud)

        # 坐标范围滤波
        mask = (
            (points[:, 0] >= x_range[0]) & (points[:, 0] <= x_range[1]) &
            (points[:, 1] >= y_range[0]) & (points[:, 1] <= y_range[1]) &
            (points[:, 2] >= z_range[0]) & (points[:, 2] <= z_range[1])
        )
        filtered_points = points[mask]

        return PointCloudProcessor.numpy_to_pointcloud2(
            filtered_points,
            pointcloud.header.frame_id,
        )

    @staticmethod
    def voxel_downsample(
        pointcloud: PointCloud2,
        voxel_size: float = 0.01,
    ) -> PointCloud2:
        """
        体素降采样点云

        Args:
            pointcloud: 输入点云
            voxel_size: 体素大小（米）

        Returns:
            PointCloud2: 降采样后的点云
        """
        points = PointCloudProcessor.pointcloud2_to_numpy(pointcloud)

        if len(points) == 0:
            return PointCloudProcessor.numpy_to_pointcloud2(
                points,
                pointcloud.header.frame_id,
            )

        # 计算体素索引
        voxel_indices = np.floor(points / voxel_size).astype(np.int32)

        # 使用字典存储每个体素的代表点（第一个点）
        voxel_dict = {}
        for i, idx in enumerate(voxel_indices):
            key = tuple(idx)
            if key not in voxel_dict:
                voxel_dict[key] = points[i]

        downsampled_points = np.array(list(voxel_dict.values()))

        return PointCloudProcessor.numpy_to_pointcloud2(
            downsampled_points,
            pointcloud.header.frame_id,
        )

    @staticmethod
    def statistical_outlier_removal(
        pointcloud: PointCloud2,
        k_neighbors: int = 20,
        std_ratio: float = 1.0,
    ) -> PointCloud2:
        """
        统计离群点滤波

        Args:
            pointcloud: 输入点云
            k_neighbors: 邻居点数
            std_ratio: 标准差倍数

        Returns:
            PointCloud2: 滤波后的点云
        """
        points = PointCloudProcessor.pointcloud2_to_numpy(pointcloud)

        if len(points) < k_neighbors:
            return pointcloud

        # 计算每个点到其 k 个最近邻居的平均距离
        from sklearn.neighbors import NearestNeighbors

        nbrs = NearestNeighbors(n_neighbors=k_neighbors + 1, algorithm='ball_tree')
        nbrs.fit(points)
        distances, _ = nbrs.kneighbors(points)

        # 跳过自身（距离为0的第一个邻居）
        mean_distances = np.mean(distances[:, 1:], axis=1)

        # 计算全局均值和标准差
        global_mean = np.mean(mean_distances)
        global_std = np.std(mean_distances)

        # 滤波离群点
        threshold = global_mean + std_ratio * global_std
        mask = mean_distances < threshold
        filtered_points = points[mask]

        return PointCloudProcessor.numpy_to_pointcloud2(
            filtered_points,
            pointcloud.header.frame_id,
        )

    @staticmethod
    def calculate_centroid(pointcloud: PointCloud2) -> Tuple[float, float, float]:
        """
        计算点云质心

        Args:
            pointcloud: 输入点云

        Returns:
            Tuple[float, float, float]: 质心坐标 (x, y, z)
        """
        points = PointCloudProcessor.pointcloud2_to_numpy(pointcloud)

        if len(points) == 0:
            return (0.0, 0.0, 0.0)

        centroid = np.mean(points, axis=0)
        return (float(centroid[0]), float(centroid[1]), float(centroid[2]))

    @staticmethod
    def calculate_bounding_box(pointcloud: PointCloud2) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        """
        计算点云包围盒

        Args:
            pointcloud: 输入点云

        Returns:
            Tuple[Point, Point]: (最小点, 最大点)
        """
        points = PointCloudProcessor.pointcloud2_to_numpy(pointcloud)

        if len(points) == 0:
            return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

        min_pt = np.min(points, axis=0)
        max_pt = np.max(points, axis=0)

        return (
            (float(min_pt[0]), float(min_pt[1]), float(min_pt[2])),
            (float(max_pt[0]), float(max_pt[1]), float(max_pt[2])),
        )
