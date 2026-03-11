# -*- coding: utf-8 -*-
"""
聚类分割

使用cuML或scikit-learn的DBSCAN进行聚类，或使用欧式距离聚类
"""

import numpy as np
from scipy.spatial import cKDTree
from dataclasses import dataclass
from .base_segmentation import BaseSegmentation, SegmentationResult

# 尝试导入cuML（GPU加速）
try:
    from cuml.cluster import DBSCAN as cuDBSCAN
    CUMML_AVAILABLE = True
except ImportError:
    CUMML_AVAILABLE = False
    cuDBSCAN = None

from sklearn.cluster import DBSCAN as skDBSCAN


@dataclass
class ClusterResult(SegmentationResult):
    """聚类分割结果"""
    labels: np.ndarray             # 聚类标签 (N,), -1表示噪声
    num_clusters: int              # 聚类数量


class DBSCANClustering(BaseSegmentation):
    """
    DBSCAN聚类

    基于密度的聚类算法，自动确定聚类数量

    Args:
        eps: 邻域半径
        min_samples: 最小样本数
        use_gpu: 是否使用GPU加速（cuML）
    """

    def __init__(self, eps: float = 0.05, min_samples: int = 10, use_gpu: bool = True):
        super().__init__()
        self.eps = eps
        self.min_samples = min_samples
        self.use_gpu = use_gpu and CUMML_AVAILABLE

        # 初始化聚类器
        if self.use_gpu:
            self.clustering = cuDBSCAN(eps=eps, min_samples=min_samples)
        else:
            self.clustering = skDBSCAN(eps=eps, min_samples=min_samples)

    def segment(self, points: np.ndarray) -> ClusterResult:
        """
        应用DBSCAN聚类

        Args:
            points: 点云坐标 (N, 3)

        Returns:
            result: 聚类结果
        """
        if len(points) == 0:
            return ClusterResult(labels=np.array([]), num_clusters=0)

        # 应用聚类（直接调用cuML或sklearn API）
        labels = self.clustering.fit_predict(points)

        # 计算聚类数量（排除噪声点标签-1）
        num_clusters = len(set(labels)) - (1 if -1 in labels else 0)

        return ClusterResult(labels=labels, num_clusters=num_clusters)

    def get_cluster_points(self, points: np.ndarray, labels: np.ndarray) -> dict:
        """
        获取每个聚类的点云

        Args:
            points: 点云坐标 (N, 3)
            labels: 聚类标签 (N,)

        Returns:
            clusters: 字典 {cluster_id: cluster_points}
        """
        clusters = {}
        unique_labels = set(labels)

        for label in unique_labels:
            if label == -1:
                continue  # 跳过噪声点

            cluster_points = points[labels == label]
            clusters[int(label)] = cluster_points

        return clusters


class EuclideanClustering(BaseSegmentation):
    """
    欧式聚类

    基于欧式距离的聚类算法

    Args:
        cluster_tolerance: 聚类容差
        min_cluster_size: 最小聚类点数
        max_cluster_size: 最大聚类点数
    """

    def __init__(self, cluster_tolerance: float = 0.1, min_cluster_size: int = 100, max_cluster_size: int = 25000):
        super().__init__()
        self.cluster_tolerance = cluster_tolerance
        self.min_cluster_size = min_cluster_size
        self.max_cluster_size = max_cluster_size

    def segment(self, points: np.ndarray) -> ClusterResult:
        """
        应用欧式聚类

        Args:
            points: 点云坐标 (N, 3)

        Returns:
            result: 聚类结果
        """
        if len(points) == 0:
            return ClusterResult(labels=np.array([]), num_clusters=0)

        # 使用SciPy的cKDTree加速邻域搜索
        tree = cKDTree(points)

        # 初始化
        num_points = len(points)
        labels = np.full(num_points, -1, dtype=np.int32)
        processed = np.zeros(num_points, dtype=bool)
        cluster_id = 0

        for i in range(num_points):
            if processed[i]:
                continue

            # BFS聚类
            queue = [i]
            processed[i] = True

            while queue:
                idx = queue.pop(0)
                labels[idx] = cluster_id

                # 查找邻点
                neighbors = tree.query_ball_point(points[idx], self.cluster_tolerance)

                for neighbor in neighbors:
                    if not processed[neighbor] and labels[neighbor] == -1:
                        processed[neighbor] = True
                        queue.append(neighbor)

            # 检查聚类大小
            cluster_size = np.sum(labels == cluster_id)
            if cluster_size < self.min_cluster_size or cluster_size > self.max_cluster_size:
                # 聚类大小不符合要求，重置为噪声
                labels[labels == cluster_id] = -1
            else:
                cluster_id += 1

        num_clusters = cluster_id

        return ClusterResult(labels=labels, num_clusters=num_clusters)
