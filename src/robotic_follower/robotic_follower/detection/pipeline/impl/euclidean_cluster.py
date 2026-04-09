"""欧式聚簇算法"""

import numpy as np

from ..data import PipelineData
from ..registry import StageRegistry
from ..stages import AlgorithmStage


@StageRegistry.register_algorithm("euclidean_cluster", class_names=["cluster"])
class EuclideanCluster(AlgorithmStage):
    """欧式聚簇：对剩余点云做聚类，每个簇作为一个目标"""

    def __init__(
        self,
        tolerance: float = 0.2,
        min_cluster_size: int = 10,
        min_neighbors: int = 3,
        parent_node=None,
    ):
        super().__init__("euclidean_cluster", parent_node=parent_node)
        self.tolerance = tolerance
        self.min_cluster_size = min_cluster_size
        self.min_neighbors = min_neighbors

    def detect(self, data: PipelineData) -> PipelineData:
        """执行聚类检测"""
        self._debug(f"EuclideanCluster 输入: points={len(data.points)}")
        if len(data.points) == 0:
            return data

        # 获取剩余点
        remaining_mask = data.point_mask.copy()
        remaining_points = data.points[remaining_mask]
        self._debug(f"EuclideanCluster 剩余点: {len(remaining_points)}")

        if len(remaining_points) == 0:
            return data

        # 执行聚类
        clusters = self._cluster(remaining_points)
        self._debug(f"EuclideanCluster 聚类结果: {len(clusters)} 簇")

        # 为每个簇创建检测结果
        for cluster_points, cluster_indices in clusters:
            if len(cluster_points) < self.min_cluster_size:
                continue

            # cluster_indices 是在 remaining_points 中的索引
            # 需要通过 data.original_indices 转换为原始索引
            remaining_indices = data.original_indices[np.where(remaining_mask)[0]]
            actual_indices = remaining_indices[cluster_indices]

            # 更新 labels
            if len(data.labels) == 0:
                data.labels = np.full(len(data.points), -1, dtype=np.int32)
            data.labels[actual_indices] = 0

            # 创建检测结果
            detection = self.make_detection(
                points=cluster_points,
                name="cluster",
                score=min(1.0, len(cluster_points) / 100),
                point_indices=actual_indices,
            )
            data.detections.append(detection)

            # 标记这些点为已处理（先复制避免修改原始掩码）
            data.point_mask = data.point_mask.copy()
            data.point_mask[actual_indices] = False

        return data

    def _cluster(self, points: np.ndarray) -> list:
        """使用欧式距离聚类

        Returns:
            list of (cluster_points, cluster_indices) tuples
        """
        from scipy.spatial import KDTree

        if len(points) < self.min_cluster_size:
            return []

        tree = KDTree(points)
        clusters = []
        visited = np.zeros(len(points), dtype=bool)

        for i in range(len(points)):
            if visited[i]:
                continue

            # 使用种子区域生长
            cluster = []
            seed = [i]
            visited[i] = True

            while seed:
                current = seed.pop()
                cluster.append(current)

                # 找邻居
                neighbors = tree.query_ball_point(points[current], r=self.tolerance)
                for neighbor in neighbors:
                    if not visited[neighbor]:
                        visited[neighbor] = True
                        seed.append(neighbor)

            if len(cluster) >= self.min_cluster_size:
                cluster_points = points[cluster]
                clusters.append((cluster_points, np.array(cluster)))

        return clusters


__all__ = ["EuclideanCluster"]
