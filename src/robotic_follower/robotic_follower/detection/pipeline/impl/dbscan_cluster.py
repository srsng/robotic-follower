"""DBSCAN 聚类算法

基于密度的空间聚类算法，适合检测不同密度的细小物体。
"""

import numpy as np

from ..data import PipelineData
from ..registry import StageRegistry
from ..stages import AlgorithmStage


@StageRegistry.register_algorithm("dbscan_cluster", class_names=["cluster"])
class DBSCANCluster(AlgorithmStage):
    """DBSCAN 聚类：对剩余点云做密度聚类，每个簇作为一个目标

    算法参数：
        eps: 邻域半径 (m)，距离小于此值的点被视为邻居
        min_samples: 核心点的最小邻域数，少于此则为噪声点

    优点：
        - 自动发现任意形状的簇
        - 噪声检测（min_samples 以下的点被视为噪声，不输出）
        - 不需要预设簇数量
        - 对细小物体更友好（通过 eps 控制局部密度）
    """

    def __init__(
        self,
        eps: float = 0.025,
        min_samples: int = 3,
        parent_node=None,
    ):
        super().__init__("dbscan_cluster", parent_node=parent_node)
        self.eps = eps
        self.min_samples = min_samples

    def detect(self, data: PipelineData) -> PipelineData:
        """执行 DBSCAN 聚类检测"""
        self._info(
            f"DBSCANCluster 输入: total_points={len(data.points)}, "
            f"remaining_after_ground={np.sum(data.point_mask) if len(data.point_mask) > 0 else 0}"
        )
        if len(data.points) == 0:
            return data

        # 获取剩余点
        remaining_mask = data.point_mask.copy()
        remaining_points = data.points[remaining_mask]
        self._info(f"DBSCANCluster 剩余点: {len(remaining_points)}")

        if len(remaining_points) == 0:
            self._info("DBSCANCluster: 无剩余点，跳过")
            return data

        # 打印剩余点的 Z 范围（帮助判断地面移除效果）
        z_remaining = remaining_points[:, 2]
        self._info(
            f"DBSCANCluster Z 范围: "
            f"min={float(z_remaining.min()):.4f}, "
            f"max={float(z_remaining.max()):.4f}, "
            f"mean={float(z_remaining.mean()):.4f}"
        )

        # 执行 DBSCAN 聚类
        labels = self._dbscan(remaining_points)
        n_clusters = len(set(labels)) - (1 if -1 in labels else 0)
        n_noise = int(np.sum(labels == -1))
        self._info(
            f"DBSCANCluster 聚类结果: {n_clusters} 簇, {n_noise} 噪声点 "
            f"(eps={self.eps}, min_samples={self.min_samples})"
        )

        # 为每个簇创建检测结果
        unique_labels = set(labels)
        for label in unique_labels:
            if label == -1:  # 噪声点，跳过
                continue

            cluster_mask = labels == label
            cluster_points = remaining_points[cluster_mask]
            cluster_indices_in_remaining = np.where(cluster_mask)[0]

            # 更新 labels（在 original data.points 中）
            if len(data.labels) == 0:
                data.labels = np.full(len(data.points), -1, dtype=np.int32)
            actual_indices = np.where(remaining_mask)[0][cluster_indices_in_remaining]
            data.labels[actual_indices] = label

            # 创建检测结果
            detection = self.make_detection(
                points=cluster_points,
                name=f"cluster_{label}",
                score=min(1.0, len(cluster_points) / 50),
                point_indices=actual_indices,
            )
            data.detections.append(detection)

            # 打印每个簇的详细信息
            c_z = cluster_points[:, 2]
            self._info(
                f"  簇 {label}: {len(cluster_points)} 点, "
                f"Z=[{float(c_z.min()):.4f}, {float(c_z.max()):.4f}], "
                f"bbox_size={detection['bbox'][3:6].tolist()}"
            )

        return data

    def _dbscan(self, points: np.ndarray) -> np.ndarray:
        """DBSCAN 聚类实现

        Args:
            points: 输入点云 (N, 3)

        Returns:
            labels: 每个点的簇标签，-1 表示噪声
        """
        from scipy.spatial import KDTree

        n = len(points)
        labels = np.full(n, -1, dtype=np.int32)
        cluster_id = 0

        if n < self.min_samples:
            return labels

        # 构建 KDTree 用于快速邻域查询
        tree = KDTree(points)

        for i in range(n):
            if labels[i] != -1:  # 已访问
                continue

            # 找邻居
            neighbors = tree.query_ball_point(points[i], r=self.eps)
            if len(neighbors) < self.min_samples:
                continue  # 不是核心点，标记为噪声（后续可能被其他点合并）

            # 开始扩散
            labels[i] = cluster_id
            seed = list(neighbors)

            while seed:
                j = seed.pop()
                if labels[j] == -1:  # 之前是噪声，现在成为边界点
                    labels[j] = cluster_id

                if labels[j] != -1:  # 已访问
                    continue

                labels[j] = cluster_id
                j_neighbors = tree.query_ball_point(points[j], r=self.eps)
                if len(j_neighbors) >= self.min_samples:
                    seed.extend(j_neighbors)

            cluster_id += 1

        return labels


__all__ = ["DBSCANCluster"]
