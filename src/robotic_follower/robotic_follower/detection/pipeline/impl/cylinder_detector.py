"""圆柱体检测算法

检测竖直放置在水平面上的圆柱体（轴线垂直于水平面）。
输入点云已过滤掉水平面（地面）点云。

算法逻辑：
1. 卡尔曼滤波平滑点云
2. 忽略高度，在2D平面(XY)上检测圆弧
3. 检查圆弧所在圆内是否无其他点，确认是圆柱
4. 检测顶部（如可见）：水平点云层作为顶面，否则用最高点
5. 计算圆柱高度、包围盒
"""

import numpy as np
from scipy.signal import savgol_filter
from scipy.spatial import KDTree

from ..data import PipelineData
from ..registry import StageRegistry
from ..stages import AlgorithmStage


@StageRegistry.register_algorithm("cylinder_detector")
class CylinderDetector(AlgorithmStage):
    """圆柱体检测：检测竖直放置的圆柱体

    圆柱体特点：
    - 轴线垂直于水平面
    - 单视角下呈圆弧状
    - 从顶部看，圆弧所在圆内应无其他点
    """

    def __init__(
        self,
        # 卡尔曼滤波参数
        kalman_process_noise: float = 0.01,
        kalman_measurement_noise: float = 0.1,
        # 圆弧检测参数
        min_arc_angle: float = np.pi / 6,  # 最小弧度（30度）
        max_arc_angle: float = np.pi * 2,  # 最大弧度（可整圆）
        arc_cluster_tolerance: float = 0.15,  # 圆弧聚类容差
        min_arc_points: int = 15,
        # 圆柱体验证参数
        radius_tolerance: float = 0.08,
        min_radius: float = 0.03,
        max_radius: float = 0.5,
        hollow_tolerance: float = 0.05,  # 圆内检测空白区的容差
        # 高度相关参数
        min_height: float = 0.1,
        max_height: float = 2.5,
        top_layer_thickness: float = 0.05,  # 顶面层厚度
        top_layer_min_points: int = 10,
        # 聚类参数
        cluster_tolerance: float = 0.1,
        min_cluster_points: int = 20,
        parent_node=None,
    ):
        super().__init__("cylinder_detector", parent_node=parent_node)
        # 卡尔曼滤波参数
        self.kalman_process_noise = kalman_process_noise
        self.kalman_measurement_noise = kalman_measurement_noise
        # 圆弧检测参数
        self.min_arc_angle = min_arc_angle
        self.max_arc_angle = max_arc_angle
        self.arc_cluster_tolerance = arc_cluster_tolerance
        self.min_arc_points = min_arc_points
        # 圆柱体验证参数
        self.radius_tolerance = radius_tolerance
        self.min_radius = min_radius
        self.max_radius = max_radius
        self.hollow_tolerance = hollow_tolerance
        # 高度参数
        self.min_height = min_height
        self.max_height = max_height
        self.top_layer_thickness = top_layer_thickness
        self.top_layer_min_points = top_layer_min_points
        # 聚类参数
        self.cluster_tolerance = cluster_tolerance
        self.min_cluster_points = min_cluster_points

    def detect(self, data: PipelineData) -> PipelineData:
        """检测圆柱体"""
        if len(data.points) == 0:
            return data

        # 获取非地面点
        remaining_points = data.get_remaining_points()
        if len(remaining_points) < self.min_cluster_points:
            return data

        # 获取原始索引
        original_indices = np.where(data.point_mask)[0]

        # Step 1: 卡尔曼滤波平滑点云
        filtered_points, filter_indices = self._kalman_filter(remaining_points)

        # Step 2: 在XY平面检测圆弧
        arc_candidates = self._detect_arcs_2d(filtered_points)

        if len(arc_candidates) == 0:
            return data

        # Step 3: 对每个圆弧候选验证是否为圆柱
        for arc in arc_candidates:
            center, radius, arc_angle, arc_indices = arc

            # 验证半径是否合理
            if not (self.min_radius <= radius <= self.max_radius):
                continue

            # 获取圆弧对应的高度范围
            arc_points_3d = filtered_points[arc_indices]
            z_min = arc_points_3d[:, 2].min()
            z_max = arc_points_3d[:, 2].max()
            z_range = z_max - z_min

            # Step 4: 检查圆内是否有其他点（验证圆柱是空心的）
            if not self._check_cylinder_hollow(
                filtered_points, center, radius, z_min, z_max, arc_indices
            ):
                continue

            # Step 5: 检测顶部
            top_z = self._detect_top(filtered_points, center, radius, z_max)

            # 计算圆柱高度
            height = top_z - z_min
            if height < self.min_height or height > self.max_height:
                continue

            # Step 6: 获取属于该圆柱的原始点
            inlier_indices = self._get_cylinder_inliers(
                remaining_points,
                center,
                radius,
                z_min,
                top_z,
                original_indices,
                filter_indices,
            )

            if len(inlier_indices) < self.min_cluster_points:
                continue

            inlier_points = remaining_points[inlier_indices]

            # Step 7: 计算更精确的轴心（使用2D圆心作为XY，Z用中位数）
            axis_xy = center
            axis_z = (z_min + top_z) / 2

            # 创建检测结果
            detection = self.make_detection(
                points=inlier_points,
                name="cylinder",
                score=min(1.0, arc_angle / (np.pi * 2) * 2),
            )
            detection["axis_center"] = np.array([axis_xy[0], axis_xy[1], axis_z])
            detection["radius"] = radius
            detection["height"] = height
            detection["top_z"] = top_z
            detection["bottom_z"] = z_min
            detection["arc_angle"] = arc_angle

            data.detections.append(detection)

            # 从 point_mask 中排除这些点
            data.point_mask = data.point_mask.copy()
            data.point_mask[inlier_indices] = False

        return data

    def _kalman_filter(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """卡尔曼滤波平滑点云

        使用简化的1D卡尔曼滤波分别对X, Y, Z坐标滤波。
        """
        n_points = len(points)
        if n_points < 3:
            return points, np.arange(n_points)

        # 状态维度为2（位置和速度）
        state_dim = 2
        # 测量维度为1（只有位置可测量）
        meas_dim = 1

        # 状态转移矩阵（匀速模型）
        F = np.array([[1, 1], [0, 1]])
        # 测量矩阵
        H = np.array([[1, 0]])
        # 过程噪声协方差
        Q = self.kalman_process_noise**2 * np.eye(state_dim)
        # 测量噪声协方差
        R = self.kalman_measurement_noise**2

        # 初始状态协方差
        P = np.eye(state_dim)

        filtered = points.copy()

        for dim in range(3):
            x = points[0, dim]
            v = 0.0
            state = np.array([x, v])

            for i in range(n_points):
                z = points[i, dim]

                # 预测
                state = F @ state
                P = F @ P @ F.T + Q

                # 更新
                S = H @ P @ H.T + R
                K = P @ H.T @ np.linalg.inv(S)

                y = z - H @ state
                state = state + K.flatten() * y

                P = (np.eye(state_dim) - K @ H) @ P

                filtered[i, dim] = state[0]

        # 计算保留点的索引（这里保留所有点）
        keep_indices = np.arange(n_points)

        # 使用Savitzky-Golay滤波器进一步平滑（保边）
        if n_points > 5:
            window = min(5, n_points if n_points % 2 == 1 else n_points - 1)
            for dim in range(3):
                filtered[:, dim] = savgol_filter(filtered[:, dim], window, 2)

        return filtered, keep_indices

    def _detect_arcs_2d(self, points: np.ndarray) -> list:
        """在XY平面检测圆弧

        Returns:
            list of (center_2d, radius, arc_angle, arc_indices)
        """
        if len(points) < self.min_arc_points:
            return []

        # 获取2D点
        points_2d = points[:, :2]

        # Step 1: 在XY平面聚类
        clusters = self._cluster_2d(
            points_2d, self.arc_cluster_tolerance, self.min_arc_points
        )

        arc_candidates = []

        for cluster_indices, cluster_mask in clusters:
            cluster_points_2d = points_2d[cluster_indices]

            # Step 2: 对每个聚类尝试拟合圆
            if len(cluster_points_2d) < self.min_arc_points:
                continue

            circle = self._fit_circle_ransac(cluster_points_2d)
            if circle is None:
                continue

            center, radius = circle

            # Step 3: 计算弧度
            # 计算每个点到圆心的角度
            vectors = cluster_points_2d - center
            angles = np.arctan2(vectors[:, 1], vectors[:, 0])

            # 去除靠近圆心的点（可能是杂点）
            distances = np.linalg.norm(vectors, axis=1)
            dist_std = np.std(distances)
            if dist_std > self.radius_tolerance * 3:
                continue

            # 计算弧度范围
            angle_range = self._compute_angle_coverage(angles)

            # 验证是否为有效圆弧
            if angle_range < self.min_arc_angle:
                continue

            arc_candidates.append((center, radius, angle_range, cluster_indices))

        return arc_candidates

    def _cluster_2d(self, points: np.ndarray, tolerance: float, min_size: int) -> list:
        """在2D空间做欧式聚类"""
        if len(points) < min_size:
            return []

        tree = KDTree(points)

        visited = np.zeros(len(points), dtype=bool)
        clusters = []

        for i in range(len(points)):
            if visited[i]:
                continue

            cluster = [i]
            queue = [i]
            visited[i] = True

            while queue:
                current = queue.pop()
                neighbors = tree.query_ball_point(points[current], r=tolerance)
                for neighbor in neighbors:
                    if not visited[neighbor]:
                        visited[neighbor] = True
                        cluster.append(neighbor)
                        queue.append(neighbor)

            if len(cluster) >= min_size:
                cluster_mask = np.zeros(len(points), dtype=bool)
                cluster_mask[cluster] = True
                clusters.append((np.array(cluster), cluster_mask))

        return clusters

    def _fit_circle_ransac(
        self,
        points: np.ndarray,
        ransac_iterations: int = 50,
        inlier_threshold: float = 0.05,
    ) -> tuple[np.ndarray, float] | None:
        """使用RANSAC拟合圆"""
        best_inliers = 0
        best_circle = None

        n_points = len(points)

        for _ in range(ransac_iterations):
            # 随机选择3个点拟合圆
            indices = np.random.choice(n_points, size=min(3, n_points), replace=False)
            sample = points[indices]

            if len(sample) < 3:
                continue

            circle = self._circle_from_three_points(sample)
            if circle is None:
                continue

            center, radius = circle

            # 计算内点
            distances = np.linalg.norm(points - center, axis=1)
            inliers = np.sum(np.abs(distances - radius) < inlier_threshold)

            if inliers > best_inliers:
                best_inliers = inliers
                best_circle = (center, radius)

        if best_circle is None or best_inliers < self.min_arc_points:
            return None

        return best_circle

    def _circle_from_three_points(
        self, points: np.ndarray
    ) -> tuple[np.ndarray, float] | None:
        """从三点计算外接圆"""
        if len(points) < 3:
            return None

        # 构建矩阵
        x1, y1 = points[0]
        x2, y2 = points[1]
        x3, y3 = points[2]

        # 计算圆心
        # 参考: https://www.baeldung.com/cs/three-points-circumcircle
        A = x1 * (y2 - y3) - y1 * (x2 - x3) + x2 * y3 - x3 * y2

        if abs(A) < 1e-10:
            return None

        B = (
            (x1**2 + y1**2) * (y3 - y2)
            + (x2**2 + y2**2) * (y1 - y3)
            + (x3**2 + y3**2) * (y2 - y1)
        )

        C = (
            (x1**2 + y1**2) * (x2 - x3)
            + (x2**2 + y2**2) * (x3 - x1)
            + (x3**2 + y3**2) * (x1 - x2)
        )

        D = (
            (x1**2 + y1**2) * (x3 * y2 - x2 * y3)
            + (x2**2 + y2**2) * (x1 * y3 - x3 * y1)
            + (x3**2 + y3**2) * (x1 * y2 - x2 * y1)
        )

        cx = -B / (2 * A)
        cy = -C / (2 * A)
        radius = np.sqrt((cx - x1) ** 2 + (cy - y1) ** 2)

        return (np.array([cx, cy]), radius)

    def _compute_angle_coverage(self, angles: np.ndarray) -> float:
        """计算角度覆盖范围"""
        if len(angles) < 2:
            return 0.0

        angles_sorted = np.sort(angles)

        # 计算最大间隙
        gaps = np.diff(angles_sorted)
        # 考虑首尾连接处的间隙
        wrap_gap = 2 * np.pi - angles_sorted[-1] + angles_sorted[0]
        all_gaps = np.concatenate([gaps, [wrap_gap]])

        max_gap = np.max(all_gaps)

        # 弧度 = 2*pi - 最大间隙
        coverage = 2 * np.pi - max_gap

        return coverage

    def _check_cylinder_hollow(
        self,
        points: np.ndarray,
        center: np.ndarray,
        radius: float,
        z_min: float,
        z_max: float,
        arc_indices: np.ndarray,
    ) -> bool:
        """检查圆柱内部是否为空（验证圆柱是空心的）"""
        # 在圆内但不属于圆弧的区域应该没有点（或很少点）
        points_2d = points[:, :2]
        center_2d = center[:2] if len(center) > 2 else center

        # 到圆心的距离
        distances = np.linalg.norm(points_2d - center_2d, axis=1)

        # 圆内区域（排除圆弧点）
        arc_mask = np.zeros(len(points), dtype=bool)
        arc_mask[arc_indices] = True

        # 圆内、但不在圆弧上、且高度在圆柱范围内
        inside_mask = (
            (np.abs(distances - radius) < self.radius_tolerance * 2)
            & (~arc_mask)
            & (points[:, 2] >= z_min)
            & (points[:, 2] <= z_max)
        )

        # 计算圆心到圆弧的平均距离
        if len(arc_indices) > 0:
            arc_mean_dist = np.mean(np.abs(distances[arc_indices] - radius))
        else:
            arc_mean_dist = 0

        # 圆心附近应该没有点（圆柱是空心的）
        center_region = distances < (radius - self.hollow_tolerance)
        center_region[arc_mask] = False

        # 如果圆心附近有很多点，说明不是圆柱
        if np.sum(center_region) > len(arc_indices) * 0.1:
            return False

        return True

    def _detect_top(
        self,
        points: np.ndarray,
        center: np.ndarray,
        radius: float,
        arc_max_z: float,
    ) -> float:
        """检测圆柱顶部

        策略：
        1. 查找圆柱顶部高度的"水平点云层"（顶面）
        2. 如果没有，则用圆弧最高点
        """
        center_2d = center[:2] if len(center) > 2 else center

        # 到圆心距离
        distances = np.linalg.norm(points[:, :2] - center_2d, axis=1)

        # 在圆柱半径范围内的点
        in_radius_mask = np.abs(distances - radius) < self.radius_tolerance * 3

        if not np.any(in_radius_mask):
            return arc_max_z

        candidate_points = points[in_radius_mask]

        # 按高度分层
        z_values = candidate_points[:, 2]

        # 查找是否有水平层（高度相近的点云）
        z_hist, z_bins = np.histogram(z_values, bins=50)
        z_bin_centers = (z_bins[:-1] + z_bins[1:]) / 2

        # 找到最高密度区域
        max_density_idx = np.argmax(z_hist)
        top_density_z = z_bin_centers[max_density_idx]

        # 检查是否高于圆弧最高点
        if top_density_z > arc_max_z:
            return top_density_z

        # 检查顶面层
        top_layer_mask = (z_values > arc_max_z - 0.1) & (z_values < arc_max_z + 0.1)

        if np.sum(top_layer_mask) >= self.top_layer_min_points:
            # 找到水平层，返回该层的高度
            return np.median(z_values[top_layer_mask])

        # 没有顶面信息，返回圆弧最高点
        return arc_max_z

    def _get_cylinder_inliers(
        self,
        points: np.ndarray,
        center: np.ndarray,
        radius: float,
        z_min: float,
        z_max: float,
        original_indices: np.ndarray,
        filter_indices: np.ndarray,
    ) -> np.ndarray:
        """获取属于圆柱体的原始点索引"""
        center_2d = center[:2] if len(center) > 2 else center

        # 到圆心距离
        distances = np.linalg.norm(points[:, :2] - center_2d, axis=1)

        # 在圆柱侧面范围内
        side_mask = (
            (np.abs(distances - radius) < self.radius_tolerance)
            & (points[:, 2] >= z_min)
            & (points[:, 2] <= z_max)
        )

        # 顶面范围内的点
        top_mask = (distances < radius + self.radius_tolerance) & (
            np.abs(points[:, 2] - z_max) < self.top_layer_thickness
        )

        # 底面范围内的点
        bottom_mask = (distances < radius + self.radius_tolerance) & (
            np.abs(points[:, 2] - z_min) < self.top_layer_thickness
        )

        # 合并所有掩码
        inlier_mask = side_mask | top_mask | bottom_mask

        return original_indices[inlier_mask]


__all__ = ["CylinderDetector"]
