"""管道阶段基类"""

from abc import ABC, abstractmethod
from typing import Generic, TypeVar

import numpy as np

from robotic_follower.util.handler import NodeHandler

from .data import PipelineData


T = TypeVar("T", bound="PipelineStage")


class PipelineStage(NodeHandler, ABC, Generic[T]):
    """管道阶段的抽象基类"""

    def __init__(
        self,
        stage_type: str,
        stage_name: str | None = None,
        parent_node: "rclpy.node.Node" = None,  # type: ignore  # noqa: F821
    ):
        """
        Args:
            stage_type: 阶段类型标识
            stage_name: 阶段名称（可选）
            parent_node: ROS2 节点实例，用于日志输出
        """
        NodeHandler.__init__(self, parent_node)
        self.stage_type = stage_type
        self.stage_name = stage_name or f"{stage_type}_stage"

    @abstractmethod
    def process(self, data: PipelineData) -> PipelineData:
        """处理数据

        Args:
            data: 输入数据

        Returns:
            处理后的数据
        """
        ...

    def validate(self, data: PipelineData) -> bool:
        """验证输入数据是否有效

        Args:
            data: 输入数据

        Returns:
            是否有效
        """
        return len(data.points) > 0


class PreProcessor(PipelineStage):
    """预处理阶段基类（点云滤波）"""

    def __init__(
        self,
        stage_name: str | None = None,
        parent_node: "rclpy.node.Node" = None,  # type: ignore  # noqa: F821
    ):
        super().__init__("preprocessor", stage_name, parent_node)

    @abstractmethod
    def filter(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """滤波处理

        Args:
            points: 输入点云 (N, 3)

        Returns:
            tuple of (滤波后的点云 (M, 3), 保留点的原始索引 (M,))
        """
        ...

    def process(self, data: PipelineData) -> PipelineData:
        """预处理点云"""
        if not self.validate(data):
            data.context.setdefault("warnings", []).append(
                f"{self.stage_name}: 输入点云为空，跳过"
            )
            return data

        filtered_points, keep_indices = self.filter(data.points)

        # 过滤后所有点都有效，因此掩码全为 True
        # 确保 point_mask 与过滤后的 points 长度一致
        data.point_mask = np.ones(len(filtered_points), dtype=bool)

        # 更新原始索引映射
        if len(data.original_indices) > 0:
            data.original_indices = data.original_indices[keep_indices]
        else:
            data.original_indices = keep_indices

        # 过滤标签（如果有）
        if len(data.labels) > 0:
            data.labels = data.labels[keep_indices]

        data.points = filtered_points

        return data


class AlgorithmStage(PipelineStage):
    """算法阶段基类（核心检测）"""

    # 预定义的类别标签（按检测顺序分配）
    LABEL_GROUND = 0
    LABEL_OTHERS = 1

    # 类别名称映射
    LABEL_NAMES: dict[int, str] = {
        LABEL_GROUND: "ground",
        LABEL_OTHERS: "others",
    }

    def __init__(
        self,
        stage_name: str | None = None,
        parent_node: "rclpy.node.Node" = None,  # type: ignore  # noqa: F821
    ):
        super().__init__("algorithm", stage_name, parent_node)

    @abstractmethod
    def detect(self, data: PipelineData) -> PipelineData:
        """执行检测

        Args:
            data: 输入数据

        Returns:
            更新后的数据，包含 detections
        """
        ...

    def process(self, data: PipelineData) -> PipelineData:
        """算法处理"""
        if not self.validate(data):
            self._warn(f"输入点云为空，跳过 {self.stage_name}")
            return data

        return self.detect(data)

    @staticmethod
    def make_detection(
        points: np.ndarray,
        label: int,
        score: float = 1.0,
        name: str | None = None,
        point_indices: np.ndarray | None = None,
    ) -> dict:
        """创建检测结果

        Args:
            points: 目标点云 (N, 3)
            label: 类别标签
            score: 置信度
            name: 类别名称
            point_indices: 原始点云中的索引，未提供则默认 0,1,2...

        Returns:
            检测结果字典
        """
        if name is None:
            name = AlgorithmStage.LABEL_NAMES.get(label, f"class_{label}")

        if point_indices is None:
            point_indices = np.arange(len(points))

        # 计算最小包围盒
        # bbox 格式: [x, y, z, dx, dy, dz, yaw]
        # 其中 (x, y, z) 是几何中心
        min_coords = points.min(axis=0)
        max_coords = points.max(axis=0)
        center = (min_coords + max_coords) / 2
        sizes = max_coords - min_coords

        return {
            "bbox": np.concatenate(
                [center, sizes, np.zeros(1)]
            ),  # [x,y,z,dx,dy,dz,yaw] - z 是几何中心
            "score": score,
            "label": label,
            "name": name,
            "points": points,
            "point_indices": point_indices,
        }


class PostProcessor(PipelineStage):
    """后处理阶段基类（结果变换）"""

    def __init__(
        self,
        stage_name: str | None = None,
        parent_node: "rclpy.node.Node" = None,  # type: ignore  # noqa: F821
    ):
        super().__init__("postprocessor", stage_name, parent_node)

    @abstractmethod
    def transform(self, detections: list[dict]) -> list[dict]:
        """变换检测结果

        Args:
            detections: 输入检测结果列表

        Returns:
            变换后的检测结果列表
        """
        ...

    def process(self, data: PipelineData) -> PipelineData:
        """后处理"""
        if not data.detections:
            return data

        try:
            data.detections = self.transform(data.detections)
        except Exception as e:
            data.context.setdefault("warnings", []).append(
                f"{self.stage_name}: 变换失败 ({e})，保留原结果"
            )

        return data
