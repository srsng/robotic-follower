"""3D 目标检测推理模块（MMDetection3D 集成）。"""

import os

import numpy as np
import torch


class Detector3D:
    """3D 目标检测器封装（MMDetection3D）。"""

    def __init__(
        self,
        config_file: str,
        checkpoint_file: str,
        device: str = "cuda:0",
        score_threshold: float = 0.3,
        detector_type: str = "mmdet3d",
    ):
        """
        初始化 3D 检测器。

        Args:
            config_file: MMDetection3D 配置文件路径
            checkpoint_file: 模型权重文件路径
            device: 设备 ('cuda:0' 或 'cpu')
            score_threshold: 检测置信度阈值
            detector_type: 检测器类型 ('mmdet3d', 'dspdet3d')
        """
        self.config_file = os.path.expanduser(config_file)
        self.checkpoint_file = os.path.expanduser(checkpoint_file)
        self.device = device
        self.score_threshold = score_threshold
        self.detector_type = detector_type
        self.model = None

        self._load_model()

    def _load_model(self):
        """加载 MMDetection3D 模型。"""
        # 检查权重文件是否存在
        if not self.checkpoint_file:
            print("警告：未配置模型权重文件")
            self.model = None
            return

        if not os.path.exists(self.checkpoint_file):
            print(f"警告：模型文件不存在: {self.checkpoint_file}")
            self.model = None
            return

        try:
            from mmdet3d.apis import init_model

            config = self.config_file if self.config_file else None
            self.model = init_model(config, self.checkpoint_file, device=self.device)
            print(f"✓ 成功加载 3D 检测模型: {self.checkpoint_file}")
        except ImportError as e:
            print(f"警告：MMDetection3D 模块导入失败: {e}")
            self.model = None
        except Exception as e:
            print(f"警告：模型加载失败: {e}")
            self.model = None

    def detect(self, points: np.ndarray) -> list[dict]:
        """
        执行 3D 目标检测。

        Args:
            points: 输入点云 (N, 3)

        Returns:
            检测结果列表，每个结果包含：
            - bbox: 3D 边界框 [x, y, z, dx, dy, dz, yaw]
            - score: 置信度
            - label: 类别标签
        """
        if self.model is None:
            # 模拟检测器（用于测试）
            return self._mock_detect(points)

        # 准备输入数据 - 确保类型正确
        points = points.astype(np.float32, copy=False)

        # 执行推理
        try:
            from mmdet3d.apis import inference_detector

            # 确保模型在正确的设备上
            if "cuda" in self.device and torch.cuda.is_available():
                self.model.to(self.device)

            # inference_detector 内部使用 test_pipeline 自动计算密度等特征
            result = inference_detector(self.model, points)

            # mmdet3d 1.x 中 inference_detector 的返回值通常是一个包含结果和数据的元组 (result, data)
            # 或者如果是批处理推理，返回的可能是列表。这里进行兼容处理。
            if isinstance(result, tuple):
                result = result[0]
            if isinstance(result, list):
                result = result[0]

            # 解析结果
            detections = self._parse_mmdet3d_result(result)

            # 过滤低置信度检测
            detections = [
                det for det in detections if det["score"] >= self.score_threshold
            ]

            return detections

        except Exception as e:
            print(f"检测失败: {e}")
            return []

    def _parse_mmdet3d_result(self, result) -> list[dict]:
        """解析 MMDetection3D 检测结果。"""
        detections = []

        # 提取边界框、分数和标签 (针对 MMDetection3D 1.x 中 Det3DDataSample 结构)
        if hasattr(result, "pred_instances_3d"):
            pred = result.pred_instances_3d
            bboxes = pred.bboxes_3d.tensor.cpu().numpy()
            scores = pred.scores_3d.cpu().numpy()
            labels = pred.labels_3d.cpu().numpy()
        else:
            # 兼容老版本格式
            bboxes = result["boxes_3d"].tensor.cpu().numpy()
            scores = result["scores_3d"].cpu().numpy()
            labels = result["labels_3d"].cpu().numpy()

        for bbox, score, label in zip(bboxes, scores, labels):
            detections.append(
                {
                    "bbox": bbox.tolist(),  # [x, y, z, dx, dy, dz, yaw]
                    "score": float(score),
                    "label": int(label),
                }
            )

        return detections

    def _mock_detect(self, points: np.ndarray) -> list[dict]:
        """模拟检测器（用于测试）。"""
        # 计算点云中心作为模拟检测结果
        center = np.mean(points, axis=0)

        return [
            {
                "bbox": [
                    float(center[0]),
                    float(center[1]),
                    float(center[2]),
                    0.3,
                    0.3,
                    0.3,  # 尺寸
                    0.0,  # 朝向
                ],
                "score": 0.95,
                "label": 0,
            }
        ]


def create_detector_from_config(config: dict) -> Detector3D:
    """
    从配置字典创建检测器。

    Args:
        config: 配置字典

    Returns:
        Detector3D 实例
    """
    detector_type = config.get("type", "mmdet3d")

    return Detector3D(
        config_file=config["config_file"],
        checkpoint_file=config["checkpoint_file"],
        device=config.get("device", "cuda:0"),
        score_threshold=config.get("score_threshold", 0.3),
        detector_type=detector_type,
    )
