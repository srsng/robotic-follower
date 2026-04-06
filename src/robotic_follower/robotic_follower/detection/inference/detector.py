"""3D 目标检测推理模块（MMDetection3D 集成）。"""

import importlib
import os
from pathlib import Path

import numpy as np
from mmdet3d.apis import inference_detector  # type: ignore


class Detector3D:
    """3D 目标检测器封装（MMDetection3D）。"""

    def __init__(
        self,
        config_file: str,
        checkpoint_file: str,
        device: str = "cuda:0",
        score_threshold: float = 0.3,
        detector_type: str = "mmdet3d",
        ignore_class_names: tuple[str] = tuple([]),
        node: "rclpy.node.Node" = None,  # type: ignore
    ):
        """
        初始化 3D 检测器。

        Args:
            config_file: MMDetection3D 配置文件路径
            checkpoint_file: 模型权重文件路径
            device: 设备 ('cuda:0' 或 'cpu')
            score_threshold: 检测置信度阈值
            detector_type: 检测器类型 ('mmdet3d', 'dspdet3d')
            node: ROS2 节点实例，用于日志输出
        """
        self.config_file = os.path.expanduser(config_file)
        self.checkpoint_file = os.path.expanduser(checkpoint_file)
        self.device = device
        self.score_threshold = score_threshold
        self.detector_type = detector_type
        self.node = node

        self.class_names = ()
        self.idx2class_name = {}
        self.class_name2idx = {}
        self.model = None
        self.ignore_class_names = ignore_class_names
        self.ignore_class_idx = ()
        self._load_config_info()
        self._load_model()

    def _load_config_info(self):
        """
        通过 config_file 导入必要信息
        """

        def get_specific_constants(file_path, var_name):
            file_path = Path(file_path).resolve()
            module_name = f"dynamic_module_{file_path.stem}"

            spec = importlib.util.spec_from_file_location(module_name, file_path)  # type: ignore
            module = importlib.util.module_from_spec(spec)  # type: ignore
            spec.loader.exec_module(module)

            result = {}
            for name in var_name:
                if hasattr(module, name):
                    result[name] = getattr(module, name)
                else:
                    result[name] = None
            return result

        # 导入 class_names
        values = get_specific_constants(self.config_file, ["class_names"])

        self.class_names = (
            values["class_names"] if values["class_names"] is not None else ()
        )
        self.idx2class_name = {
            i: self.class_names[i] for i in range(len(self.class_names))
        }
        self.class_name2idx = {
            self.class_names[i]: i for i in range(len(self.class_names))
        }
        self._init_ignore_class_idx()

    def _init_ignore_class_idx(self):
        """初始化 self.ignore_class_idx 并警告无效的 ignore_class_names"""
        valid_class_names = [
            name for name in self.ignore_class_names if name in self.class_names
        ]
        invalid_class_names = tuple(
            name for name in self.ignore_class_names if name not in self.class_names
        )
        tmp = [self.class_name2idx[name] for name in valid_class_names]
        tmp.sort()
        self.ignore_class_idx = tuple(tmp)

        if len(invalid_class_names) != 0:
            (self.node.get_logger().warn if self.node else print)(
                f"无效的 ignore_class_names: {invalid_class_names}, 当前模型支持的class: {self.class_names}"
            )

    def _load_model(self):
        """加载 MMDetection3D 模型。"""
        logger = self.node.get_logger() if self.node else None

        if not os.path.exists(self.config_file):
            msg = f"警告：模型config文件不存在: {self.config_file}"
            (logger.error if logger else print)(msg)
            self.model = None
            return

        if not os.path.exists(self.checkpoint_file):
            msg = f"警告：模型权重文件不存在: {self.checkpoint_file}"
            (logger.error if logger else print)(msg)
            self.model = None
            return

        try:
            from mmdet3d.apis import init_model  # type: ignore

            config = self.config_file
            self.model = init_model(config, self.checkpoint_file, device=self.device)
            msg = f"成功加载 3D 检测模型: {self.checkpoint_file}"
            (logger.info if logger else print)(msg)
        except ImportError as e:
            msg = f"警告：MMDetection3D 模块导入失败: {e}"
            (logger.error if logger else print)(msg)
            self.model = None
        except Exception as e:
            msg = f"警告：模型加载失败: {e}"
            (logger.error if logger else print)(msg)
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

            # 过滤 忽略的class name
            detections = [
                det for det in detections if det["label"] not in self.ignore_class_idx
            ]

            return detections

        except Exception as e:
            msg = f"检测失败: {e}"
            if self.node:
                self.node.get_logger().error(msg)
            else:
                print(msg)
            return []

    def _parse_mmdet3d_result(self, result) -> list[dict]:
        """解析 MMDetection3D 检测结果。"""
        detections = []

        def _to_numpy(attr):
            """安全提取numpy数组，处理tensor和numpy两种格式"""
            if hasattr(attr, "cpu"):
                return attr.cpu().numpy()
            return np.asarray(attr)

        # 提取边界框、分数和标签 (针对 MMDetection3D 1.x 中 Det3DDataSample 结构)
        if hasattr(result, "pred_instances_3d"):
            pred = result.pred_instances_3d
            bboxes_3d = pred.bboxes_3d
            bboxes = _to_numpy(
                bboxes_3d.tensor if hasattr(bboxes_3d, "tensor") else bboxes_3d
            )
            scores = _to_numpy(pred.scores_3d)
            labels = _to_numpy(pred.labels_3d)
        else:
            # 兼容老版本格式
            boxes_3d = result["boxes_3d"]
            bboxes = _to_numpy(
                boxes_3d.tensor if hasattr(boxes_3d, "tensor") else boxes_3d
            )
            scores = _to_numpy(result["scores_3d"])
            labels = _to_numpy(result["labels_3d"])

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


def create_detector_from_config(
    config: dict,
    node: "rclpy.node.Node" = None,  # type: ignore
) -> Detector3D:
    """
    从配置字典创建检测器。

    Args:
        config: 配置字典
        node: ROS2 节点实例，用于日志输出

    Returns:
        Detector3D 实例
    """
    detector_type = config.get("type", "mmdet3d")
    ignore_class_names = tuple(config.get("ignore_class_names", ()))

    return Detector3D(
        config_file=config["config_file"],
        checkpoint_file=config["checkpoint_file"],
        device=config.get("device", "cuda:0"),
        score_threshold=config.get("score_threshold", 0.3),
        detector_type=detector_type,
        ignore_class_names=ignore_class_names,
        node=node,
    )
