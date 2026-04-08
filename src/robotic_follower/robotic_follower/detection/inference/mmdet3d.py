"""3D 目标检测推理模块（MMDetection3D 集成）"""

import os
import os.path as osp

import numpy as np
from mmdet3d.apis import inference_detector  # type: ignore

from robotic_follower.util.import_helper import load_var_from_file
from robotic_follower.util.log import log

from .__base__ import Detector


class Mmdet3dDetector(Detector):
    """3D 目标检测器封装（MMDetection3D）"""

    def __init__(
        self,
        config_file: str,
        checkpoint_file: str,
        device: str = "cuda:0",
        score_threshold: float = 0.3,
        detector_type: str = "mmdet3d",
        ignore_class_names: tuple[str] = tuple([]),
        node: "rclpy.node.Node" = None,  # type: ignore  # noqa: F821
    ):
        """
        初始化 3D 检测器

        Args:
            config_file: MMDetection3D 配置文件路径
            checkpoint_file: 模型权重文件路径
            device: 设备 ('cuda:0' 或 'cpu')
            score_threshold: 检测置信度阈值
            detector_type: 检测器类型 ('mmdet3d')
            ignore_class_names: 忽略检测结果的类名
            node: ROS2 节点实例，用于日志输出
        """
        super().__init__(detector_type=detector_type, parent_node=node)

        self.config_file = config_file
        self.checkpoint_file = checkpoint_file
        self.device = device
        self.score_threshold = score_threshold
        self.model = None
        self.ignore_class_names = ignore_class_names

        self.class_names = ()
        self.idx2class_name = {}
        self.class_name2idx = {}
        self.ignore_class_idx = ()

        self._load_config_info(self.config_file)
        self._load_model()

    @property
    def ready(self) -> bool:
        return self.model is not None and super().ready

    def _load_config_info(self, config_file: str):
        """通过 config_file 导入必要信息"""

        # 导入 class_names
        values = load_var_from_file(config_file, ["class_names"])

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
        """初始化 ignore_class_idx"""
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
            self._warn(
                f"无效的 ignore_class_names: {invalid_class_names}, 当前模型支持的class: {self.class_names}"
            )

    def _load_model(self):
        """加载 MMDetection3D 模型"""
        if not os.path.exists(self.config_file):
            self._fatal(f"模型config文件不存在: {self.config_file}")
            self.model = None
            return

        if not os.path.exists(self.checkpoint_file):
            self._fatal(f"模型权重文件不存在: {self.checkpoint_file}")
            self.model = None
            return

        try:
            from mmdet3d.apis import init_model  # type: ignore

            config = self.config_file
            self.model = init_model(config, self.checkpoint_file, device=self.device)
            self._info(f"成功加载 3D 检测模型: {self.checkpoint_file}")
        except ImportError as e:
            self._fatal(f"MMDetection3D 模块导入失败: {e}")
            self.model = None
        except Exception as e:
            self._fatal(f"模型加载失败: {e}")
            self.model = None

    def detect(self, points: np.ndarray) -> list[dict]:
        """
        执行 3D 目标检测

        Args:
            points: 输入点云 (N, 3)

        Returns:
            检测结果列表，每个结果包含：
            - bbox: 3D 边界框 [x, y, z, dx, dy, dz, yaw]
            - score: 置信度
            - label: 类别标签
            - class_name: 类别名称
        """
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

            # 补充类别名称
            for det in detections:
                det["name"] = self.idx2class_name.get(det["label"], "unknown")

            return detections

        except Exception as e:
            self._error(f"检测失败: {e}")
            return []

    def _parse_mmdet3d_result(self, result) -> list[dict]:
        """解析 MMDetection3D 检测结果"""
        detections = []

        # 提取边界框、分数和标签 (针对 MMDetection3D 1.x 中 Det3DDataSample 结构)
        if hasattr(result, "pred_instances_3d"):
            pred = result.pred_instances_3d
            bboxes_3d = pred.bboxes_3d
            bboxes = self._to_numpy(
                bboxes_3d.tensor if hasattr(bboxes_3d, "tensor") else bboxes_3d
            )
            scores = self._to_numpy(pred.scores_3d)
            labels = self._to_numpy(pred.labels_3d)
        else:
            # 兼容老版本格式
            boxes_3d = result["boxes_3d"]
            bboxes = self._to_numpy(
                boxes_3d.tensor if hasattr(boxes_3d, "tensor") else boxes_3d
            )
            scores = self._to_numpy(result["scores_3d"])
            labels = self._to_numpy(result["labels_3d"])

        for bbox, score, label in zip(bboxes, scores, labels):
            detections.append(
                {
                    "bbox": bbox.tolist(),  # [x, y, z, dx, dy, dz, yaw]
                    "score": float(score),
                    "label": int(label),
                }
            )

        return detections

    @classmethod
    def _config_check(
        cls,
        config: dict,
        parent_node: "rclpy.node.Node" = None,  # type: ignore  # noqa: F821
    ) -> bool:
        """检查config必要字段，汇总所有缺失字段，分别将相关信息通过._fatal输出到日志与错误列表。
        若有缺失字段，则返回 False
        若config正常，则返回 True
        """
        must_keys = ("config_file", "checkpoint_file")
        err_ls = []

        def fatal_cb(level, msg):
            return err_ls.append(msg)

        for key in must_keys:
            if key not in config:
                log(
                    "fatal",
                    f"检测模型未加载: 配置缺失 {key} 项",
                    parent_node,
                    call=fatal_cb,
                )

        if err_ls:
            log(
                "error",
                "检测模型未加载: 请修复配置缺失项",
                parent_node,
                call=fatal_cb,
            )
            return False
        return True

    @classmethod
    def _config_norm(
        cls,
        config: dict,
        parent_node: "rclpy.node.Node" = None,  # type: ignore  # noqa: F821
    ):
        """规范化检测器config的值 (in-place)

        1. 展开用户路径
        """
        path_keys = ("config_file", "checkpoint_file")
        for key in path_keys:
            if key in config:
                config[key] = osp.expanduser(config[key])

    @classmethod
    def create_from_config(
        cls,
        config: dict,
        parent_node: "rclpy.node.Node" = None,  # type: ignore  # noqa: F821
    ) -> "Mmdet3dDetector | None":
        """
        从配置字典创建检测器

        Args:
            config: 配置字典
            parent_node: ROS2 节点实例，用于日志输出

        Returns:
            Detector3D 实例
        """
        if not cls._config_check(config, parent_node):
            return None
        cls._config_norm(config, parent_node)

        log("info", f"config_file: {config['config_file']}", parent_node)
        log("info", f"checkpoint_file: {config['checkpoint_file']}", parent_node)

        if "name" in config:
            log("info", f"检测器名称: {config['name']}")

        return cls(
            config_file=config["config_file"],
            checkpoint_file=config["checkpoint_file"],
            device=config.get("device", "cuda:0"),
            score_threshold=config.get("score_threshold", 0.3),
            detector_type=config.get("type", "mmdet3d"),
            ignore_class_names=tuple(config.get("ignore_class_names", ())),
            node=parent_node,
        )
