"""算法检测器实现"""

import contextlib
import re

import numpy as np
import yaml

from robotic_follower.util.log import log

from ..pipeline import (
    AlgorithmStage,
    PipelineData,
    PostProcessor,
    PreProcessor,
    StageRegistry,
)
from .__base__ import Detector


class AlgoDetectorConfigError(ValueError):
    """配置错误异常"""


class AlgoDetector(Detector):
    """基于管道式算法处理的检测器

    支持从配置文件加载预处理、算法、后处理阶段，
    管道式执行以完成目标检测。
    """

    # 必要配置字段
    REQUIRED_FIELDS = ["algorithm"]

    def __init__(
        self,
        preprocessors: list[PreProcessor],
        algorithms: list[AlgorithmStage],
        postprocessors: list[PostProcessor],
        detector_name: str | None = None,
        ignore_class_names: tuple[str] = tuple([]),
        parent_node: "rclpy.node.Node" = None,  # type: ignore  # noqa: F821
    ):
        """
        Args:
            detector_name: 检测器名称
            preprocessors: 预处理阶段列表
            algorithms: 算法阶段列表
            postprocessors: 后处理阶段列表
            ignore_class_names: 忽略目标检测的类名
            parent_node: ROS2 节点实例
        """
        # 在 super().__init__() 之前设置 algorithms，以便 _get_class_names 使用
        self.preprocessors = preprocessors
        self.algorithms = algorithms
        self.postprocessors = postprocessors

        super().__init__(
            detector_type="algo",
            detector_name=detector_name,
            ignore_class_names=ignore_class_names,
            parent_node=parent_node,
        )

    def _get_class_names(self) -> tuple[str]:
        """从算法阶段收集 class_names"""
        seen = set()
        class_names = []
        for algo in self.algorithms:
            for name in algo.class_names:
                if name not in seen:
                    class_names.append(name)
                    seen.add(name)
        return tuple(class_names)

    @classmethod
    def _config_check(
        cls,
        config: dict,
        parent_node: "rclpy.node.Node | None" = None,  # type: ignore  # noqa: F821
    ) -> bool:
        """检查配置必要字段和值有效性

        Raises:
            AlgoDetectorConfigError: 配置无效时抛出
        """
        # 检查必要字段
        for field in cls.REQUIRED_FIELDS:
            if field not in config:
                raise AlgoDetectorConfigError(f"Missing required field: '{field}'")

        # 检查 algorithm 字段
        algorithm = config.get("algorithm", [])
        if isinstance(algorithm, dict):
            algorithm = [algorithm]
        if not algorithm:
            raise AlgoDetectorConfigError("algorithm list cannot be empty")

        # 检查每个算法阶段的类型是否有效
        for i, algo in enumerate(algorithm):
            algo_type = algo.get("type")
            if not algo_type:
                raise AlgoDetectorConfigError(f"algorithm[{i}]: missing 'type' field")
            if algo_type not in StageRegistry.list_algorithms():
                raise AlgoDetectorConfigError(
                    f"algorithm[{i}]: unsupported type '{algo_type}'. "
                    f"Supported: {StageRegistry.list_algorithms()}"
                )

        # 检查预处理阶段
        for i, step in enumerate(config.get("pre_process", [])):
            step_type = step.get("type")
            if step_type and step_type not in StageRegistry.list_preprocessors():
                raise AlgoDetectorConfigError(
                    f"pre_process[{i}]: unsupported type '{step_type}'. "
                    f"Supported: {StageRegistry.list_preprocessors()}"
                )

        # 检查后处理阶段
        for i, step in enumerate(config.get("post_process", [])):
            step_type = step.get("type")
            if step_type and step_type not in StageRegistry.list_postprocessors():
                raise AlgoDetectorConfigError(
                    f"post_process[{i}]: unsupported type '{step_type}'. "
                    f"Supported: {StageRegistry.list_postprocessors()}"
                )

        return True

    @classmethod
    def _config_norm(cls, config: dict, parent_node: "rclpy.node.Node | None" = None):  # type: ignore  # noqa: F821
        """规范化配置，填充默认值和处理引用

        Returns:
            规范化后的配置字典
        """
        # 规范化 algorithm 字段：单个 dict 转为 list
        if "algorithm" in config and isinstance(config["algorithm"], dict):
            config["algorithm"] = [config["algorithm"]]

        # 规范化 pre_process 字段：单个 dict 转为 list
        if "pre_process" in config and isinstance(config["pre_process"], dict):
            config["pre_process"] = [config["pre_process"]]

        # 规范化 post_process 字段：单个 dict 转为 list
        if "post_process" in config and isinstance(config["post_process"], dict):
            config["post_process"] = [config["post_process"]]

    @classmethod
    def create_from_config(
        cls,
        config: dict,
        parent_node: "rclpy.node.Node | None" = None,  # type: ignore  # noqa: F821
    ) -> "AlgoDetector | None":
        """从配置字典创建检测器

        Args:
            config: 检测器配置字典
            parent_node: ROS2 节点实例

        Returns:
            AlgoDetector 实例，失败返回 None

        Raises:
            AlgoDetectorConfigError: 配置无效时抛出
        """
        try:
            config = config.copy()
            cls._config_check(config, parent_node)
            cls._config_norm(config)

            # 解析全局参数
            global_params = cls._parse_global_params(config.get("global_params", {}))

            # 创建预处理阶段
            preprocessors = []
            for step in config.get("pre_process", []):
                if not step.get("enabled", True):
                    continue
                preprocessor = cls._create_stage_from_config(
                    step, "preprocessor", global_params, parent_node=parent_node
                )
                if preprocessor:
                    preprocessors.append(preprocessor)

            # 创建算法阶段
            algorithms = []
            for step in config.get("algorithm", []):
                algorithm = cls._create_stage_from_config(
                    step, "algorithm", global_params, parent_node=parent_node
                )
                if algorithm:
                    algorithms.append(algorithm)

            # 创建后处理阶段
            postprocessors = []
            for step in config.get("post_process", []):
                if not step.get("enabled", True):
                    continue
                postprocessor = cls._create_stage_from_config(
                    step, "postprocessor", global_params, parent_node=parent_node
                )
                if postprocessor:
                    postprocessors.append(postprocessor)

            # 检查是否至少有一个算法阶段
            if not algorithms:
                log(
                    "fatal",
                    "No valid algorithm stages could be created",
                    node=parent_node,
                )
                raise AlgoDetectorConfigError(
                    "No valid algorithm stages could be created"
                )

            return cls(
                detector_name=config.get("name", None),
                preprocessors=preprocessors,
                algorithms=algorithms,
                postprocessors=postprocessors,
                ignore_class_names=tuple(config.get("ignore_class_names", ())),
                parent_node=parent_node,
            )

        except AlgoDetectorConfigError:
            raise  # 直接重新抛出配置错误
        except Exception as e:
            raise AlgoDetectorConfigError(f"Failed to create detector: {e}") from e

    @classmethod
    def _parse_global_params(cls, global_params: dict) -> dict:
        """解析全局参数，支持 ${} 引用"""
        result = {}
        for key, value in global_params.items():
            if (
                isinstance(value, str)
                and value.startswith("${")
                and value.endswith("}")
            ):
                # 引用其他全局参数（暂不支持复杂引用）
                ref_key = value[2:-1]
                result[key] = global_params.get(ref_key, value)
            else:
                result[key] = value
        return result

    @classmethod
    def _resolve_params(cls, params: dict, global_params: dict) -> dict:
        """解析参数中的 ${} 引用"""
        resolved = {}
        for key, value in params.items():
            if isinstance(value, str):
                # 替换 ${global_params.xxx} 形式的引用
                pattern = r"\$\{([^}]+)\}"
                matches = re.findall(pattern, value)
                for match in matches:
                    if match in global_params:
                        value = value.replace(
                            f"${{{match}}}", str(global_params[match])
                        )
                # 尝试转换类型
                with contextlib.suppress(ValueError, TypeError):
                    value = float(value) if "." in value else int(value)
            resolved[key] = value
        return resolved

    @classmethod
    def _create_stage_from_config(
        cls,
        step: dict,
        stage_type: str,
        global_params: dict,
        parent_node: "rclpy.node.Node | None" = None,  # type: ignore  # noqa: F821
    ) -> PreProcessor | AlgorithmStage | PostProcessor | None:
        """从配置创建单个阶段"""
        step_type = step.get("type")
        if not step_type:
            return None

        params = cls._resolve_params(step.get("params", {}), global_params)

        try:
            if stage_type == "preprocessor":
                return StageRegistry.create_preprocessor(
                    step_type, params, parent_node=parent_node
                )
            if stage_type == "algorithm":
                return StageRegistry.create_algorithm(
                    step_type, params, parent_node=parent_node
                )
            if stage_type == "postprocessor":
                return StageRegistry.create_postprocessor(
                    step_type, params, parent_node=parent_node
                )
        except Exception as e:
            log(
                "error",
                f"Failed to create {stage_type} '{step_type}': {e}",
                node=parent_node,
            )
            return None

        return None

    @property
    def ready(self) -> bool:
        """返回检测器是否就绪"""
        return len(self.algorithms) > 0 and super().ready

    def detect(self, points: np.ndarray) -> list[dict]:
        """执行检测

        Args:
            points: 输入点云 (N, 3)

        Returns:
            检测结果列表
        """
        if not self.ready:
            self._warn("Detector not ready")
            return []

        # 初始化数据
        data = PipelineData()
        data.points = np.asarray(points)
        data.point_mask = np.ones(len(points), dtype=bool)
        data.original_indices = np.arange(len(points))

        # 预处理阶段
        for preprocessor in self.preprocessors:
            try:
                data = preprocessor.process(data)
            except Exception as e:
                self._warn(f"Preprocessor {preprocessor.stage_name} failed: {e}")
                continue

        # 算法阶段
        self._debug(f"开始算法阶段，共 {len(self.algorithms)} 个算法")
        for algorithm in self.algorithms:
            self._debug(f"调用算法: {algorithm.stage_name}")
            try:
                data = algorithm.process(data)
                self._debug(
                    f"算法 {algorithm.stage_name} 完成，detections={len(data.detections)}"
                )
            except Exception as e:
                self._error(f"Algorithm {algorithm.stage_name} failed: {e}")
                # 算法失败时继续处理，但不返回
                break

        # 后处理阶段
        for postprocessor in self.postprocessors:
            try:
                data = postprocessor.process(data)
            except Exception as e:
                self._warn(f"Postprocessor {postprocessor.stage_name} failed: {e}")
                continue

        # 格式化输出
        return self._format_output(data.detections)

    def _format_output(self, detections: list[dict]) -> list[dict]:
        """格式化检测结果，由检测器统一分配 label"""
        formatted = []
        class_name2idx = getattr(self, "class_name2idx", {})
        for det in detections:
            if "bbox" not in det:
                continue
            name = det.get("name", "unknown")
            label = det.get("label")
            if label is None and name in class_name2idx:
                label = class_name2idx[name]
            item = {
                "bbox": det["bbox"],
                "score": det.get("score", 1.0),
                "label": label if label is not None else 0,
                "name": name,
            }
            formatted.append(item)

        return formatted

    @classmethod
    def from_config_file(
        cls,
        config_path: str,
        parent_node: "rclpy.node.Node | None" = None,  # type: ignore  # noqa: F821
    ) -> "AlgoDetector | None":
        """从配置文件创建检测器

        Args:
            config_path: 配置文件路径
            parent_node: ROS2 节点实例

        Returns:
            AlgoDetector 实例，失败返回 None

        Raises:
            AlgoDetectorConfigError: 配置文件无效时抛出
        """
        try:
            with open(config_path) as f:
                config = yaml.safe_load(f)
                if "detector" not in config:
                    log(
                        "fatal",
                        "从config创建检测器失败，缺少detector字段",
                        node=parent_node,
                    )
            return cls.create_from_config(config, parent_node)
        except (yaml.YAMLError, OSError) as e:
            msg = f"Failed to load config from {config_path}: {e}"
            log("fatal", msg, node=parent_node)
            raise AlgoDetectorConfigError(msg) from e


__all__ = ["AlgoDetector", "AlgoDetectorConfigError"]
