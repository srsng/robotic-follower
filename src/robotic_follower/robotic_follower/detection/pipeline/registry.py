"""管道阶段注册表"""

from .stages import AlgorithmStage, PostProcessor, PreProcessor


class StageRegistry:
    """阶段类型注册表，使用装饰器模式"""

    _preprocessors: dict[str, type[PreProcessor]] = {}
    _algorithms: dict[str, type[AlgorithmStage]] = {}
    _postprocessors: dict[str, type[PostProcessor]] = {}

    @classmethod
    def register_preprocessor(cls, name: str):
        """注册预处理阶段类型

        Usage:
            @StageRegistry.register_preprocessor("passthrough")
            class PassthroughFilter(PreProcessor):
                ...
        """

        def decorator(stage_cls: type[PreProcessor]) -> type[PreProcessor]:
            cls._preprocessors[name] = stage_cls
            return stage_cls

        return decorator

    @classmethod
    def register_algorithm(
        cls, name: str, class_names: list[str] | tuple[str, ...] | None = None
    ):
        """注册算法阶段类型

        Args:
            name: 算法类型名称
            class_names: 该算法检测的类别名称列表

        Usage:
            @StageRegistry.register_algorithm("ground_detector", class_names=["ground", "others"])
            class GroundDetector(AlgorithmStage):
                ...
        """

        def decorator(stage_cls: type[AlgorithmStage]) -> type[AlgorithmStage]:
            if class_names is not None:
                stage_cls._registered_class_names = tuple(class_names)
            cls._algorithms[name] = stage_cls
            return stage_cls

        return decorator

    @classmethod
    def register_postprocessor(cls, name: str):
        """注册后处理阶段类型"""

        def decorator(stage_cls: type[PostProcessor]) -> type[PostProcessor]:
            cls._postprocessors[name] = stage_cls
            return stage_cls

        return decorator

    @classmethod
    def create_preprocessor(
        cls, name: str, params: dict | None = None, **kwargs
    ) -> PreProcessor:
        """创建预处理阶段实例"""
        if name not in cls._preprocessors:
            raise ValueError(
                f"Unknown preprocessor: {name}, available: {list(cls._preprocessors.keys())}"
            )
        return cls._preprocessors[name](**(params or {}), **kwargs)

    @classmethod
    def create_algorithm(
        cls, name: str, params: dict | None = None, **kwargs
    ) -> AlgorithmStage:
        """创建算法阶段实例"""
        if name not in cls._algorithms:
            raise ValueError(
                f"Unknown algorithm: {name}, available: {list(cls._algorithms.keys())}"
            )
        return cls._algorithms[name](**(params or {}), **kwargs)

    @classmethod
    def create_postprocessor(
        cls, name: str, params: dict | None = None, **kwargs
    ) -> PostProcessor:
        """创建后处理阶段实例"""
        if name not in cls._postprocessors:
            raise ValueError(
                f"Unknown postprocessor: {name}, available: {list(cls._postprocessors.keys())}"
            )
        return cls._postprocessors[name](**(params or {}), **kwargs)

    @classmethod
    def list_preprocessors(cls) -> list[str]:
        return list(cls._preprocessors.keys())

    @classmethod
    def list_algorithms(cls) -> list[str]:
        return list(cls._algorithms.keys())

    @classmethod
    def list_postprocessors(cls) -> list[str]:
        return list(cls._postprocessors.keys())
