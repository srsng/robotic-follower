"""RGBD pipeline stage registry."""

from __future__ import annotations

from .rgbd_stages import RgbdPostProcessor, RgbdPreProcessor, RgbdProcessor


class RgbdStageRegistry:
    """Registry for RGBD pipeline stages."""

    _preprocessors: dict[str, type[RgbdPreProcessor]] = {}
    _processors: dict[str, type[RgbdProcessor]] = {}
    _postprocessors: dict[str, type[RgbdPostProcessor]] = {}

    @classmethod
    def register_preprocessor(cls, name: str):
        def decorator(stage_cls: type[RgbdPreProcessor]) -> type[RgbdPreProcessor]:
            cls._preprocessors[name] = stage_cls
            return stage_cls

        return decorator

    @classmethod
    def register_processor(cls, name: str):
        def decorator(stage_cls: type[RgbdProcessor]) -> type[RgbdProcessor]:
            cls._processors[name] = stage_cls
            return stage_cls

        return decorator

    @classmethod
    def register_postprocessor(cls, name: str):
        def decorator(stage_cls: type[RgbdPostProcessor]) -> type[RgbdPostProcessor]:
            cls._postprocessors[name] = stage_cls
            return stage_cls

        return decorator

    @classmethod
    def create_preprocessor(
        cls, name: str, params: dict | None = None, **kwargs
    ) -> RgbdPreProcessor:
        if name not in cls._preprocessors:
            raise ValueError(
                f"Unknown rgbd preprocessor: {name}, available: {list(cls._preprocessors.keys())}"
            )
        return cls._preprocessors[name](**(params or {}), **kwargs)

    @classmethod
    def create_processor(
        cls, name: str, params: dict | None = None, **kwargs
    ) -> RgbdProcessor:
        if name not in cls._processors:
            raise ValueError(
                f"Unknown rgbd processor: {name}, available: {list(cls._processors.keys())}"
            )
        return cls._processors[name](**(params or {}), **kwargs)

    @classmethod
    def create_postprocessor(
        cls, name: str, params: dict | None = None, **kwargs
    ) -> RgbdPostProcessor:
        if name not in cls._postprocessors:
            raise ValueError(
                f"Unknown rgbd postprocessor: {name}, available: {list(cls._postprocessors.keys())}"
            )
        return cls._postprocessors[name](**(params or {}), **kwargs)

    @classmethod
    def list_preprocessors(cls) -> list[str]:
        return list(cls._preprocessors.keys())

    @classmethod
    def list_processors(cls) -> list[str]:
        return list(cls._processors.keys())

    @classmethod
    def list_postprocessors(cls) -> list[str]:
        return list(cls._postprocessors.keys())


__all__ = ["RgbdStageRegistry"]
