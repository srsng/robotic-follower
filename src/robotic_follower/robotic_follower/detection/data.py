"""Shared detection data structures."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class DetectionCandidate:
    bbox: list[float]
    score: float
    label: str
    occlusion_ratio: float
    graspable: bool

    def to_dict(self) -> dict:
        return {
            "bbox": self.bbox,
            "score": float(self.score),
            "label": str(self.label),
            "occlusion_ratio": float(self.occlusion_ratio),
            "graspable": bool(self.graspable),
        }
