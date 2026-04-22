"""Segmenter interface for pluggable image segmentation backends."""

from abc import ABC, abstractmethod

import numpy as np

from robotic_follower.util.handler import NodeHandler


class SegmenterBase(NodeHandler, ABC):
    """Abstract image segmenter backend."""

    def __init__(self, segmenter_type: str, parent_node=None):
        super().__init__(parent_node=parent_node)
        self.segmenter_type = segmenter_type

    @abstractmethod
    def segment(self, image_bgr: np.ndarray) -> dict:
        """Run segmentation and return a unified structure.

        Returns:
            dict with keys:
            - object_masks: list[np.ndarray(bool)]
            - person_mask: np.ndarray(bool)
            - scores: list[float]
            - labels: list[str]
        """
