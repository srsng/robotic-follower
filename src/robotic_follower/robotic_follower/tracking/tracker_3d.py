"""3D 多目标追踪器。

基于 IOU 匹配和匀速模型的多目标追踪算法（类似 SORT/DeepSORT 的简化版）。
"""

from collections import OrderedDict
from dataclasses import dataclass, field

import numpy as np

from robotic_follower.util.handler import NodeHandler


@dataclass
class Track:
    """单条跟踪轨迹。"""

    track_id: int
    bbox: np.ndarray  # [x, y, z, dx, dy, dz, yaw]
    label: str
    score: float
    hits: int = 1
    age: int = 1
    missing_count: int = 0
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    last_bbox: np.ndarray = field(default_factory=lambda: np.zeros(7))

    @property
    def is_confirmed(self) -> bool:
        """是否已确认为有效轨迹。"""
        return self.hits >= 3


class Tracker3D(NodeHandler):
    """3D 多目标追踪器。

    使用 IOU 匹配和匀速模型进行多目标跟踪。
    """

    def __init__(
        self,
        iou_threshold: float = 0.3,
        max_age: int = 30,
        min_hits: int = 3,
        max_speed: float = 2.0,
        parent_node=None,
    ):
        """初始化追踪器。

        Args:
            iou_threshold: IOU 匹配阈值，低于该值的匹配被认为不匹配
            max_age: 最大丢失帧数，超过该值则删除轨迹
            min_hits: 最小命中帧数，达到该值才认为轨迹有效
            max_speed: 最大速度限制 (m/frame)，用于防止大跳变
        """
        super().__init__(parent_node)

        self.iou_threshold = iou_threshold
        self.max_age = max_age
        self.min_hits = min_hits
        self.max_speed = max_speed

        self.tracks: OrderedDict[int, Track] = OrderedDict()
        self._next_id = 1

    def update(self, detections: list[dict], header=None) -> list[dict]:
        """更新追踪器，处理新一帧的检测结果。

        Args:
            detections: 检测结果列表，每个元素包含:
                - bbox: [x, y, z, dx, dy, dz, yaw]
                - label: 类别标签
                - score: 置信度
            header: ROS 消息头（当前未使用）

        Returns:
            有效跟踪目标列表
        """
        if not detections:
            return self._handle_no_detections()

        det_bboxes = np.array([d["bbox"] for d in detections])

        # 预测现有轨迹的位置
        self._predict()

        # 匹配检测和轨迹
        matches, unmatched_tracks, unmatched_detections = self._match(det_bboxes)

        # 更新匹配的轨迹
        for track_id, det_idx in matches:
            self._update_track(self.tracks[track_id], detections[det_idx])

        # 处理未匹配的轨迹
        for track_id in unmatched_tracks:
            self.tracks[track_id].missing_count += 1

        # 创建新轨迹
        for det_idx in unmatched_detections:
            self._create_track(detections[det_idx])

        # 清理已丢失轨迹
        self._cleanup()

        return self._get_valid_tracks()

    def _predict(self):
        """预测所有轨迹的下一位置（匀速模型）。"""
        for track in self.tracks.values():
            if track.missing_count == 0:
                # 保存当前位置（预测前位置），用于后续速度计算
                track.last_bbox = track.bbox.copy()
                # 匀速预测：当前位置 + 速度
                track.bbox[0:3] += track.velocity

    def _match(self, det_bboxes: np.ndarray):
        """匹配检测和轨迹。

        Returns:
            matches: 已匹配的 (track_id, det_idx) 列表
            unmatched_tracks: 未匹配的 track_id 列表
            unmatched_detections: 未匹配的 det_idx 列表
        """
        if not self.tracks:
            return [], [], list(range(len(det_bboxes)))

        track_bboxes = np.array([t.bbox for t in self.tracks.values()])
        track_ids = list(self.tracks.keys())

        # 计算 IOU 矩阵
        iou_matrix = self._compute_iou_matrix(track_bboxes, det_bboxes)

        # 贪心匹配
        matches = []
        unmatched_tracks = list(track_ids)

        # 贪心匹配
        while unmatched_tracks and len(det_bboxes) > 0:
            best_match = None
            best_iou = self.iou_threshold

            for track_id in unmatched_tracks:
                track_idx = track_ids.index(track_id)
                for det_idx in range(len(det_bboxes)):
                    if det_idx in [m[1] for m in matches]:
                        continue
                    iou = iou_matrix[track_idx, det_idx]
                    if iou > best_iou:
                        best_iou = iou
                        best_match = (track_id, det_idx)

            if best_match is None:
                break

            matches.append(best_match)
            unmatched_tracks.remove(best_match[0])

        unmatched_detections = [
            i for i in range(len(det_bboxes)) if i not in [m[1] for m in matches]
        ]

        return matches, unmatched_tracks, unmatched_detections

    def _compute_iou_matrix(
        self, bboxes1: np.ndarray, bboxes2: np.ndarray
    ) -> np.ndarray:
        """计算两组 bbox 之间的 IOU 矩阵。

        Args:
            bboxes1: 轨迹 bbox (N, 7) [x, y, z, dx, dy, dz, yaw]
            bboxes2: 检测 bbox (M, 7)

        Returns:
            IOU 矩阵 (N, M)
        """
        n, m = len(bboxes1), len(bboxes2)
        iou_matrix = np.zeros((n, m))

        for i in range(n):
            for j in range(m):
                iou_matrix[i, j] = self._compute_iou_3d(bboxes1[i], bboxes2[j])

        return iou_matrix

    def _compute_iou_3d(self, bbox1: np.ndarray, bbox2: np.ndarray) -> float:
        """计算两个 3D bbox 的 IOU."""
        center1, size1 = bbox1[0:3], bbox1[3:6]
        center2, size2 = bbox2[0:3], bbox2[3:6]

        # 计算重叠区域
        overlap_min = np.maximum(center1 - size1 / 2, center2 - size2 / 2)
        overlap_max = np.minimum(center1 + size1 / 2, center2 + size2 / 2)
        overlap_size = np.maximum(0.0, overlap_max - overlap_min)
        overlap_volume = np.prod(overlap_size)
        vol1 = np.prod(size1)
        vol2 = np.prod(size2)
        union_volume = vol1 + vol2 - overlap_volume

        if union_volume <= 0:
            return 0.0

        return float(np.clip(overlap_volume / union_volume, 0, 1))

    def _update_track(self, track: Track, detection: dict):
        """更新已匹配的轨迹。"""
        new_bbox = np.array(detection["bbox"])

        # 首帧不计算速度，直接用检测初始化
        if track.age == 1:
            old_bbox = new_bbox.copy()
            track.velocity = np.zeros(3)
        elif track.missing_count > 0:
            # 丢失后重连：速度归零，用当前位置重新开始
            track.velocity = np.zeros(3)
            old_bbox = new_bbox.copy()
        else:
            # 正常更新：用 last_bbox（预测前位置）计算速度
            old_bbox = track.last_bbox.copy()

        # 计算并限制速度
        track.velocity = new_bbox[0:3] - old_bbox[0:3]
        speed = np.linalg.norm(track.velocity)
        if speed > self.max_speed:
            track.velocity = track.velocity / speed * self.max_speed

        # 更新轨迹
        track.bbox = new_bbox
        track.label = detection["label"]
        track.score = detection["score"]
        track.hits += 1
        track.age += 1
        track.missing_count = 0

    def _create_track(self, detection: dict):
        """创建新轨迹。"""
        track_id = self._next_id
        self._next_id += 1

        track = Track(
            track_id=track_id,
            bbox=np.array(detection["bbox"]),
            label=detection["label"],
            score=detection["score"],
            velocity=np.zeros(3),
            last_bbox=np.array(detection["bbox"]),
        )
        self.tracks[track_id] = track

    def _handle_no_detections(self) -> list[dict]:
        """没有检测时的处理。"""
        for track in self.tracks.values():
            track.missing_count += 1
            track.age += 1

        self._cleanup()
        return self._get_valid_tracks()

    def _cleanup(self):
        """清理已丢失的轨迹。"""
        to_remove = [
            track_id
            for track_id, track in self.tracks.items()
            if track.missing_count > self.max_age
        ]
        for track_id in to_remove:
            del self.tracks[track_id]

    def _get_valid_tracks(self) -> list[dict]:
        """获取所有有效轨迹。"""
        result = []
        for track in self.tracks.values():
            if track.hits >= self.min_hits:
                result.append(
                    {
                        "track_id": track.track_id,
                        "bbox": track.bbox.tolist(),
                        "label": track.label,
                        "score": track.score,
                        "hits": track.hits,
                        "age": track.age,
                    }
                )
        return result


__all__ = ["Tracker3D", "Track"]
