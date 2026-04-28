"""EMA-based 3D tracker that uses 2D track IDs for direct association.

Instead of Hungarian matching in 3D (which is noisy), this tracker relies on
2D track IDs from ultralytics BoT-SORT/ByteTrack for frame-to-frame identity,
and applies exponential moving average (EMA) smoothing on 3D positions.

Key difference from KalmanTracker3D:
- No data association problem: 2D tracker provides stable IDs
- EMA smoothing is simpler and more robust than Kalman filtering for noisy 3D
- Velocity estimation for short-term prediction during brief depth dropouts
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class EMATrack3D:
    track_id: int
    bbox: np.ndarray
    raw_bbox: np.ndarray
    label: str
    score: float
    hits: int = 1
    age: int = 1
    missing_count: int = 0
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    size_history: list[np.ndarray] = field(default_factory=list)


class EMATracker3D:
    def __init__(
        self,
        alpha_pos: float = 0.40,
        alpha_size: float = 0.20,
        max_age: int = 15,
        min_hits: int = 1,
        velocity_decay: float = 0.70,
        max_velocity: float = 2.0,
        duplicate_track_dist_m: float = 0.15,
        size_clamp_ratio: float = 0.4,
    ):
        self.alpha_pos = alpha_pos
        self.alpha_size = alpha_size
        self.max_age = max_age
        self.min_hits = min_hits
        self.velocity_decay = velocity_decay
        self.max_velocity = max_velocity
        self.duplicate_track_dist_m = duplicate_track_dist_m
        self.size_clamp_ratio = size_clamp_ratio
        self.tracks: dict[int, EMATrack3D] = {}
        self._next_untracked_id: int = -1

    def update(self, detections: list[dict], dt: float = 0.033) -> list[dict]:
        current_ids: set[int] = set()

        for det in detections:
            tid = det.get("2d_track_id")
            if tid is None:
                tid = self._assign_untracked_id(detections, det, current_ids)
                if tid is None:
                    continue

            current_ids.add(tid)
            new_bbox = np.asarray(det["bbox"], dtype=np.float64)
            label = str(det.get("label", "object"))
            score = float(det.get("score", 1.0))

            if tid in self.tracks:
                self._update_existing_track(tid, new_bbox, label, score, dt)
            else:
                self._create_track(tid, new_bbox, label, score)

        for tid in list(self.tracks.keys()):
            if tid not in current_ids:
                self._mark_missing(tid, dt)

        self._cleanup()
        return self._export_tracks()

    def _assign_untracked_id(
        self,
        all_detections: list[dict],
        det: dict,
        current_ids: set[int],
    ) -> int | None:
        bbox = np.asarray(det["bbox"], dtype=np.float64)
        pos = bbox[:3]
        best_id: int | None = None
        best_dist = float(self.duplicate_track_dist_m)

        for tid, track in self.tracks.items():
            if tid in current_ids:
                continue
            if track.missing_count > 3:
                continue
            dist = float(np.linalg.norm(track.bbox[:3] - pos))
            if dist < best_dist:
                label_match = track.label == det.get("label", "object")
                if label_match or dist < best_dist * 0.5:
                    best_dist = dist
                    best_id = tid

        if best_id is not None:
            return best_id

        tid = self._next_untracked_id
        self._next_untracked_id -= 1
        return tid

    def _update_existing_track(
        self,
        tid: int,
        new_bbox: np.ndarray,
        label: str,
        score: float,
        dt: float,
    ):
        track = self.tracks[tid]
        prev_pos = track.bbox[:3].copy()

        track.bbox[:3] = self.alpha_pos * new_bbox[:3] + (1 - self.alpha_pos) * track.bbox[:3]

        if len(track.size_history) >= 3:
            avg_size = np.mean(track.size_history, axis=0)
            clamped = np.clip(
                new_bbox[3:6],
                (1.0 - self.size_clamp_ratio) * avg_size,
                (1.0 + self.size_clamp_ratio) * avg_size,
            )
            track.bbox[3:6] = self.alpha_size * clamped + (1 - self.alpha_size) * track.bbox[3:6]
        else:
            track.bbox[3:6] = self.alpha_size * new_bbox[3:6] + (1 - self.alpha_size) * track.bbox[3:6]

        track.size_history.append(new_bbox[3:6].copy())
        if len(track.size_history) > 8:
            track.size_history.pop(0)

        track.bbox[6] = new_bbox[6]

        if dt > 1e-6:
            raw_velocity = (track.bbox[:3] - prev_pos) / dt
            speed = float(np.linalg.norm(raw_velocity))
            if speed > self.max_velocity:
                raw_velocity = raw_velocity / max(speed, 1e-9) * self.max_velocity
            track.velocity = raw_velocity

        track.raw_bbox = new_bbox.copy()
        track.label = label
        track.score = score
        track.hits += 1
        track.age += 1
        track.missing_count = 0

    def _create_track(self, tid: int, bbox: np.ndarray, label: str, score: float):
        state = bbox.copy()
        self.tracks[tid] = EMATrack3D(
            track_id=tid,
            bbox=state,
            raw_bbox=bbox.copy(),
            label=label,
            score=score,
            size_history=[bbox[3:6].copy()],
        )

    def _mark_missing(self, tid: int, dt: float):
        track = self.tracks[tid]
        track.missing_count += 1
        track.age += 1
        track.velocity *= self.velocity_decay
        if track.missing_count <= 3:
            if dt > 1e-6:
                track.bbox[:3] += track.velocity * dt

    def _cleanup(self):
        to_remove = [
            tid for tid, t in self.tracks.items() if t.missing_count > self.max_age
        ]
        for tid in to_remove:
            del self.tracks[tid]

    def _export_tracks(self) -> list[dict]:
        candidates = []
        for track in self.tracks.values():
            if track.hits < self.min_hits:
                continue
            candidates.append(
                {
                    "track_id": track.track_id,
                    "bbox": [
                        float(track.bbox[0]),
                        float(track.bbox[1]),
                        float(track.bbox[2]),
                        float(track.bbox[3]),
                        float(track.bbox[4]),
                        float(track.bbox[5]),
                        float(track.bbox[6]),
                    ],
                    "label": track.label,
                    "score": track.score,
                    "hits": track.hits,
                    "age": track.age,
                }
            )

        if len(candidates) <= 1:
            return candidates

        keep = [True] * len(candidates)
        for i in range(len(candidates)):
            if not keep[i]:
                continue
            ci = np.asarray(candidates[i]["bbox"][:3], dtype=np.float64)
            si = np.asarray(candidates[i]["bbox"][3:6], dtype=np.float64)
            for j in range(i + 1, len(candidates)):
                if not keep[j]:
                    continue
                cj = np.asarray(candidates[j]["bbox"][:3], dtype=np.float64)
                sj = np.asarray(candidates[j]["bbox"][3:6], dtype=np.float64)
                dist = float(np.linalg.norm(ci - cj))
                if dist > self.duplicate_track_dist_m:
                    continue
                size_ratio_diff = abs(
                    float(np.linalg.norm(si)) - float(np.linalg.norm(sj))
                )
                size_avg = max(float(np.linalg.norm(si)), float(np.linalg.norm(sj)), 1e-3)
                if size_ratio_diff / size_avg > 0.6:
                    continue
                ai = (int(candidates[i]["hits"]), float(candidates[i]["score"]))
                aj = (int(candidates[j]["hits"]), float(candidates[j]["score"]))
                if aj > ai:
                    keep[i] = False
                    break
                keep[j] = False

        return [c for c, k in zip(candidates, keep, strict=False) if k]