"""Kalman-like 3D tracker for fused RGBD pipeline.

State: [x, y, z, vx, vy, vz, sx, sy, sz]
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class KalmanTrack3D:
    track_id: int
    state: np.ndarray
    score: float
    label: str
    hits: int = 1
    age: int = 1
    missing_count: int = 0


class KalmanTracker3D:
    """Lightweight tracker with distance-size association and size smoothing."""

    def __init__(
        self,
        dist_gate_m: float = 0.08,
        max_age: int = 8,
        min_hits: int = 3,
        size_alpha: float = 0.25,
        duplicate_track_dist_m: float = 0.04,
    ):
        self.dist_gate_m = dist_gate_m
        self.max_age = max_age
        self.min_hits = min_hits
        self.size_alpha = size_alpha
        self.duplicate_track_dist_m = duplicate_track_dist_m
        self.tracks: dict[int, KalmanTrack3D] = {}
        self._next_id = 1

    def update(self, detections: list[dict], dt: float = 1.0) -> list[dict]:
        self._predict(dt)

        unmatched_det = list(range(len(detections)))
        unmatched_tracks = list(self.tracks.keys())
        matches: list[tuple[int, int]] = []

        if detections and self.tracks:
            cost = self._build_cost_matrix(detections)
            while True:
                if cost.size == 0:
                    break
                tidx, didx = np.unravel_index(np.argmin(cost), cost.shape)
                best = float(cost[tidx, didx])
                if best >= 1e6:
                    break
                track_id = unmatched_tracks[tidx]
                det_id = unmatched_det[didx]
                matches.append((track_id, det_id))
                cost[tidx, :] = 1e6
                cost[:, didx] = 1e6

            matched_tracks = {t for t, _ in matches}
            matched_dets = {d for _, d in matches}
            unmatched_tracks = [t for t in unmatched_tracks if t not in matched_tracks]
            unmatched_det = [d for d in unmatched_det if d not in matched_dets]

        for track_id, det_idx in matches:
            self._update_track(self.tracks[track_id], detections[det_idx])

        for track_id in unmatched_tracks:
            tr = self.tracks[track_id]
            tr.missing_count += 1
            tr.age += 1

        for det_idx in unmatched_det:
            if self._is_duplicate_new_detection(detections[det_idx]):
                continue
            self._create_track(detections[det_idx])

        self._cleanup()
        return self._export_tracks()

    def _predict(self, dt: float):
        for tr in self.tracks.values():
            # Avoid smooth drifting for unmatched tracks.
            if tr.missing_count > 0:
                tr.state[3:6] *= 0.5
                continue
            tr.state[0:3] += tr.state[3:6] * dt

    def _build_cost_matrix(self, detections: list[dict]) -> np.ndarray:
        track_ids = list(self.tracks.keys())
        cost = np.full((len(track_ids), len(detections)), 1e6, dtype=np.float32)
        for i, track_id in enumerate(track_ids):
            t = self.tracks[track_id]
            t_pos = t.state[0:3]
            t_size = t.state[6:9]
            for j, det in enumerate(detections):
                d_bbox = np.asarray(det["bbox"], dtype=np.float32)
                d_pos = d_bbox[0:3]
                d_size = d_bbox[3:6]
                dist = float(np.linalg.norm(t_pos - d_pos))
                if dist > self.dist_gate_m:
                    continue
                size_diff = float(np.linalg.norm(t_size - d_size))
                cost[i, j] = dist + 0.2 * size_diff
        return cost

    def _update_track(self, track: KalmanTrack3D, det: dict):
        bbox = np.asarray(det["bbox"], dtype=np.float32)
        prev_pos = track.state[0:3].copy()
        meas_pos = bbox[0:3]
        meas_size = bbox[3:6]
        occ = float(det.get("occlusion_ratio", 0.0))
        size_weight = max(0.05, 1.0 - occ)

        track.state[3:6] = meas_pos - prev_pos
        track.state[0:3] = meas_pos
        alpha = self.size_alpha * size_weight
        track.state[6:9] = (1.0 - alpha) * track.state[6:9] + alpha * meas_size

        track.score = float(det.get("score", track.score))
        track.label = det.get("label", track.label)
        track.hits += 1
        track.age += 1
        track.missing_count = 0

    def _create_track(self, det: dict):
        bbox = np.asarray(det["bbox"], dtype=np.float32)
        state = np.zeros(9, dtype=np.float32)
        state[0:3] = bbox[0:3]
        state[6:9] = np.maximum(bbox[3:6], 1e-3)
        track = KalmanTrack3D(
            track_id=self._next_id,
            state=state,
            score=float(det.get("score", 1.0)),
            label=str(det.get("label", "object")),
        )
        self.tracks[self._next_id] = track
        self._next_id += 1

    def _is_duplicate_new_detection(self, det: dict) -> bool:
        if not self.tracks:
            return False
        center = np.asarray(det["bbox"][0:3], dtype=np.float32)
        for tr in self.tracks.values():
            t_center = tr.state[0:3]
            if float(np.linalg.norm(center - t_center)) <= self.duplicate_track_dist_m:
                return True
        return False

    def _cleanup(self):
        stale_ids = [
            tid for tid, tr in self.tracks.items() if tr.missing_count > self.max_age
        ]
        for tid in stale_ids:
            del self.tracks[tid]

    def _export_tracks(self) -> list[dict]:
        candidates = []
        for tr in self.tracks.values():
            if tr.hits < self.min_hits:
                continue
            candidates.append(
                {
                    "track_id": tr.track_id,
                    "bbox": [
                        float(tr.state[0]),
                        float(tr.state[1]),
                        float(tr.state[2]),
                        float(tr.state[6]),
                        float(tr.state[7]),
                        float(tr.state[8]),
                        0.0,
                    ],
                    "label": tr.label,
                    "score": tr.score,
                    "hits": tr.hits,
                    "age": tr.age,
                }
            )

        # Remove near-duplicate tracks, keep the one with higher hits/score.
        if len(candidates) <= 1:
            return candidates

        keep = [True] * len(candidates)
        for i in range(len(candidates)):
            if not keep[i]:
                continue
            ci = np.asarray(candidates[i]["bbox"][:3], dtype=np.float32)
            for j in range(i + 1, len(candidates)):
                if not keep[j]:
                    continue
                cj = np.asarray(candidates[j]["bbox"][:3], dtype=np.float32)
                dist = float(np.linalg.norm(ci - cj))
                if dist > self.duplicate_track_dist_m:
                    continue

                # Prefer longer-lived track, then higher score.
                ai = (int(candidates[i]["hits"]), float(candidates[i]["score"]))
                aj = (int(candidates[j]["hits"]), float(candidates[j]["score"]))
                if aj > ai:
                    keep[i] = False
                    break
                keep[j] = False

        return [c for c, k in zip(candidates, keep, strict=False) if k]
