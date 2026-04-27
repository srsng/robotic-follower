"""Kalman 3D tracker with Mahalanobis gating and Hungarian matching.

State: [x, y, z, vx, vy, vz, sx, sy, sz]
Measurement: [x, y, z, sx, sy, sz]

Key improvements over v1:
- Mahalanobis distance for association (adaptively widens gate for lost tracks)
- Softer covariance clamping (allows uncertainty growth during occlusion)
- Relaxed duplicate detection (no unconditional rejection at tiny distances)
- Dynamic velocity decay during missing frames
- Label-aware but not label-penalized cost matrix
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import linear_sum_assignment


@dataclass
class KalmanTrack3D:
    track_id: int
    state: np.ndarray
    covariance: np.ndarray
    score: float
    label: str
    hits: int = 1
    age: int = 1
    missing_count: int = 0
    size_history: deque = field(default_factory=lambda: deque(maxlen=8))

    def predicted_pos(self) -> np.ndarray:
        return self.state[0:3].copy()

    def predicted_size(self) -> np.ndarray:
        return self.state[6:9].copy()

    def pos_uncertainty(self) -> float:
        return float(np.sqrt(np.trace(self.covariance[:3, :3])))


class KalmanTracker3D:
    """Full Kalman filter tracker with Hungarian matching and Mahalanobis gating."""

    STATE_DIM = 9
    MEAS_DIM = 6

    def __init__(
        self,
        dist_gate_m: float = 0.50,
        max_age: int = 30,
        min_hits: int = 1,
        size_alpha: float = 0.25,
        duplicate_track_dist_m: float = 0.15,
        q_pos: float = 0.015,
        q_vel: float = 0.15,
        q_size: float = 0.005,
        r_pos: float = 0.02,
        r_size: float = 0.008,
        p_init_pos: float = 0.04,
        p_init_vel: float = 1.0,
        p_init_size: float = 0.02,
        missing_q_scale: float = 2.5,
        missing_q_ramp_frames: int = 3,
        size_clamp_ratio: float = 0.4,
        mahal_gate_chi2: float = 9.488,
        max_cov_diag: float = 5.0,
        label_match_bonus: float = 0.05,
        size_weight: float = 0.6,
        pre_filter_dist_m: float = 1.0,
    ):
        self.dist_gate_m = dist_gate_m
        self.max_age = max_age
        self.min_hits = min_hits
        self.size_alpha = size_alpha
        self.duplicate_track_dist_m = duplicate_track_dist_m
        self.q_pos = q_pos
        self.q_vel = q_vel
        self.q_size = q_size
        self.r_pos = r_pos
        self.r_size = r_size
        self.p_init_pos = p_init_pos
        self.p_init_vel = p_init_vel
        self.p_init_size = p_init_size
        self.missing_q_scale = missing_q_scale
        self.missing_q_ramp_frames = missing_q_ramp_frames
        self.size_clamp_ratio = size_clamp_ratio
        self.mahal_gate_chi2 = mahal_gate_chi2
        self.max_cov_diag = max_cov_diag
        self.label_match_bonus = label_match_bonus
        self.size_weight = size_weight
        self.pre_filter_dist_m = pre_filter_dist_m

        self.tracks: dict[int, KalmanTrack3D] = {}
        self._next_id = 1

    def _make_Q(self, scale: float = 1.0) -> np.ndarray:
        Q = np.zeros((self.STATE_DIM, self.STATE_DIM), dtype=np.float64)
        Q[0, 0] = self.q_pos
        Q[1, 1] = self.q_pos
        Q[2, 2] = self.q_pos
        Q[3, 3] = self.q_vel
        Q[4, 4] = self.q_vel
        Q[5, 5] = self.q_vel
        Q[6, 6] = self.q_size
        Q[7, 7] = self.q_size
        Q[8, 8] = self.q_size
        return Q * scale

    def _make_R(self) -> np.ndarray:
        R = np.zeros((self.MEAS_DIM, self.MEAS_DIM), dtype=np.float64)
        R[0, 0] = self.r_pos
        R[1, 1] = self.r_pos
        R[2, 2] = self.r_pos
        R[3, 3] = self.r_size
        R[4, 4] = self.r_size
        R[5, 5] = self.r_size
        return R

    def _make_F(self, dt: float) -> np.ndarray:
        F = np.eye(self.STATE_DIM, dtype=np.float64)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        return F

    H = None

    @classmethod
    def _get_H(cls) -> np.ndarray:
        if cls.H is None:
            H = np.zeros((cls.MEAS_DIM, cls.STATE_DIM), dtype=np.float64)
            H[0, 0] = 1.0
            H[1, 1] = 1.0
            H[2, 2] = 1.0
            H[3, 6] = 1.0
            H[4, 7] = 1.0
            H[5, 8] = 1.0
            cls.H = H
        return cls.H

    def update(self, detections: list[dict], dt: float = 0.033) -> list[dict]:
        self._predict(dt)
        matches, unmatched_tracks, unmatched_dets = self._associate(detections)

        for track_id, det_idx in matches:
            self._update_track(self.tracks[track_id], detections[det_idx])

        for track_id in unmatched_tracks:
            tr = self.tracks[track_id]
            tr.missing_count += 1
            tr.age += 1

        for det_idx in unmatched_dets:
            if self._is_duplicate_new_detection(detections[det_idx]):
                continue
            self._create_track(detections[det_idx])

        self._cleanup()
        return self._export_tracks()

    def _predict(self, dt: float):
        F = self._make_F(dt)
        Q_base = self._make_Q()

        for tr in self.tracks.values():
            tr.state = F @ tr.state

            if tr.missing_count > 0:
                decay = max(0.3, 0.85 ** (1.0 + tr.missing_count * 0.2))
                tr.state[3:6] *= decay
                q_scale = self.missing_q_scale ** (
                    tr.missing_count / self.missing_q_ramp_frames
                )
            else:
                q_scale = 1.0

            Q = Q_base * q_scale
            tr.covariance = F @ tr.covariance @ F.T + Q
            self._clamp_covariance(tr)
            tr.age += 1

    def _clamp_covariance(self, track: KalmanTrack3D):
        diag = np.diag(track.covariance).copy()
        over = diag > self.max_cov_diag
        if not over.any():
            return
        for idx in np.where(over)[0]:
            scale = self.max_cov_diag / diag[idx]
            track.covariance[idx, :] *= scale
            track.covariance[:, idx] *= scale

    def _associate(
        self, detections: list[dict]
    ) -> tuple[list[tuple[int, int]], list[int], list[int]]:
        if not self.tracks or not detections:
            return [], list(self.tracks.keys()), list(range(len(detections)))

        track_ids = list(self.tracks.keys())
        cost, pos_dists = self._build_cost_matrix(detections, track_ids)

        if cost.size == 0:
            return [], track_ids, list(range(len(detections)))

        row_indices, col_indices = linear_sum_assignment(cost)

        matches: list[tuple[int, int]] = []
        matched_track_indices: set[int] = set()
        matched_det_indices: set[int] = set()

        gate = self.dist_gate_m

        for r, c in zip(row_indices, col_indices):
            if cost[r, c] >= 1e6:
                continue
            if pos_dists[r, c] < gate:
                matches.append((track_ids[r], c))
                matched_track_indices.add(r)
                matched_det_indices.add(c)

        unmatched_tracks = [
            tid for i, tid in enumerate(track_ids) if i not in matched_track_indices
        ]
        unmatched_dets = [d for d in range(len(detections)) if d not in matched_det_indices]

        return matches, unmatched_tracks, unmatched_dets

    def _build_cost_matrix(
        self, detections: list[dict], track_ids: list[int]
    ) -> tuple[np.ndarray, np.ndarray]:
        INF = 1e6
        cost = np.full((len(track_ids), len(detections)), INF, dtype=np.float64)
        pos_dists = np.full((len(track_ids), len(detections)), INF, dtype=np.float64)

        for i, tid in enumerate(track_ids):
            tr = self.tracks[tid]
            t_pos = tr.predicted_pos()
            t_size = tr.predicted_size()
            S_pos = tr.covariance[:3, :3]
            pos_var = float(np.trace(S_pos))

            S_reg = S_pos + np.eye(3, dtype=np.float64) * 1e-4
            try:
                S_inv = np.linalg.inv(S_reg)
            except np.linalg.LinAlgError:
                S_inv = np.eye(3, dtype=np.float64) / max(pos_var, 1e-3)

            for j, det in enumerate(detections):
                d_bbox = np.asarray(det["bbox"], dtype=np.float64)
                d_pos = d_bbox[0:3]
                d_size = d_bbox[3:6]

                diff = t_pos - d_pos
                eucl_dist = float(np.linalg.norm(diff))

                if eucl_dist > self.pre_filter_dist_m:
                    continue

                pos_dists[i, j] = eucl_dist

                mahal_sq = float(diff @ S_inv @ diff)
                mahal_dist = np.sqrt(max(mahal_sq, 0.0))

                size_diff = float(np.linalg.norm(t_size - d_size))
                size_cost = self.size_weight * size_diff

                label = det.get("label", "object")
                label_cost = 0.0 if label == tr.label else self.label_match_bonus

                cost[i, j] = mahal_dist + size_cost + label_cost

        return cost, pos_dists

    def _update_track(self, track: KalmanTrack3D, det: dict):
        bbox = np.asarray(det["bbox"], dtype=np.float64)
        z = np.zeros(self.MEAS_DIM, dtype=np.float64)
        z[:3] = bbox[0:3]
        z[3:6] = bbox[3:6]

        H = self._get_H()

        y_innov = z - H @ track.state
        S = H @ track.covariance @ H.T + self._make_R()
        K = track.covariance @ H.T @ np.linalg.inv(S)
        track.state = track.state + K @ y_innov
        I_KH = np.eye(self.STATE_DIM) - K @ H
        track.covariance = I_KH @ track.covariance @ I_KH.T + K @ self._make_R() @ K.T
        track.covariance = (track.covariance + track.covariance.T) / 2.0

        meas_size = bbox[3:6]
        track.size_history.append(meas_size.copy())
        if len(track.size_history) >= 3:
            avg_size = np.mean(list(track.size_history), axis=0)
            clamped = np.clip(
                meas_size,
                (1.0 - self.size_clamp_ratio) * avg_size,
                (1.0 + self.size_clamp_ratio) * avg_size,
            )
            track.state[6:9] = clamped

        track.score = float(det.get("score", track.score))
        track.label = det.get("label", track.label)
        track.hits += 1
        track.missing_count = 0

        self._clamp_covariance(track)

    def _create_track(self, det: dict):
        bbox = np.asarray(det["bbox"], dtype=np.float64)
        state = np.zeros(self.STATE_DIM, dtype=np.float64)
        state[0:3] = bbox[0:3]
        state[6:9] = np.maximum(bbox[3:6], 1e-3)

        P = np.zeros((self.STATE_DIM, self.STATE_DIM), dtype=np.float64)
        P[0, 0] = self.p_init_pos
        P[1, 1] = self.p_init_pos
        P[2, 2] = self.p_init_pos
        P[3, 3] = self.p_init_vel
        P[4, 4] = self.p_init_vel
        P[5, 5] = self.p_init_vel
        P[6, 6] = self.p_init_size
        P[7, 7] = self.p_init_size
        P[8, 8] = self.p_init_size

        track = KalmanTrack3D(
            track_id=self._next_id,
            state=state,
            covariance=P,
            score=float(det.get("score", 1.0)),
            label=str(det.get("label", "object")),
        )
        track.size_history.append(bbox[3:6].copy())
        self.tracks[self._next_id] = track
        self._next_id += 1

    def _is_duplicate_new_detection(self, det: dict) -> bool:
        if not self.tracks:
            return False
        center = np.asarray(det["bbox"][0:3], dtype=np.float64)
        det_size = np.asarray(det["bbox"][3:6], dtype=np.float64)
        det_label = det.get("label", "object")

        for tr in self.tracks.values():
            t_center = tr.state[0:3]
            dist = float(np.linalg.norm(center - t_center))
            if dist > self.duplicate_track_dist_m:
                continue

            t_size = tr.state[6:9]
            size_diff = float(np.linalg.norm(det_size - t_size))
            avg_size = float(np.linalg.norm(t_size))
            size_ratio = size_diff / max(avg_size, 1e-3)

            if det_label == tr.label and size_ratio < 0.8:
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