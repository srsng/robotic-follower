#!/usr/bin/env python3
"""融合 RGBD 检测+追踪节点。"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

import cv2
import message_filters
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Quaternion, Vector3
from rclpy.duration import Duration
from robotic_follower_msgs.msg import TrackedObject3D, TrackedObject3DArray
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from robotic_follower.segmentation import create_segmenter_from_config
from robotic_follower.tracking.kalman_tracker_3d import KalmanTracker3D
from robotic_follower.util.wrapper import NodeWrapper


@dataclass
class DetectionCandidate:
    bbox: list[float]
    score: float
    label: str
    occlusion_ratio: float
    graspable: bool


class RgbdDetectTrackNode(NodeWrapper):
    """融合节点：分割、2.5D 投影、AABB、3D 追踪。"""

    def __init__(self):
        super().__init__("rgbd_detect_track_node")

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.target_frame = self.declare_and_get_parameter("target_frame", "base_link")
        self.base_frame = self.target_frame

        self.depth_valid_ratio_min = self.declare_and_get_parameter(
            "depth_valid_ratio_min", 0.6
        )

        self.mask_area_min_px = self.declare_and_get_parameter("mask_area_min_px", 400)

        self.mask_aspect_ratio_max = self.declare_and_get_parameter(
            "mask_aspect_ratio_max", 6.0
        )

        self.z_trim_quantile = self.declare_and_get_parameter("z_trim_quantile", 0.05)
        self.table_margin_m = self.declare_and_get_parameter("table_margin_m", 0.01)
        self.table_z = self.declare_and_get_parameter("table_z", 0.0)
        self.table_reestimate_interval_s = self.declare_and_get_parameter(
            "table_reestimate_interval_s", 2.0
        )
        self.table_reestimate_min_inlier_ratio = self.declare_and_get_parameter(
            "table_reestimate_min_inlier_ratio", 0.35
        )
        self.tf_stale_threshold_ms = self.declare_and_get_parameter(
            "tf_stale_threshold_ms", 80.0
        )
        self.sync_skew_threshold_ms = self.declare_and_get_parameter(
            "sync_skew_threshold_ms", 30.0
        )
        self.tf_fallback_warn_every = self.declare_and_get_parameter(
            "tf_fallback_warn_every", 45
        )
        self.table_warn_every = self.declare_and_get_parameter("table_warn_every", 45)
        self.sync_warn_every = self.declare_and_get_parameter("sync_warn_every", 45)
        self.association_dist_gate_m = self.declare_and_get_parameter(
            "association_dist_gate_m", 0.08
        )
        self.detection_merge_dist_m = self.declare_and_get_parameter(
            "detection_merge_dist_m", 0.06
        )
        self.detection_merge_iou_min = self.declare_and_get_parameter(
            "detection_merge_iou_min", 0.18
        )
        self.duplicate_track_dist_m = self.declare_and_get_parameter(
            "duplicate_track_dist_m", 0.04
        )
        self.max_age = self.declare_and_get_parameter("max_age", 8)
        self.min_hits = self.declare_and_get_parameter("min_hits", 3)
        self.occlusion_ratio_max_for_grasp = self.declare_and_get_parameter(
            "occlusion_ratio_max_for_grasp", 0.45
        )
        self.max_non_person_distance_m = self.declare_and_get_parameter(
            "max_non_person_distance_m", 1.2
        )
        self.fallback_to_source_frame_when_tf_disconnected = bool(
            self.declare_and_get_parameter(
                "fallback_to_source_frame_when_tf_disconnected", True
            )
        )

        self.tf_lookup_fail_count = 0
        self.last_tf_age_ms = 0.0
        self.last_sync_skew_ms = 0.0
        self.last_stamp_ns: int | None = None
        self.last_table_reestimate_ns = 0
        self._warn_counters: dict[str, int] = {}
        self._current_output_frame = self.base_frame

        config_file = self.declare_and_get_parameter(
            "config_file", "model/config/yolov8_seg_rgbd_track.yaml"
        )
        config = self._load_config(config_file)
        segmenter_cfg = config.get("segmenter", {"type": "yolov8_seg"})
        self.segmenter = create_segmenter_from_config(segmenter_cfg, parent_node=self)

        self.tracker = KalmanTracker3D(
            dist_gate_m=self.association_dist_gate_m,
            max_age=self.max_age,
            min_hits=self.min_hits,
            duplicate_track_dist_m=self.duplicate_track_dist_m,
        )
        self.track_quality: dict[int, dict] = {}

        rgb_topic = self.declare_and_get_parameter(
            "rgb_topic", "/camera/camera/color/image_raw"
        )
        depth_topic = self.declare_and_get_parameter(
            "depth_topic", "/camera/camera/aligned_depth_to_color/image_raw"
        )
        camera_info_topic = self.declare_and_get_parameter(
            "camera_info_topic", "/camera/camera/color/camera_info"
        )

        self.rgb_sub = message_filters.Subscriber(self, Image, rgb_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        self.info_sub = message_filters.Subscriber(self, CameraInfo, camera_info_topic)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.info_sub], queue_size=10, slop=0.03
        )
        self.sync.registerCallback(self.synced_callback)

        self.tracked_pub = self.create_publisher(
            Detection3DArray, "/perception/tracked_objects", 10
        )
        self.detections_pub = self.create_publisher(
            Detection3DArray, "/perception/detections", 10
        )
        self.tracked_custom_pub = self.create_publisher(
            TrackedObject3DArray, "/perception/tracked_objects_custom", 10
        )
        self.enable_segmentation_debug_vis = bool(
            self.declare_and_get_parameter("enable_segmentation_debug_vis", True)
        )
        self.segmentation_debug_topic = self.declare_and_get_parameter(
            "segmentation_debug_topic", "/perception/segmentation_debug"
        )
        self.segmentation_debug_pub = self.create_publisher(
            Image,
            self.segmentation_debug_topic,
            10,
        )

        self._info("融合检测追踪节点已启动")

    def _load_config(self, config_file: str) -> dict:
        expanded = os.path.expanduser(config_file)
        tried_paths: list[str] = []
        if not os.path.isabs(expanded):
            from ament_index_python.packages import get_package_share_directory

            pkg_path = get_package_share_directory("robotic_follower")
            expanded = os.path.join(pkg_path, expanded)
            tried_paths.append(expanded)

            # Fallback to workspace source path when launch points to a config
            # file that exists in src but has not been installed yet.
            rel = config_file
            for parent in Path(__file__).resolve().parents:
                src_candidate = parent / "src" / "robotic_follower" / rel
                if src_candidate.exists():
                    expanded = str(src_candidate)
                    break
        if not os.path.exists(expanded):
            msg = f"配置文件不存在，使用默认配置: {expanded}"
            if tried_paths:
                msg += f"; 已尝试: {tried_paths}"
            self._warn(msg)
            return {}
        self._info(f"加载配置文件: {expanded}")
        with open(expanded) as f:
            return yaml.safe_load(f) or {}

    def synced_callback(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        try:
            self._update_sync_skew(rgb_msg, depth_msg)

            tf_data = self._lookup_transform(
                depth_msg.header.frame_id, rgb_msg.header.stamp
            )
            if tf_data is None:
                tracked = self.tracker.update([])
                self._publish_tracks(tracked, rgb_msg.header, is_stale=True)
                return
            t_mat, is_stale, output_frame = tf_data
            self._current_output_frame = output_frame

            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            depth = self._depth_to_meters(depth_msg)
            if rgb.shape[:2] != depth.shape[:2]:
                self._warn("RGB 和 Depth 分辨率不一致，跳过本帧")
                return

            seg = self.segmenter.segment(rgb)
            person_mask = seg["person_mask"]

            detections: list[DetectionCandidate] = []
            debug_raw_masks: list[np.ndarray] = []
            debug_cleaned_masks: list[np.ndarray] = []
            for mask, score, label in zip(
                seg["object_masks"], seg["scores"], seg["labels"], strict=False
            ):
                raw_mask = mask if mask.dtype == bool else mask.astype(bool)
                debug_raw_masks.append(raw_mask)
                cleaned_for_vis = self._compute_cleaned_mask(raw_mask, person_mask)
                if cleaned_for_vis is None:
                    debug_cleaned_masks.append(np.zeros_like(raw_mask, dtype=bool))
                else:
                    debug_cleaned_masks.append(cleaned_for_vis)

                cand = self._build_detection_candidate(
                    mask=raw_mask,
                    person_mask=person_mask,
                    score=score,
                    label=label,
                    depth=depth,
                    info_msg=info_msg,
                    t_mat=t_mat,
                    is_stale=is_stale,
                )
                if cand is not None:
                    detections.append(cand)

            detections = self._merge_detection_candidates(detections)

            if self.enable_segmentation_debug_vis:
                self._publish_segmentation_debug(
                    header=rgb_msg.header,
                    rgb=rgb,
                    person_mask=person_mask,
                    raw_masks=debug_raw_masks,
                    cleaned_masks=debug_cleaned_masks,
                    accepted_count=len(detections),
                )

            self._publish_raw_detections(detections, rgb_msg.header)

            det_dicts = [
                {
                    "bbox": d.bbox,
                    "score": d.score,
                    "label": d.label,
                    "occlusion_ratio": d.occlusion_ratio,
                    "graspable": d.graspable,
                }
                for d in detections
            ]
            tracked = self.tracker.update(det_dicts)
            self._update_track_quality(tracked, detections, is_stale)
            self._publish_tracks(tracked, rgb_msg.header, is_stale=is_stale)

            self._maybe_reestimate_table(depth, info_msg, t_mat)
        except Exception as exc:
            self._error(f"融合节点处理失败: {exc}")

    def _update_sync_skew(self, rgb_msg: Image, depth_msg: Image):
        rgb_ns = self._stamp_to_ns(rgb_msg.header.stamp)
        depth_ns = self._stamp_to_ns(depth_msg.header.stamp)
        self.last_sync_skew_ms = abs(rgb_ns - depth_ns) / 1e6
        if self.last_sync_skew_ms > self.sync_skew_threshold_ms:
            self._warn_throttled(
                f"sync_skew 过大: {self.last_sync_skew_ms:.1f} ms",
                key="sync_skew",
                period_frames=int(self.sync_warn_every),
            )

    def _lookup_transform(
        self, source_frame: str, stamp_msg
    ) -> tuple[np.ndarray, bool, str] | None:
        try:
            stamp = rclpy.time.Time.from_msg(stamp_msg)
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                source_frame,
                stamp,
                timeout=Duration(seconds=0.05),
            )
            q = transform.transform.rotation
            t = transform.transform.translation
            mat = np.eye(4, dtype=np.float32)
            mat[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            mat[:3, 3] = [t.x, t.y, t.z]

            tf_stamp_ns = self._stamp_to_ns(transform.header.stamp)
            req_ns = self._stamp_to_ns(stamp_msg)
            self.last_tf_age_ms = max(0.0, (req_ns - tf_stamp_ns) / 1e6)
            is_stale = (
                self.last_tf_age_ms > self.tf_stale_threshold_ms
                or self.last_sync_skew_ms > self.sync_skew_threshold_ms
            )
            return mat, is_stale, self.base_frame
        except Exception as exc:
            err_str = str(exc).lower()
            disconnected = (
                "not part of the same tree" in err_str
                or "could not find a connection" in err_str
                or "unconnected trees" in err_str
            )
            if disconnected and self.fallback_to_source_frame_when_tf_disconnected:
                self._warn_throttled(
                    f"TF 树不连通，回退到源坐标系发布: {source_frame}",
                    key="tf_disconnected_fallback",
                    period_frames=int(self.tf_fallback_warn_every),
                )
                self.last_tf_age_ms = self.tf_stale_threshold_ms + 1.0
                return np.eye(4, dtype=np.float32), True, source_frame

            # Fallback: latest available transform to avoid full pipeline starvation
            # when TF publisher lags behind image timestamps.
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    source_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.05),
                )
                q = transform.transform.rotation
                t = transform.transform.translation
                mat = np.eye(4, dtype=np.float32)
                mat[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
                mat[:3, 3] = [t.x, t.y, t.z]
                self.last_tf_age_ms = self.tf_stale_threshold_ms + 1.0
                self._warn_throttled(
                    f"TF 时间戳对齐失败，使用最新TF回退: {exc}",
                    key="tf_fallback",
                    period_frames=int(self.tf_fallback_warn_every),
                )
                return mat, True, self.base_frame
            except Exception:
                pass
            self.tf_lookup_fail_count += 1
            self._warn_throttled(
                f"TF 查询失败: {exc}",
                key="tf_fail",
                period_frames=int(self.tf_fallback_warn_every),
            )
            return None

    @staticmethod
    def _stamp_to_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _depth_to_meters(self, depth_msg: Image) -> np.ndarray:
        if depth_msg.encoding == "16UC1":
            depth_mm = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="16UC1")
            return depth_mm.astype(np.float32) * 0.001
        if depth_msg.encoding == "32FC1":
            return self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding="32FC1"
            ).astype(np.float32)
        depth = self.bridge.imgmsg_to_cv2(depth_msg)
        return depth.astype(np.float32)

    def _build_detection_candidate(
        self,
        mask: np.ndarray,
        person_mask: np.ndarray,
        score: float,
        label: str,
        depth: np.ndarray,
        info_msg: CameraInfo,
        t_mat: np.ndarray,
        is_stale: bool,
    ) -> DetectionCandidate | None:
        if mask.dtype != bool:
            mask = mask.astype(bool)

        raw_area = int(mask.sum())
        cleaned = self._compute_cleaned_mask(mask, person_mask)
        if cleaned is None:
            return None

        ys, xs = np.where(cleaned)
        if len(xs) == 0:
            return None
        width = float(xs.max() - xs.min() + 1)
        height = float(ys.max() - ys.min() + 1)
        aspect = max(width / max(height, 1.0), height / max(width, 1.0))
        if aspect > self.mask_aspect_ratio_max:
            return None

        points_cam, valid_ratio = self._mask_to_points(depth, cleaned, info_msg)
        if valid_ratio < self.depth_valid_ratio_min or len(points_cam) < 20:
            return None

        points_base = self._transform_points(points_cam, t_mat)
        points_base = self._filter_points(points_base)
        if len(points_base) < 10:
            return None

        points_base = points_base[
            points_base[:, 2] > (self.table_z + self.table_margin_m)
        ]
        if len(points_base) < 10:
            return None

        min_xyz = points_base.min(axis=0)
        max_xyz = points_base.max(axis=0)
        center = (min_xyz + max_xyz) / 2.0
        size = np.maximum(max_xyz - min_xyz, 1e-3)
        bbox = [
            float(center[0]),
            float(center[1]),
            float(center[2]),
            float(size[0]),
            float(size[1]),
            float(size[2]),
            0.0,
        ]

        occ = float(np.clip(1.0 - (cleaned.sum() / max(raw_area, 1)), 0.0, 1.0))
        center_dist = float(np.linalg.norm(center))
        if label != "person" and center_dist > float(self.max_non_person_distance_m):
            return None
        graspable = (occ <= self.occlusion_ratio_max_for_grasp) and (not is_stale)
        return DetectionCandidate(
            bbox=bbox,
            score=float(score),
            label=label,
            occlusion_ratio=occ,
            graspable=graspable,
        )

    def _compute_cleaned_mask(
        self,
        mask: np.ndarray,
        person_mask: np.ndarray,
    ) -> np.ndarray | None:
        raw_area = int(mask.sum())
        if raw_area < self.mask_area_min_px:
            return None

        cleaned = mask & (~person_mask)
        if cleaned.sum() < self.mask_area_min_px:
            return None
        cleaned = self._largest_connected_component(cleaned)
        if cleaned.sum() < self.mask_area_min_px:
            return None
        return cleaned

    def _publish_segmentation_debug(
        self,
        header,
        rgb: np.ndarray,
        person_mask: np.ndarray,
        raw_masks: list[np.ndarray],
        cleaned_masks: list[np.ndarray],
        accepted_count: int,
    ):
        overlay = rgb.copy()

        if person_mask.any():
            overlay[person_mask] = (0, 0, 255)

        for raw in raw_masks:
            if not raw.any():
                continue
            raw_u8 = raw.astype(np.uint8) * 255
            contours, _ = cv2.findContours(
                raw_u8,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )
            cv2.drawContours(overlay, contours, -1, (0, 255, 0), 1)

        for cleaned in cleaned_masks:
            if not cleaned.any():
                continue
            cleaned_u8 = cleaned.astype(np.uint8) * 255
            contours, _ = cv2.findContours(
                cleaned_u8,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )
            cv2.drawContours(overlay, contours, -1, (255, 255, 0), 2)

        blended = cv2.addWeighted(rgb, 0.55, overlay, 0.45, 0.0)

        debug_text = (
            f"raw={len(raw_masks)} accepted={accepted_count} "
            f"tf_age={self.last_tf_age_ms:.1f}ms skew={self.last_sync_skew_ms:.1f}ms"
        )
        cv2.putText(
            blended,
            debug_text,
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        msg = self.bridge.cv2_to_imgmsg(blended, encoding="bgr8")
        msg.header = header
        self.segmentation_debug_pub.publish(msg)

    @staticmethod
    def _compute_iou_3d(b1: list[float], b2: list[float]) -> float:
        c1 = np.asarray(b1[:3], dtype=np.float32)
        s1 = np.asarray(b1[3:6], dtype=np.float32)
        c2 = np.asarray(b2[:3], dtype=np.float32)
        s2 = np.asarray(b2[3:6], dtype=np.float32)
        min1, max1 = c1 - s1 / 2.0, c1 + s1 / 2.0
        min2, max2 = c2 - s2 / 2.0, c2 + s2 / 2.0
        inter_min = np.maximum(min1, min2)
        inter_max = np.minimum(max1, max2)
        inter_size = np.maximum(0.0, inter_max - inter_min)
        inter = float(np.prod(inter_size))
        v1 = float(np.prod(s1))
        v2 = float(np.prod(s2))
        union = v1 + v2 - inter
        if union <= 1e-9:
            return 0.0
        return max(0.0, min(1.0, inter / union))

    def _merge_detection_candidates(
        self,
        detections: list[DetectionCandidate],
    ) -> list[DetectionCandidate]:
        if len(detections) <= 1:
            return detections

        ordered = sorted(detections, key=lambda d: d.score, reverse=True)
        keep: list[DetectionCandidate] = []
        used = [False] * len(ordered)

        for i, base in enumerate(ordered):
            if used[i]:
                continue

            cluster = [base]
            used[i] = True
            base_center = np.asarray(base.bbox[:3], dtype=np.float32)

            for j in range(i + 1, len(ordered)):
                if used[j]:
                    continue
                other = ordered[j]
                center = np.asarray(other.bbox[:3], dtype=np.float32)
                dist = float(np.linalg.norm(base_center - center))
                iou = self._compute_iou_3d(base.bbox, other.bbox)
                if dist <= float(self.detection_merge_dist_m) or iou >= float(
                    self.detection_merge_iou_min
                ):
                    cluster.append(other)
                    used[j] = True

            if len(cluster) == 1:
                keep.append(base)
                continue

            weights = np.asarray([max(1e-3, d.score) for d in cluster], dtype=np.float32)
            weights /= float(weights.sum())
            centers = np.asarray([d.bbox[:3] for d in cluster], dtype=np.float32)
            sizes = np.asarray([d.bbox[3:6] for d in cluster], dtype=np.float32)
            merged_center = (centers * weights[:, None]).sum(axis=0)
            merged_size = np.max(sizes, axis=0)
            merged_score = float(max(d.score for d in cluster))
            merged_occ = float(min(d.occlusion_ratio for d in cluster))
            merged_graspable = any(d.graspable for d in cluster)
            merged_label = max(cluster, key=lambda d: d.score).label
            keep.append(
                DetectionCandidate(
                    bbox=[
                        float(merged_center[0]),
                        float(merged_center[1]),
                        float(merged_center[2]),
                        float(merged_size[0]),
                        float(merged_size[1]),
                        float(merged_size[2]),
                        0.0,
                    ],
                    score=merged_score,
                    label=merged_label,
                    occlusion_ratio=merged_occ,
                    graspable=merged_graspable,
                )
            )

        return keep

    @staticmethod
    def _largest_connected_component(mask: np.ndarray) -> np.ndarray:
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
            mask.astype(np.uint8), connectivity=8
        )
        if num_labels <= 1:
            return mask
        areas = stats[1:, cv2.CC_STAT_AREA]
        largest = int(np.argmax(areas)) + 1
        return labels == largest

    def _mask_to_points(
        self, depth_m: np.ndarray, mask: np.ndarray, info_msg: CameraInfo
    ) -> tuple[np.ndarray, float]:
        ys, xs = np.where(mask)
        if len(xs) == 0:
            return np.empty((0, 3), dtype=np.float32), 0.0

        z = depth_m[ys, xs]
        valid = np.isfinite(z) & (z > 0.0)
        valid_ratio = float(valid.sum() / max(len(z), 1))
        if valid.sum() == 0:
            return np.empty((0, 3), dtype=np.float32), valid_ratio

        fx = float(info_msg.k[0])
        fy = float(info_msg.k[4])
        cx = float(info_msg.k[2])
        cy = float(info_msg.k[5])

        u = xs[valid].astype(np.float32)
        v = ys[valid].astype(np.float32)
        z = z[valid].astype(np.float32)

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        points = np.stack([x, y, z], axis=1)
        return points, valid_ratio

    @staticmethod
    def _transform_points(points: np.ndarray, t_mat: np.ndarray) -> np.ndarray:
        points_h = np.hstack([points, np.ones((len(points), 1), dtype=np.float32)])
        transformed = (t_mat @ points_h.T).T
        return transformed[:, :3]

    def _filter_points(self, points: np.ndarray) -> np.ndarray:
        if len(points) < 10:
            return points
        z = points[:, 2]
        low = np.quantile(z, self.z_trim_quantile)
        high = np.quantile(z, 1.0 - self.z_trim_quantile)
        mask = (z >= low) & (z <= high)
        points = points[mask]
        if len(points) < 10:
            return points
        center = points.mean(axis=0)
        d = np.linalg.norm(points - center, axis=1)
        d_mean = float(np.mean(d))
        d_std = float(np.std(d))
        if d_std < 1e-6:
            return points
        return points[d < (d_mean + 2.5 * d_std)]

    def _maybe_reestimate_table(
        self, depth: np.ndarray, info_msg: CameraInfo, t_mat: np.ndarray
    ):
        now_ns = int(self.get_clock().now().nanoseconds)
        if now_ns - self.last_table_reestimate_ns < int(
            self.table_reestimate_interval_s * 1e9
        ):
            return
        self.last_table_reestimate_ns = now_ns

        valid = np.isfinite(depth) & (depth > 0)
        if valid.sum() < 500:
            return
        ys, xs = np.where(valid)
        z = depth[ys, xs].astype(np.float32)
        step = max(1, len(z) // 8000)
        ys = ys[::step]
        xs = xs[::step]
        z = z[::step]

        fx = float(info_msg.k[0])
        fy = float(info_msg.k[4])
        cx = float(info_msg.k[2])
        cy = float(info_msg.k[5])
        x = (xs.astype(np.float32) - cx) * z / fx
        y = (ys.astype(np.float32) - cy) * z / fy
        pts = np.stack([x, y, z], axis=1)
        pts = self._transform_points(pts, t_mat)
        if len(pts) < 100:
            return

        z_all = pts[:, 2]
        candidate = float(np.quantile(z_all, 0.06))
        inlier = np.abs(z_all - candidate) < 0.015
        inlier_ratio = float(inlier.mean())
        if inlier_ratio >= self.table_reestimate_min_inlier_ratio:
            self.table_z = candidate
            self._info(
                f"table_z 更新为 {self.table_z:.4f} (inlier_ratio={inlier_ratio:.2f})"
            )
        else:
            self._warn_throttled(
                f"table_z 重估失败，沿用旧值 {self.table_z:.4f} (inlier_ratio={inlier_ratio:.2f})"
                ,
                key="table_fail",
                period_frames=int(self.table_warn_every),
            )

    def _warn_throttled(self, msg: str, key: str, period_frames: int):
        count = self._warn_counters.get(key, 0) + 1
        self._warn_counters[key] = count
        if count % max(1, period_frames) == 1:
            self._warn(msg)

    def _publish_raw_detections(self, detections: list[DetectionCandidate], header):
        msg = Detection3DArray()
        msg.header = header
        msg.header.frame_id = self._current_output_frame
        for idx, det in enumerate(detections):
            d = Detection3D()
            d.id = str(idx)
            d.bbox.center.position = Point(
                x=det.bbox[0],
                y=det.bbox[1],
                z=det.bbox[2],
            )
            d.bbox.center.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            d.bbox.size = Vector3(x=det.bbox[3], y=det.bbox[4], z=det.bbox[5])
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = det.label
            hyp.hypothesis.score = det.score
            d.results.append(hyp)
            msg.detections.append(d)
        self.detections_pub.publish(msg)

    def _update_track_quality(
        self,
        tracked: list[dict],
        detections: list[DetectionCandidate],
        is_stale: bool,
    ):
        if detections:
            det_centers = np.array([d.bbox[:3] for d in detections], dtype=np.float32)
        else:
            det_centers = np.empty((0, 3), dtype=np.float32)

        for tr in tracked:
            tid = int(tr["track_id"])
            center = np.array(tr["bbox"][:3], dtype=np.float32)
            quality = {
                "occlusion_ratio": 1.0,
                "graspable": False,
                "is_stale": is_stale,
                "label": str(tr.get("label", "object")),
                "score": float(tr.get("score", 0.0)),
            }
            if len(det_centers) > 0:
                dists = np.linalg.norm(det_centers - center[None, :], axis=1)
                k = int(np.argmin(dists))
                if float(dists[k]) <= self.association_dist_gate_m * 1.5:
                    d = detections[k]
                    quality["occlusion_ratio"] = d.occlusion_ratio
                    quality["graspable"] = d.graspable
                    quality["label"] = d.label
                    quality["score"] = d.score
            self.track_quality[tid] = quality

        alive = {int(t["track_id"]) for t in tracked}
        stale_keys = [k for k in self.track_quality if k not in alive]
        for k in stale_keys:
            del self.track_quality[k]

    def _publish_tracks(self, tracked: list[dict], header, is_stale: bool):
        vision_msg = Detection3DArray()
        vision_msg.header = header
        vision_msg.header.frame_id = self._current_output_frame

        custom_msg = TrackedObject3DArray()
        custom_msg.header = header
        custom_msg.header.frame_id = self._current_output_frame

        for tr in tracked:
            tid = int(tr["track_id"])
            bbox = tr["bbox"]
            q = self.track_quality.get(
                tid,
                {
                    "occlusion_ratio": 1.0,
                    "graspable": False,
                    "is_stale": is_stale,
                    "label": str(tr.get("label", "object")),
                    "score": float(tr.get("score", 0.0)),
                },
            )

            det = Detection3D()
            det.id = str(tid)
            det.bbox.center.position = Point(x=bbox[0], y=bbox[1], z=bbox[2])
            det.bbox.center.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            det.bbox.size = Vector3(x=bbox[3], y=bbox[4], z=bbox[5])
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(q["label"])
            hyp.hypothesis.score = float(q["score"])
            det.results.append(hyp)
            vision_msg.detections.append(det)

            obj = TrackedObject3D()
            obj.tracking_id = tid
            obj.center = Point(x=bbox[0], y=bbox[1], z=bbox[2])
            obj.size = Vector3(x=bbox[3], y=bbox[4], z=bbox[5])
            obj.yaw = float(bbox[6])
            obj.score = float(q["score"])
            obj.occlusion_ratio = float(q["occlusion_ratio"])
            obj.is_stale = bool(q["is_stale"])
            obj.graspable = bool(q["graspable"])
            obj.label = str(q["label"])
            custom_msg.objects.append(obj)

        self.tracked_pub.publish(vision_msg)
        self.tracked_custom_pub.publish(custom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RgbdDetectTrackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._info("收到中断信号")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
