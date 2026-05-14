#!/usr/bin/env python3
"""融合 RGBD 检测+追踪节点。"""

from __future__ import annotations

import os
import time
from pathlib import Path

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

from robotic_follower.detection.data import DetectionCandidate
from robotic_follower.detection.inference.seg_projection import SegProjectionDetector
from robotic_follower.tracking.kalman_tracker_3d import KalmanTracker3D
from robotic_follower.util.perf import PerfTimer
from robotic_follower.util.wrapper import NodeWrapper


class RgbdDetectTrackNode(NodeWrapper):
    """融合节点：分割、2.5D 投影、AABB、3D 追踪。"""

    def __init__(self):
        super().__init__("rgbd_detect_track_node")

        self._param_defaults: dict[str, object] = {
            "target_frame": "base_link",
            "tf_stale_threshold_ms": 80.0,
            "sync_skew_threshold_ms": 30.0,
            "tf_fallback_warn_every": 45,
            "sync_warn_every": 45,
            "fallback_to_source_frame_when_tf_disconnected": True,
        }

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.target_frame = self.declare_and_get_parameter("target_frame", "base_link")
        self.base_frame = self.target_frame

        self.tf_stale_threshold_ms = self.declare_and_get_parameter(
            "tf_stale_threshold_ms", 80.0
        )
        self.sync_skew_threshold_ms = self.declare_and_get_parameter(
            "sync_skew_threshold_ms", 30.0
        )
        self.tf_fallback_warn_every = self.declare_and_get_parameter(
            "tf_fallback_warn_every", 45
        )
        self.sync_warn_every = self.declare_and_get_parameter("sync_warn_every", 45)
        self.fallback_to_source_frame_when_tf_disconnected = bool(
            self.declare_and_get_parameter(
                "fallback_to_source_frame_when_tf_disconnected", True
            )
        )
        self.tf_lookup_fail_count = 0
        self.last_tf_age_ms = 0.0
        self.last_sync_skew_ms = 0.0
        self.last_stamp_ns: int | None = None
        self._prev_stamp_ns: int | None = None
        self._warn_counters: dict[str, int] = {}
        self._current_output_frame = self.base_frame
        self._perf_frame_id = 0
        self.perf_aggregate_interval = int(
            self.declare_and_get_parameter("perf_aggregate_interval", 100)
        )
        self._perf_timer = PerfTimer(
            lambda level, msg, channel: self._log(level, msg, channel=channel),
            channel="perf",
            stats_channel="perf_stats",
            aggregate_interval=self.perf_aggregate_interval,
        )

        config_file = self.declare_and_get_parameter(
            "config_file", "model/config/yolov8_seg_rgbd_track.yaml"
        )
        config = self._load_config(config_file)
        if not isinstance(config, dict):
            config = {}

        detector_cfg = config.get("detector", {})
        tracker_cfg = config.get("tracker", {})
        node_cfg = config.get("node", {})

        node_params = node_cfg.get("params", {}) if isinstance(node_cfg, dict) else {}

        self._apply_config_params(node_params)

        detector_cfg = detector_cfg if isinstance(detector_cfg, dict) else {}
        detector_cfg = detector_cfg.copy()
        detector_cfg.setdefault("type", "seg_projection")
        detector_cfg.setdefault("segmenter", {"type": "yolov8_seg"})
        self.detector = SegProjectionDetector.create_from_config(
            detector_cfg, parent_node=self
        )

        tracker_type = (
            tracker_cfg.get("type", "kalman3d")
            if isinstance(tracker_cfg, dict)
            else "kalman3d"
        )
        if tracker_type != "kalman3d":
            self._warn(f"未知 tracker type: {tracker_type}, 回退到 kalman3d")
        tracker_params = tracker_cfg.get("params", {}) if isinstance(tracker_cfg, dict) else {}
        self.association_dist_gate_m = float(
            tracker_params.get("association_dist_gate_m", 0.50)
        )
        self.tracker = KalmanTracker3D(
            dist_gate_m=self.association_dist_gate_m,
            max_age=int(tracker_params.get("max_age", 30)),
            min_hits=int(tracker_params.get("min_hits", 1)),
            duplicate_track_dist_m=float(tracker_params.get("duplicate_track_dist_m", 0.15)),
        )
        self.table_z: float = float(node_params.get("table_z", 0.0))
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
        self.segmentation_debug_vis_interval = int(
            self.declare_and_get_parameter("segmentation_debug_vis_interval", 1)
        )
        self._debug_vis_counter = 0

        self._info("融合检测追踪节点已启动")

    def _apply_config_params(self, cfg_params: dict):
        """将配置文件中的 params 应用于运行参数。

        优先级：显式 ROS 参数覆盖 > 配置文件 > 代码默认值。
        """
        if not cfg_params:
            return

        changed: list[str] = []
        for key, cfg_value in cfg_params.items():
            if not hasattr(self, key):
                continue
            if key not in self._param_defaults:
                continue

            current_value = getattr(self, key)
            default_value = self._param_defaults[key]

            if current_value != default_value:
                continue

            setattr(self, key, cfg_value)
            changed.append(f"{key}={cfg_value}")

        if changed:
            self._info("应用配置文件参数: " + ", ".join(changed))

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
        t0 = time.monotonic()
        perf = self._perf_timer.frame()
        perf.mark("start")
        self._perf_frame_id += 1
        try:
            current_stamp_ns = self._stamp_to_ns(rgb_msg.header.stamp)
            if self._prev_stamp_ns is not None:
                dt = max((current_stamp_ns - self._prev_stamp_ns) / 1e9, 0.001)
            else:
                dt = 0.033
            self._prev_stamp_ns = current_stamp_ns
            perf.record_value("dt", dt)

            t_sync_start = perf.mark("sync_start")
            self._update_sync_skew(rgb_msg, depth_msg)
            perf.record("sync_update", t_sync_start)

            t_tf_start = perf.mark("tf_lookup_start")
            tf_data = self._lookup_transform(
                depth_msg.header.frame_id, rgb_msg.header.stamp
            )
            perf.record("tf_lookup", t_tf_start)
            if tf_data is None:
                t_track_start = perf.mark("track_update_start")
                tracked = self.tracker.update([], dt=dt)
                perf.record("track_update", t_track_start)
                t_pub_tracks_start = perf.mark("publish_tracks_start")
                self._publish_tracks(tracked, rgb_msg.header, is_stale=True)
                perf.record("publish_tracks", t_pub_tracks_start)
                perf.record("total", "start")
                perf.flush(
                    extra={
                        "frame_id": self._perf_frame_id,
                        "n_raw": 0,
                        "n_accepted": 0,
                        "n_tracked": len(tracked),
                        "tf_ok": False,
                    }
                )
                return
            t_mat, is_stale, output_frame = tf_data
            self._current_output_frame = output_frame
            perf.record_value("tf_ok", True)

            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            t_depth_start = perf.mark("depth_convert_start")
            depth = self._depth_to_meters(depth_msg)
            perf.record("depth_convert", t_depth_start)
            if rgb.shape[:2] != depth.shape[:2]:
                self._warn("RGB 和 Depth 分辨率不一致，跳过本帧")
                perf.record("total", "start")
                perf.flush(
                    extra={
                        "frame_id": self._perf_frame_id,
                        "n_raw": 0,
                        "n_accepted": 0,
                        "n_tracked": 0,
                        "shape_mismatch": True,
                    }
                )
                return

            if self.detector is None or not self.detector.ready:
                self._warn("seg_projection detector 未就绪，跳过本帧")
                return

            t_seg_start = time.monotonic()
            t_preprocess_start = time.monotonic()
            t_mask_proc_start = perf.mark("mask_proc_start")
            collect_debug = self.enable_segmentation_debug_vis
            if collect_debug:
                self._debug_vis_counter += 1
                do_debug = (
                    self._debug_vis_counter
                    % max(1, self.segmentation_debug_vis_interval)
                    == 0
                )
            else:
                do_debug = False
            debug_text = (
                f"tf_age={self.last_tf_age_ms:.1f}ms "
                f"skew={self.last_sync_skew_ms:.1f}ms"
            )
            det_result = self.detector.detect_rgbd(
                rgb=rgb,
                depth_m=depth,
                camera_k=(
                    float(info_msg.k[0]),
                    float(info_msg.k[4]),
                    float(info_msg.k[2]),
                    float(info_msg.k[5]),
                ),
                t_mat=t_mat,
                is_stale=is_stale,
                debug=do_debug,
                debug_text=debug_text,
                now_ns=current_stamp_ns,
            )
            perf.record("mask_proc", t_mask_proc_start)

            t_seg = time.monotonic() - t_seg_start
            perf.record_value("segmentation", t_seg)

            detections: list[DetectionCandidate] = det_result.detections

            t_preprocess = time.monotonic() - t_preprocess_start
            perf.record_value("preprocess", t_preprocess)

            self._log(
                "debug",
                f"t_seg={t_seg:.6f} t_preprocess={t_preprocess:.6f} n_raw={det_result.raw_count} n_accepted={len(detections)}",
                channel="detection",
            )

            if do_debug and det_result.debug_overlay is not None:
                t_debug_vis_start = perf.mark("debug_vis_start")
                self._publish_segmentation_debug(
                    header=rgb_msg.header,
                    overlay=det_result.debug_overlay,
                )
                perf.record("debug_vis", t_debug_vis_start)

            t_pub_det_start = perf.mark("publish_detections_start")
            self._publish_raw_detections(detections, rgb_msg.header)
            perf.record("publish_detections", t_pub_det_start)

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
            t_track_update_start = perf.mark("track_update_start")
            tracked = self.tracker.update(det_dicts, dt=dt)
            perf.record("track_update", t_track_update_start)
            t_quality_update_start = perf.mark("quality_update_start")
            self._update_track_quality(tracked, detections, is_stale)
            perf.record("quality_update", t_quality_update_start)
            t_track_total = time.monotonic() - t0
            perf.record_value("total", t_track_total)
            t_pub_tracks_start = perf.mark("publish_tracks_start")
            self._publish_tracks(tracked, rgb_msg.header, is_stale=is_stale)
            perf.record("publish_tracks", t_pub_tracks_start)

            t_table_start = perf.mark("table_est_start")
            self._update_table_from_pipeline(det_result)
            perf.record("table_est", t_table_start)
            perf.flush(
                extra={
                    "frame_id": self._perf_frame_id,
                    "n_raw": det_result.raw_count,
                    "n_accepted": len(detections),
                    "n_tracked": len(tracked),
                    "tf_age_ms": self.last_tf_age_ms,
                    "sync_skew_ms": self.last_sync_skew_ms,
                }
            )
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

    def _publish_segmentation_debug(self, header, overlay: np.ndarray):
        msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        msg.header = header
        self.segmentation_debug_pub.publish(msg)

    def _update_table_from_pipeline(self, det_result):
        table_z = getattr(det_result, "table_z", None)
        if table_z is not None:
            self.table_z = float(table_z)

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
                if float(dists[k]) <= self.association_dist_gate_m * 2.0:
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
