#!/usr/bin/env python3
"""Unified configurable detection and tracking node."""

from __future__ import annotations

import json
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
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from robotic_follower.perception import (
    PerceptionFrame,
    PerceptionPipeline,
    create_perception_pipeline_from_config,
)
from robotic_follower.util.perf import PerfTimer
from robotic_follower.util.ros_pointcloud import (
    numpy_to_pointcloud2,
    pointcloud2_to_numpy,
)
from robotic_follower.util.wrapper import NodeWrapper
from robotic_follower_msgs.msg import TrackedObject3D, TrackedObject3DArray


class DetectTrackNode(NodeWrapper):
    """Unified perception node driven by a detector/tracker YAML pipeline."""

    def __init__(self):
        super().__init__("detect_track_node")

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.input_mode = self.declare_and_get_parameter("input_mode", "rgbd")
        self.target_frame = self.declare_and_get_parameter("target_frame", "base_link")
        self.base_frame = self.target_frame
        self.tf_stale_threshold_ms = float(
            self.declare_and_get_parameter("tf_stale_threshold_ms", 80.0)
        )
        self.sync_skew_threshold_ms = float(
            self.declare_and_get_parameter("sync_skew_threshold_ms", 30.0)
        )
        self.tf_fallback_warn_every = int(
            self.declare_and_get_parameter("tf_fallback_warn_every", 45)
        )
        self.sync_warn_every = int(
            self.declare_and_get_parameter("sync_warn_every", 45)
        )
        self.fallback_to_source_frame_when_tf_disconnected = bool(
            self.declare_and_get_parameter(
                "fallback_to_source_frame_when_tf_disconnected",
                True,
            )
        )
        self._warn_counters: dict[str, int] = {}
        self._current_output_frame = self.base_frame
        self.last_tf_age_ms = 0.0
        self.last_sync_skew_ms = 0.0
        self._prev_stamp_ns: int | None = None
        self._perf_frame_id = 0
        self._published_class_names = False
        self._last_table_z = 0.0
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
            "config_file",
            "model/config/yolov8_seg_rgbd_track.yaml",
        )
        self.pipeline = self._load_pipeline(config_file)

        self._init_publishers()
        self._init_subscribers()
        self._info(
            f"统一检测追踪节点已启动: input_mode={self.input_mode}, pipeline={self.pipeline.name}"
        )

    def _load_pipeline(self, config_file: str) -> PerceptionPipeline:
        config = self._load_config(config_file)
        if not isinstance(config, dict):
            config = {}
        return create_perception_pipeline_from_config(config, parent_node=self)

    def _load_config(self, config_file: str) -> dict:
        expanded = os.path.expanduser(config_file)
        tried_paths: list[str] = []
        if not os.path.isabs(expanded):
            from ament_index_python.packages import get_package_share_directory

            pkg_path = get_package_share_directory("robotic_follower")
            expanded = os.path.join(pkg_path, expanded)
            tried_paths.append(expanded)
            rel = config_file
            for parent in Path(__file__).resolve().parents:
                src_candidate = parent / "src" / "robotic_follower" / rel
                if src_candidate.exists():
                    expanded = str(src_candidate)
                    break
        if not os.path.exists(expanded):
            msg = f"配置文件不存在: {expanded}"
            if tried_paths:
                msg += f"; 已尝试: {tried_paths}"
            raise FileNotFoundError(msg)
        self._info(f"加载感知配置文件: {expanded}")
        with open(expanded, encoding="utf-8") as f:
            return yaml.safe_load(f) or {}

    def _init_publishers(self):
        self.detections_pub = self.create_publisher(
            Detection3DArray, "/perception/detections", 10
        )
        self.tracked_pub = self.create_publisher(
            Detection3DArray, "/perception/tracked_objects", 10
        )
        self.tracked_custom_pub = self.create_publisher(
            TrackedObject3DArray, "/perception/tracked_objects_custom", 10
        )
        self.class_names_pub = self.create_publisher(
            String, "/perception/class_names_info", 10
        )
        self.transformed_pc_pub = self.create_publisher(
            PointCloud2, "/perception/pointcloud_transformed", 10
        )
        self.enable_segmentation_debug_vis = bool(
            self.declare_and_get_parameter("enable_segmentation_debug_vis", True)
        )
        self.segmentation_debug_topic = self.declare_and_get_parameter(
            "segmentation_debug_topic",
            "/perception/segmentation_debug",
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

    def _init_subscribers(self):
        if self.input_mode == "rgbd":
            rgb_topic = self.declare_and_get_parameter(
                "rgb_topic",
                "/camera/camera/color/image_raw",
            )
            depth_topic = self.declare_and_get_parameter(
                "depth_topic",
                "/camera/camera/aligned_depth_to_color/image_raw",
            )
            camera_info_topic = self.declare_and_get_parameter(
                "camera_info_topic",
                "/camera/camera/color/camera_info",
            )
            self.rgb_sub = message_filters.Subscriber(self, Image, rgb_topic)
            self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
            self.info_sub = message_filters.Subscriber(self, CameraInfo, camera_info_topic)
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [self.rgb_sub, self.depth_sub, self.info_sub],
                queue_size=10,
                slop=0.03,
            )
            self.sync.registerCallback(self.rgbd_callback)
            return

        if self.input_mode == "pointcloud":
            pointcloud_topic = self.declare_and_get_parameter(
                "pointcloud_topic",
                "/camera/camera/depth/color/points",
            )
            self.pointcloud_sub = self.create_subscription(
                PointCloud2,
                pointcloud_topic,
                self.pointcloud_callback,
                10,
            )
            return

        raise ValueError(f"Unsupported input_mode: {self.input_mode}")

    def rgbd_callback(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        t0 = time.monotonic()
        perf = self._perf_timer.frame()
        perf.mark("start")
        self._perf_frame_id += 1
        try:
            current_stamp_ns = self._stamp_to_ns(rgb_msg.header.stamp)
            dt = self._compute_dt(current_stamp_ns)
            perf.record_value("dt", dt)

            t_sync_start = perf.mark("sync_start")
            self._update_sync_skew(rgb_msg, depth_msg)
            perf.record("sync_update", t_sync_start)

            t_tf_start = perf.mark("tf_lookup_start")
            tf_data = self._lookup_transform(
                depth_msg.header.frame_id,
                rgb_msg.header.stamp,
            )
            perf.record("tf_lookup", t_tf_start)
            if tf_data is None:
                result = self.pipeline.process(
                    PerceptionFrame(input_mode="rgbd", dt=dt, is_stale=True)
                )
                self._publish_tracks(result.tracks, rgb_msg.header, is_stale=True)
                return
            t_mat, is_stale, output_frame = tf_data
            self._current_output_frame = output_frame

            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            t_depth_start = perf.mark("depth_convert_start")
            depth = self._depth_to_meters(depth_msg)
            perf.record("depth_convert", t_depth_start)
            if rgb.shape[:2] != depth.shape[:2]:
                self._warn("RGB 和 Depth 分辨率不一致，跳过本帧")
                return

            debug = self._should_publish_debug_overlay()
            debug_text = (
                f"tf_age={self.last_tf_age_ms:.1f}ms "
                f"skew={self.last_sync_skew_ms:.1f}ms"
            )
            t_pipeline_start = perf.mark("pipeline_start")
            result = self.pipeline.process(
                PerceptionFrame(
                    input_mode="rgbd",
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
                    dt=dt,
                    now_ns=current_stamp_ns,
                    debug=debug,
                    debug_text=debug_text,
                )
            )
            perf.record("pipeline", t_pipeline_start)
            self._last_table_z = float(result.metadata.get("table_z", self._last_table_z))

            self._publish_raw_detections(result.detections, rgb_msg.header)
            self._publish_tracks(result.tracks, rgb_msg.header, is_stale=is_stale)
            if debug and result.debug_overlay is not None:
                self._publish_segmentation_debug(rgb_msg.header, result.debug_overlay)
            perf.record_value("total", time.monotonic() - t0)
            perf.flush(
                extra={
                    "frame_id": self._perf_frame_id,
                    "n_raw": result.raw_count,
                    "n_accepted": len(result.detections),
                    "n_tracked": len(result.tracks),
                    "tf_age_ms": self.last_tf_age_ms,
                    "sync_skew_ms": self.last_sync_skew_ms,
                }
            )
        except Exception as exc:
            self._error(f"RGBD 感知处理失败: {exc}")

    def pointcloud_callback(self, msg: PointCloud2):
        t0 = time.monotonic()
        try:
            if not self._published_class_names:
                self._publish_class_names_info()
                self._published_class_names = True

            transformed_cloud = self._transform_pointcloud(msg)
            if transformed_cloud is None:
                return
            self.transformed_pc_pub.publish(transformed_cloud)

            points_full = pointcloud2_to_numpy(transformed_cloud)
            if len(points_full) > 20000:
                indices = np.random.choice(len(points_full), 20000, replace=False)
                points_full = points_full[indices]
            points_xyz = points_full[:, :3] if points_full.shape[1] > 3 else points_full
            if len(points_xyz) < 100:
                self._warn("点云点数过少，跳过检测")
                return

            current_stamp_ns = self._stamp_to_ns(msg.header.stamp)
            dt = self._compute_dt(current_stamp_ns)
            result = self.pipeline.process(
                PerceptionFrame(
                    input_mode="pointcloud",
                    points=points_xyz,
                    dt=dt,
                    now_ns=current_stamp_ns,
                )
            )
            self._current_output_frame = transformed_cloud.header.frame_id
            self._publish_raw_detections(result.detections, transformed_cloud.header)
            self._publish_tracks(result.tracks, transformed_cloud.header, is_stale=False)
            self._log(
                "debug",
                f"t_total={time.monotonic() - t0:.6f} n_pts={len(points_xyz)} "
                f"n_det={len(result.detections)} n_tracked={len(result.tracks)}",
                channel="perception",
            )
        except Exception as exc:
            self._error(f"点云感知处理失败: {exc}")

    def _publish_class_names_info(self):
        detector = self.pipeline.detector
        info = {
            "class_names": list(detector.class_names),
            "idx2class_name": {
                str(k): v for k, v in detector.idx2class_name.items()
            },
            "class_name2idx": detector.class_name2idx,
            "ignore_class_idx": list(detector.ignore_class_idx),
            "ignore_class_names": list(detector.ignore_class_names),
        }
        msg = String()
        msg.data = json.dumps(info)
        self.class_names_pub.publish(msg)

    def _compute_dt(self, current_stamp_ns: int) -> float:
        if self._prev_stamp_ns is None:
            self._prev_stamp_ns = current_stamp_ns
            return 0.033
        dt = max((current_stamp_ns - self._prev_stamp_ns) / 1e9, 0.001)
        self._prev_stamp_ns = current_stamp_ns
        return dt

    def _should_publish_debug_overlay(self) -> bool:
        if not self.enable_segmentation_debug_vis:
            return False
        self._debug_vis_counter += 1
        return (
            self._debug_vis_counter
            % max(1, self.segmentation_debug_vis_interval)
            == 0
        )

    def _transform_pointcloud(self, cloud_msg: PointCloud2) -> PointCloud2 | None:
        source_frame = cloud_msg.header.frame_id
        if source_frame == self.target_frame:
            return cloud_msg
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                rclpy.time.Time(seconds=0),
                timeout=Duration(seconds=1.0),
            )
        except Exception as exc:
            self._warn(f"TF 变换查询失败: {exc}")
            return None

        t_mat = self._transform_to_matrix(transform)
        points = pointcloud2_to_numpy(cloud_msg)
        points_xyz = points[:, :3]
        extra_fields = points[:, 3:] if points.shape[1] > 3 else None
        points_hom = np.hstack([points_xyz, np.ones((points_xyz.shape[0], 1))])
        transformed_xyz = (t_mat @ points_hom.T).T[:, :3]
        transformed = (
            np.hstack([transformed_xyz, extra_fields])
            if extra_fields is not None
            else transformed_xyz
        )
        return numpy_to_pointcloud2(
            transformed,
            frame_id=self.target_frame,
            stamp=cloud_msg.header.stamp,
            pack_rgb=False,
        )

    def _lookup_transform(
        self,
        source_frame: str,
        stamp_msg,
    ) -> tuple[np.ndarray, bool, str] | None:
        try:
            stamp = rclpy.time.Time.from_msg(stamp_msg)
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                source_frame,
                stamp,
                timeout=Duration(seconds=0.05),
            )
            mat = self._transform_to_matrix(transform)
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
                    period_frames=self.tf_fallback_warn_every,
                )
                self.last_tf_age_ms = self.tf_stale_threshold_ms + 1.0
                return np.eye(4, dtype=np.float32), True, source_frame
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    source_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.05),
                )
                self.last_tf_age_ms = self.tf_stale_threshold_ms + 1.0
                self._warn_throttled(
                    f"TF 时间戳对齐失败，使用最新TF回退: {exc}",
                    key="tf_fallback",
                    period_frames=self.tf_fallback_warn_every,
                )
                return self._transform_to_matrix(transform), True, self.base_frame
            except Exception:
                pass
            self._warn_throttled(
                f"TF 查询失败: {exc}",
                key="tf_fail",
                period_frames=self.tf_fallback_warn_every,
            )
            return None

    @staticmethod
    def _transform_to_matrix(transform) -> np.ndarray:
        q = transform.transform.rotation
        t = transform.transform.translation
        mat = np.eye(4, dtype=np.float32)
        mat[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        mat[:3, 3] = [t.x, t.y, t.z]
        return mat

    def _update_sync_skew(self, rgb_msg: Image, depth_msg: Image):
        rgb_ns = self._stamp_to_ns(rgb_msg.header.stamp)
        depth_ns = self._stamp_to_ns(depth_msg.header.stamp)
        self.last_sync_skew_ms = abs(rgb_ns - depth_ns) / 1e6
        if self.last_sync_skew_ms > self.sync_skew_threshold_ms:
            self._warn_throttled(
                f"sync_skew 过大: {self.last_sync_skew_ms:.1f} ms",
                key="sync_skew",
                period_frames=self.sync_warn_every,
            )

    def _warn_throttled(self, msg: str, key: str, period_frames: int):
        count = self._warn_counters.get(key, 0) + 1
        self._warn_counters[key] = count
        if count % max(1, period_frames) == 1:
            self._warn(msg)

    @staticmethod
    def _stamp_to_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _depth_to_meters(self, depth_msg: Image) -> np.ndarray:
        if depth_msg.encoding == "16UC1":
            depth_mm = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="16UC1")
            return depth_mm.astype(np.float32) * 0.001
        if depth_msg.encoding == "32FC1":
            return self.bridge.imgmsg_to_cv2(
                depth_msg,
                desired_encoding="32FC1",
            ).astype(np.float32)
        depth = self.bridge.imgmsg_to_cv2(depth_msg)
        return depth.astype(np.float32)

    def _publish_segmentation_debug(self, header, overlay: np.ndarray):
        msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        msg.header = header
        self.segmentation_debug_pub.publish(msg)

    def _publish_raw_detections(self, detections: list[dict], header):
        msg = Detection3DArray()
        msg.header = header
        msg.header.frame_id = self._current_output_frame
        for idx, det in enumerate(detections):
            d = self._dict_to_detection_msg(det, str(idx))
            msg.detections.append(d)
        self.detections_pub.publish(msg)

    def _publish_tracks(self, tracks: list[dict], header, is_stale: bool):
        vision_msg = Detection3DArray()
        vision_msg.header = header
        vision_msg.header.frame_id = self._current_output_frame

        custom_msg = TrackedObject3DArray()
        custom_msg.header = header
        custom_msg.header.frame_id = self._current_output_frame

        for track in tracks:
            tid = int(track["track_id"])
            q = self.pipeline.track_quality_for(track, is_stale)
            det = self._dict_to_detection_msg(
                {
                    "bbox": track["bbox"],
                    "label": q["label"],
                    "score": q["score"],
                },
                str(tid),
            )
            vision_msg.detections.append(det)

            bbox = track["bbox"]
            obj = TrackedObject3D()
            obj.tracking_id = tid
            obj.center = Point(x=float(bbox[0]), y=float(bbox[1]), z=float(bbox[2]))
            obj.size = Vector3(x=float(bbox[3]), y=float(bbox[4]), z=float(bbox[5]))
            obj.yaw = float(bbox[6])
            obj.score = float(q["score"])
            obj.occlusion_ratio = float(q["occlusion_ratio"])
            obj.is_stale = bool(q["is_stale"])
            obj.graspable = bool(q["graspable"])
            obj.label = str(q["label"])
            custom_msg.objects.append(obj)

        self.tracked_pub.publish(vision_msg)
        self.tracked_custom_pub.publish(custom_msg)

    def _dict_to_detection_msg(self, det: dict, det_id: str) -> Detection3D:
        bbox = det["bbox"]
        msg = Detection3D()
        msg.id = det_id
        msg.bbox.center.position = Point(
            x=float(bbox[0]),
            y=float(bbox[1]),
            z=float(bbox[2]),
        )
        msg.bbox.center.orientation = self._yaw_to_quaternion(float(bbox[6]))
        msg.bbox.size = Vector3(
            x=float(bbox[3]),
            y=float(bbox[4]),
            z=float(bbox[5]),
        )
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = str(det.get("label", det.get("name", "object")))
        hyp.hypothesis.score = float(det.get("score", 1.0))
        msg.results.append(hyp)
        return msg

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        import math

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return Quaternion(x=0.0, y=0.0, z=qz, w=qw)


def main(args=None):
    rclpy.init(args=args)
    node = DetectTrackNode()
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
