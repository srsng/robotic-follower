#!/usr/bin/env python3
"""目标选择器节点

订阅跟踪目标，发布选中的目标 track_id

订阅话题：
    - /perception/tracked_objects (vision_msgs/Detection3DArray)
        跟踪目标列表

发布话题：
    - /perception/selected_target (std_msgs/Int32)
        选中的目标 track_id

使用示例：
    ros2 run robotic_follower target_selector_node
"""

import rclpy
from std_msgs.msg import Int32
from vision_msgs.msg import Detection3DArray

from robotic_follower.util.ui.target_selector import TargetSelectorUI
from robotic_follower.util.wrapper import NodeWrapper


class TrackSelectorNode(NodeWrapper):
    """目标选择器节点"""

    def __init__(self):
        super().__init__("target_selector_node")

        # 参数
        tracking_topic = self.declare_and_get_parameter(
            "tracking_topic", "/perception/tracked_objects"
        )

        # 发布选中目标
        self.selected_pub = self.create_publisher(
            Int32, "/perception/selected_target", 10
        )

        # 订阅跟踪目标
        self.tracked_sub = self.create_subscription(
            Detection3DArray,
            tracking_topic,
            self._tracked_callback,
            10,
        )

        # 当前跟踪目标列表
        self._tracked_objects: list[dict] = []
        self._selected_track_id: int | None = None

        # 检查 DISPLAY 环境变量
        import os

        if not os.environ.get("DISPLAY"):
            self._error("$DISPLAY 环境变量未设置，无法启动 GUI")
            raise RuntimeError("$DISPLAY 环境变量未设置")

        # 创建 UI
        self._ui = TargetSelectorUI(
            on_select=self._on_target_selected, parent_node=self
        )

        self._info("目标选择器节点已启动")

    def _tracked_callback(self, msg: Detection3DArray):
        """接收跟踪目标回调"""
        self._tracked_objects = []

        for det in msg.detections:
            bbox = det.bbox
            x, y, z = (
                bbox.center.position.x,
                bbox.center.position.y,
                bbox.center.position.z,
            )

            # 提取 track_id
            track_id = None
            label = "unknown"
            if det.results:
                try:
                    track_id = int(det.results[0].hypothesis.class_id)
                    label = str(det.results[0].hypothesis.class_id)
                except (ValueError, IndexError):
                    pass

            self._tracked_objects.append(
                {
                    "track_id": track_id,
                    "label": label,
                    "position": (x, y, z),
                }
            )

        # 按 track_id 排序
        self._tracked_objects.sort(key=lambda t: t["track_id"] or 0)

        # 更新 UI
        self._ui.update_targets(self._tracked_objects)

    def _on_target_selected(self, track_id: int):
        """目标被选中回调"""
        msg = Int32()
        msg.data = track_id
        self.selected_pub.publish(msg)

        if track_id > 0:
            self._selected_track_id = track_id
            self._info(f"已选择目标: track_id={track_id}")
        else:
            self._selected_track_id = None
            self._info("已取消跟随")

        self._ui.set_selected(self._selected_track_id)

    def spin(self):
        """运行节点"""
        while rclpy.ok() and self._ui.running:
            self._ui.update()
            rclpy.spin_once(self, timeout_sec=0.01)

        self._ui.destroy()


def main(args=None):
    rclpy.init(args=args)
    node = TrackSelectorNode()
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
