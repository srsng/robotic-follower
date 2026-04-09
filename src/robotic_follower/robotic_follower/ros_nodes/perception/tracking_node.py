"""3D 多目标追踪节点。

该节点接收 3D 检测结果，执行多目标追踪并发布跟踪轨迹。

功能描述：
    - 订阅 3D 检测结果
    - 使用 IOU 匹配关联检测和跟踪轨迹
    - 使用匀速模型进行状态估计
    - 维护目标生命周期（创建、删除）
    - 发布跟踪轨迹

订阅话题：
    - /perception/detections (vision_msgs/Detection3DArray)
        3D 目标检测结果

发布话题：
    - /perception/tracked_objects (vision_msgs/Detection3DArray)
        带跟踪 ID 的 3D 目标跟踪结果

参数：
    - iou_threshold (float, 默认 0.3)
        IOU 匹配阈值
    - max_age (int, 默认 30)
        最大丢失帧数，超过则删除轨迹
    - min_hits (int, 默认 3)
        最小命中帧数，才认为轨迹有效
    - input_topic (string, 默认 "/perception/detections")
        检测结果输入话题
    - output_topic (string, 默认 "/perception/tracked_objects")
        跟踪结果发布话题

使用示例：
    ros2 run robotic_follower tracking_node
    ros2 run robotic_follower tracking_node --ros-args -p iou_threshold:=0.5
"""

import rclpy
from geometry_msgs.msg import Point, Quaternion, Vector3
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from robotic_follower.tracking.tracker_3d import Tracker3D
from robotic_follower.util.wrapper import NodeWrapper


class TrackingNode(NodeWrapper):
    """3D 多目标追踪节点。"""

    def __init__(self):
        super().__init__("tracking_node")

        # 参数
        self.declare_parameter("iou_threshold", 0.3)
        self.declare_parameter("max_age", 30)
        self.declare_parameter("min_hits", 3)
        self.declare_parameter("input_topic", "/perception/detections")
        self.declare_parameter("output_topic", "/perception/tracked_objects")

        iou_threshold = self.get_parameter("iou_threshold").value
        max_age = self.get_parameter("max_age").value
        min_hits = self.get_parameter("min_hits").value
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        # 初始化追踪器
        self.tracker = Tracker3D(
            iou_threshold=iou_threshold,
            max_age=max_age,
            min_hits=min_hits,
            parent_node=self,
        )

        # 订阅检测话题
        self.detection_sub = self.create_subscription(
            Detection3DArray,
            input_topic,
            self.detection_callback,
            10,
        )

        # 发布跟踪话题
        self.tracked_pub = self.create_publisher(Detection3DArray, output_topic, 10)

        self._info("3D 追踪节点已启动")

    def detection_callback(self, msg: Detection3DArray):
        """检测回调。"""
        self._debug(f"收到 {len(msg.detections)} 个检测")

        # 提取检测信息
        detections = []
        for det in msg.detections:
            bbox = det.bbox
            # bbox.center.position: Point (x, y, z)
            # bbox.center.orientation: Quaternion
            # bbox.size: Vector3 (x, y, z)
            x = bbox.center.position.x
            y = bbox.center.position.y
            z = bbox.center.position.z
            dx = bbox.size.x
            dy = bbox.size.y
            dz = bbox.size.z

            # 从四元数提取 yaw
            q = bbox.center.orientation
            yaw = self._quaternion_to_yaw(q)

            # 提取类别和置信度
            if det.results:
                label = det.results[0].hypothesis.class_id
                score = det.results[0].hypothesis.score
            else:
                label = "0"
                score = 1.0

            detections.append(
                {
                    "bbox": [x, y, z, dx, dy, dz, yaw],
                    "label": label,
                    "score": score,
                }
            )

        # 更新追踪器
        tracked_objects = self.tracker.update(detections, msg.header)

        # 始终发布（即使为空），保证下游节点能感知到追踪器存活
        tracked_msg = self._create_tracked_msg(tracked_objects, msg.header)
        self.tracked_pub.publish(tracked_msg)
        if tracked_objects:
            self._debug(f"发布 {len(tracked_objects)} 个跟踪目标")

    def _quaternion_to_yaw(self, q: Quaternion) -> float:
        """从四元数提取 yaw 角。"""
        import math

        # 假设 x=0, y=0, 只用 z 和 w 计算 yaw
        siny_spawn = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_spawn = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_spawn, cosy_spawn)
        return yaw

    def _create_tracked_msg(
        self, tracked_objects: list[dict], header
    ) -> Detection3DArray:
        """创建跟踪消息。"""
        msg = Detection3DArray()
        msg.header = header
        msg.header.frame_id = "camera_depth_optical_frame"

        for obj in tracked_objects:
            detection = Detection3D()
            bbox = obj["bbox"]
            detection.bbox.center.position = Point(x=bbox[0], y=bbox[1], z=bbox[2])
            detection.bbox.size = Vector3(x=bbox[3], y=bbox[4], z=bbox[5])
            detection.bbox.center.orientation = self._yaw_to_quaternion(bbox[6])

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(obj["label"])
            hypothesis.hypothesis.score = obj["score"]
            detection.results.append(hypothesis)
            msg.detections.append(detection)

        return msg

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        """将 yaw 角转换为四元数。"""
        import math

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return Quaternion(x=0.0, y=0.0, z=qz, w=qw)


def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._info("收到中断信号")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
