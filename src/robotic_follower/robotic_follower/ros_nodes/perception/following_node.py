"""目标跟随节点。

该节点订阅跟踪目标，选中目标后计算跟随点并驱动机械臂到达。

功能描述：
    - 订阅 /perception/tracked_objects 获取跟踪目标列表
    - 订阅 /perception/selected_target 获取选中的目标 track_id
    - 通过 TF 获取机械臂末端执行器位置
    - 计算跟随目标点（目标中心与末端连线上，距目标 15cm）
    - 调用 MoveIt IK 求解并驱动机械臂运动

订阅话题：
    - /perception/tracked_objects (vision_msgs/Detection3DArray)
        跟踪目标列表
    - /perception/selected_target (std_msgs/Int32)
        选中的目标 track_id

发布话题：
    - /perception/following_target_pose (geometry_msgs/PoseStamped)
        跟随目标点（调试用）

参数：
    - follow_distance (float, 默认 0.15)
        跟随距离，单位米
    - tracking_topic (string, 默认 "/perception/tracked_objects")
        跟踪目标话题
    - selected_topic (string, 默认 "/perception/selected_target")
        选中目标话题
    - end_effector_frame (string, 默认 "link6_1_1")
        末端执行器 frame
    - update_rate (float, 默认 5.0)
        跟随更新频率

使用示例：
    ros2 run robotic_follower following_node
    ros2 run robotic_follower following_node --ros-args -p follow_distance:=0.2
"""

import contextlib
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection3DArray

from robotic_follower.util.wrapper import NodeWrapper


class FollowingNode(NodeWrapper):
    """目标跟随节点。"""

    def __init__(self):
        super().__init__("following_node")

        # 参数
        self.follow_distance = self.declare_and_get_parameter("follow_distance", 0.15)
        tracking_topic = self.declare_and_get_parameter(
            "tracking_topic", "/perception/tracked_objects"
        )
        selected_topic = self.declare_and_get_parameter(
            "selected_topic", "/perception/selected_target"
        )
        self.end_effector_frame = self.declare_and_get_parameter(
            "end_effector_frame", "link6_1_1"
        )
        self.update_rate = self.declare_and_get_parameter("update_rate", 5.0)

        # TF 初始化
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 订阅话题
        self.tracked_sub = self.create_subscription(
            Detection3DArray,
            tracking_topic,
            self.tracked_callback,
            10,
        )
        self.selected_sub = self.create_subscription(
            Int32,
            selected_topic,
            self.selected_callback,
            10,
        )

        # 发布跟随目标点（调试用）
        self.target_pose_pub = self.create_publisher(
            PoseStamped, "/perception/following_target_pose", 10
        )

        # 当前选中的目标
        self.selected_track_id: int | None = None
        self.tracked_objects: list[dict] = []

        # 初始化 MoveIt2
        self._init_moveit2()

        # 启动定时器进行跟随
        self._follow_timer = self.create_timer(
            1.0 / self.update_rate, self._follow_timer_callback
        )

        self._info("目标跟随节点已启动")

    def _init_moveit2(self):
        """初始化 MoveIt2 接口。"""
        try:
            from pymoveit2 import MoveIt2
        except ImportError as e:
            self._fatal(f"pymoveit2 未安装: {e}")
            raise

        # 直接定义机械臂配置（与 URDF 一致）
        joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        base_link_name = "base_link"
        end_effector_name = "link6_1_1"
        group_name = "dummy_arm"

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=joint_names,
            base_link_name=base_link_name,
            end_effector_name=end_effector_name,
            group_name=group_name,
        )

        # 设置速度和加速度
        self.moveit2.max_velocity = 0.3
        self.moveit2.max_acceleration = 0.3

        # 添加地面障碍物: 中心(0,0,0.05), 5m x 5m, 厚度0.01m
        self.moveit2.add_collision_box(
            id="ground",
            position=(0.0, 0.0, 0.05),
            quat_xyzw=(0.0, 0.0, 0.0, 1.0),
            size=(5.0, 5.0, 0.01),
        )

        # 使用单线程 executor，MoveIt2 服务调用在 spin 中处理
        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self)

        self._info("MoveIt2 初始化完成")

    def tracked_callback(self, msg: Detection3DArray):
        """跟踪目标回调。"""
        self.tracked_objects = []
        for det in msg.detections:
            bbox = det.bbox
            x, y, z = (
                bbox.center.position.x,
                bbox.center.position.y,
                bbox.center.position.z,
            )
            dx, dy, dz = bbox.size.x, bbox.size.y, bbox.size.z

            # 从四元数提取 yaw
            q = bbox.center.orientation
            siny_spawn = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_spawn = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_spawn, cosy_spawn)

            # 提取 track_id（从 Detection3D.id 字段）
            track_id = None
            if det.id:
                with contextlib.suppress(ValueError):
                    track_id = int(det.id)

            # 提取类别标签
            label = "unknown"
            if det.results:
                label = det.results[0].hypothesis.class_id

            self.tracked_objects.append(
                {
                    "track_id": track_id,
                    "label": label,
                    "bbox": [x, y, z, dx, dy, dz, yaw],
                }
            )

    def selected_callback(self, msg: Int32):
        """选中目标回调。"""
        track_id = msg.data
        if track_id > 0:
            self.selected_track_id = track_id
            self._info(f"选中目标: track_id={track_id}")
        else:
            self.selected_track_id = None
            self._info("取消跟随")

    def _get_end_effector_pose(self) -> np.ndarray | None:
        """获取末端执行器在 base_link 下的位置。"""
        try:
            now = rclpy.time.Time(seconds=0)
            transform = self.tf_buffer.lookup_transform(
                "base_link",
                self.end_effector_frame,
                now,
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            pos = transform.transform.translation
            return np.array([pos.x, pos.y, pos.z])
        except Exception as e:
            self._warn(f"获取末端位置失败: {e}")
            return None

    def _compute_follow_point(
        self, target_center: np.ndarray, end_effector_pos: np.ndarray
    ) -> np.ndarray:
        """计算跟随目标点。

        目标点位于目标中心与末端执行器连线上，距目标中心 follow_distance 处。

        Args:
            target_center: 目标中心位置 [x, y, z]
            end_effector_pos: 末端执行器位置 [x, y, z]

        Returns:
            跟随目标点 [x, y, z]
        """
        direction = end_effector_pos - target_center
        distance = np.linalg.norm(direction)

        if distance < 1e-6:
            self._warn("末端与目标距离过近，使用默认方向")
            return target_center + np.array([0, 0, self.follow_distance])

        direction = direction / distance
        follow_point = target_center + direction * self.follow_distance
        return follow_point

    def _follow_timer_callback(self):
        """定时执行跟随。"""
        if self.selected_track_id is None:
            return

        # 查找选中的目标
        target = None
        for obj in self.tracked_objects:
            if obj["track_id"] == self.selected_track_id:
                target = obj
                break

        if target is None:
            self._warn(f"未找到 track_id={self.selected_track_id} 的目标")
            return

        # 获取末端位置
        end_effector_pos = self._get_end_effector_pose()
        if end_effector_pos is None:
            return

        # 计算跟随点
        target_center = np.array(target["bbox"][0:3])
        follow_point = self._compute_follow_point(target_center, end_effector_pos)

        # 发布目标点（调试用）
        self._publish_follow_target_pose(follow_point)

        # 执行 IK 并移动
        self._move_to_follow_point(follow_point)

    def _publish_follow_target_pose(self, follow_point: np.ndarray):
        """发布跟随目标点（调试用）。"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.pose.position.x = float(follow_point[0])
        msg.pose.position.y = float(follow_point[1])
        msg.pose.position.z = float(follow_point[2])
        # 默认朝下（沿 -Z 方向）
        msg.pose.orientation.w = 1.0
        self.target_pose_pub.publish(msg)

    def _move_to_follow_point(self, follow_point: np.ndarray):
        """移动到跟随目标点。"""
        try:
            # 计算 IK（使用默认方向，朝下）
            # quaternion: w=1, x=y=z=0 表示无旋转
            quat_xyzw = (0.0, 0.0, 0.0, 1.0)

            joint_states = self.moveit2.compute_ik(
                position=(
                    float(follow_point[0]),
                    float(follow_point[1]),
                    float(follow_point[2]),
                ),
                quat_xyzw=quat_xyzw,
            )

            if joint_states is None:
                self._warn("IK 求解失败")
                return

            # 移动到目标关节角度
            joint_positions = list(joint_states.position)
            self.moveit2.move_to_configuration(joint_positions)
            self.moveit2.wait_until_executed()

            self._debug(f"已移动到跟随点: {follow_point}")

        except Exception as e:
            self._error(f"移动失败: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = FollowingNode()
    try:
        node._executor.spin()
    except KeyboardInterrupt:
        node._info("收到中断信号")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
