"""目标跟随节点。

该节点订阅跟踪目标，选中目标后计算跟随点并驱动机械臂到达。

功能描述：
    - 订阅 /perception/tracked_objects 获取跟踪目标列表
    - 订阅 /perception/selected_target 获取选中的目标 track_id
    - 通过 TF 获取机械臂末端执行器位置
    - 计算跟随目标点（目标中心与末端连线上，距目标 15cm）
    - 调用 MoveIt IK 求解并通过 MoveGroup Action 驱动机械臂运动

架构说明：
    - 后台线程 rclpy.spin 处理 ROS 回调（话题、TF 等）
    - 主线程执行跟随循环：IK + MoveGroup 规划执行
    - 与 moveit_rviz_planner.py 一致，避免 spin 嵌套死锁

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
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Int32
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
        self.current_joint_positions: list[float] | None = None
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

        # 初始化 MoveIt2（仅用于 IK）
        self._init_moveit2()

        # MoveGroup Action（规划+执行）
        self.move_group_client = ActionClient(self, MoveGroup, "/move_action")
        self.joint_state_subscriber = self.create_subscription(
            JointState, "joint_states", self._joint_state_callback, 10
        )

        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self._fatal("MoveGroup Action 服务不可用: /move_action")
            raise RuntimeError("MoveGroup Action 服务不可用")

        self._info("目标跟随节点已启动")

    def _init_moveit2(self):
        """初始化 MoveIt2 接口（仅用于 IK 求解）。"""
        try:
            from pymoveit2 import MoveIt2
        except ImportError as e:
            self._fatal(f"pymoveit2 未安装: {e}")
            raise

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

        self.moveit2.max_velocity = 0.3
        self.moveit2.max_acceleration = 0.3

        self.moveit2.add_collision_box(
            id="ground",
            position=(0.0, 0.0, 0.05),
            quat_xyzw=(0.0, 0.0, 0.0, 1.0),
            size=(5.0, 5.0, 0.01),
        )

        self._info("MoveIt2 初始化完成")

    def _joint_state_callback(self, msg: JointState):
        """更新当前关节状态。"""
        if len(msg.position) >= 6:
            self.current_joint_positions = list(msg.position[:6])

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

            q = bbox.center.orientation
            siny_spawn = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_spawn = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_spawn, cosy_spawn)

            track_id = None
            if det.id:
                with contextlib.suppress(ValueError):
                    track_id = int(det.id)

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

    def _publish_follow_target_pose(self, follow_point: np.ndarray):
        """发布跟随目标点（调试用）。"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.pose.position.x = float(follow_point[0])
        msg.pose.position.y = float(follow_point[1])
        msg.pose.position.z = float(follow_point[2])
        msg.pose.orientation.w = 1.0
        self.target_pose_pub.publish(msg)

    def create_move_group_goal(self, target_positions: list[float]) -> MoveGroup.Goal:
        """创建 MoveGroup goal。"""
        goal = MoveGroup.Goal()

        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = "dummy_arm"

        motion_plan_request.start_state.joint_state.header = Header()
        motion_plan_request.start_state.joint_state.header.stamp = (
            self.get_clock().now().to_msg()
        )
        motion_plan_request.start_state.joint_state.name = self.joint_names
        motion_plan_request.start_state.joint_state.position = (
            self.current_joint_positions or [0.0] * 6
        )

        joint_constraints = []
        for name, position in zip(self.joint_names, target_positions):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = name
            joint_constraint.position = position
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            joint_constraints.append(joint_constraint)

        goal_constraints = Constraints()
        goal_constraints.joint_constraints = joint_constraints
        motion_plan_request.goal_constraints = [goal_constraints]

        motion_plan_request.planner_id = "RRTConnectkConfigDefault"
        motion_plan_request.num_planning_attempts = 10
        motion_plan_request.allowed_planning_time = 10.0
        motion_plan_request.max_velocity_scaling_factor = 0.3
        motion_plan_request.max_acceleration_scaling_factor = 0.3

        goal.request = motion_plan_request
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 10

        return goal

    def move_to_joints_rad(self, joints_rad: list[float]) -> bool:
        """规划并执行到指定关节角度（弧度）。

        与 moveit_rviz_planner.py / arm_controller.py 一致:
        使用 MoveGroup Action + spin_until_future_complete。
        """
        try:
            goal = self.create_move_group_goal(joints_rad)

            self._info("发送目标到 MoveGroup...")
            send_goal_future = self.move_group_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)

            if send_goal_future.result() is None:
                self._warn("目标发送超时")
                return False

            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                self._warn("MoveGroup 目标被拒绝")
                return False

            self._info("目标已接受，开始规划和执行...")

            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=60.0)

            if get_result_future.result() is None:
                self._warn("运动执行超时")
                return False

            result = get_result_future.result()
            error_code = result.result.error_code.val
            if error_code != 1:
                self._warn(f"运动执行失败，错误码: {error_code}")
                return False

            self._info("规划和执行成功完成")
            return True

        except Exception as e:
            self._error(f"规划执行异常: {e}")
            return False

    def follow_step(self):
        """执行一次跟随: IK 求解 + MoveGroup 规划执行。"""
        if self.selected_track_id is None:
            return

        target = None
        for obj in self.tracked_objects:
            if obj["track_id"] == self.selected_track_id:
                target = obj
                break

        if target is None:
            return

        end_effector_pos = self._get_end_effector_pose()
        if end_effector_pos is None:
            return

        target_center = np.array(target["bbox"][0:3])
        follow_point = self._compute_follow_point(target_center, end_effector_pos)

        self._publish_follow_target_pose(follow_point)

        # IK 求解
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

        # 规划+执行
        joint_positions = list(joint_states.position)
        self.move_to_joints_rad(joint_positions)


def main(args=None):
    rclpy.init(args=args)
    node = FollowingNode()

    # 后台线程 spin 处理回调（与 moveit_rviz_planner.py 一致）
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        # 主循环：跟随
        rate = 1.0 / node.update_rate
        while rclpy.ok():
            try:
                node.follow_step()
            except Exception as e:
                node._error(f"跟随异常: {e}")
            time.sleep(rate)
    except KeyboardInterrupt:
        node._info("收到中断信号")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()