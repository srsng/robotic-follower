"""目标跟随节点。

该节点订阅跟踪目标，选中目标后旋转 base joint (joint1) 指向目标，
其余关节保持 View 位姿，实现简单的水平跟随。

功能描述：
    - 订阅 /perception/tracked_objects 获取跟踪目标列表
    - 订阅 /perception/selected_target 获取选中的目标 track_id
    - 计算目标在 base_link 下的方向角，只改 joint1 指向目标
    - 通过 MoveGroup Action 驱动机械臂运动（关节目标，无需 IK）
    - 目标丢失超过5秒自动回到 View 位姿

架构说明：
    - 后台线程 rclpy.spin 处理 ROS 回调
    - 单规划模型: 同一时间最多一个活跃 goal，等执行完毕后才发下一个
    - 目标位移超过1cm时才发新 goal，无活跃 goal 时立即发送

订阅话题：
    - /perception/tracked_objects (vision_msgs/Detection3DArray)
    - /perception/selected_target (std_msgs/Int32)

发布话题：
    - /perception/following_target_pose (geometry_msgs/PoseStamped)

参数：
    - follow_distance: 保留参数，当前未使用
    - tracking_topic, selected_topic, update_rate
"""

import contextlib
import math
import os
import subprocess
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

# View 位姿关节角 (度): joint1=0, joint2=-36.04, joint3=-21.09, joint4=0, joint5=-89.63, joint6=0
VIEW_POSE_RAD = [math.radians(d) for d in [0, -36.04, -21.09, 0, -89.63, 0]]
VIEW_POSE_TIMEOUT_SEC = 5.0


class FollowingNode(NodeWrapper):
    """目标跟随节点 - 只旋转 base joint 指向目标。"""

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
        self.update_rate = self.declare_and_get_parameter("update_rate", 2.0)

        # TF 初始化
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 订阅话题
        self.tracked_sub = self.create_subscription(
            Detection3DArray, tracking_topic, self.tracked_callback, 10
        )
        self.selected_sub = self.create_subscription(
            Int32, selected_topic, self.selected_callback, 10
        )

        # 发布跟随目标点（调试用）
        self.target_pose_pub = self.create_publisher(
            PoseStamped, "/perception/following_target_pose", 10
        )

        # 状态
        self.selected_track_id: int | None = None
        self.tracked_objects: list[dict] = []
        self.current_joint_positions: list[float] | None = None
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

        # 超时回 View 位姿
        self._last_target_time: float | None = None
        self._returning_to_view: bool = False

        # 运动控制状态 (单规划模型)
        self._motion_lock = threading.Lock()
        self._active_goal_handle = None
        self._goal_generation = 0
        self._last_commanded_xyz: tuple[float, float, float] | None = None
        self._DISPLACEMENT_THRESHOLD = 0.01  # 1cm

        # 添加地面障碍物
        self._add_ground_obstacle()

        # MoveGroup Action（规划+执行）
        self.move_group_client = ActionClient(self, MoveGroup, "/move_action")
        self.joint_state_subscriber = self.create_subscription(
            JointState, "joint_states", self._joint_state_callback, 10
        )

        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self._fatal("MoveGroup Action 服务不可用: /move_action")
            raise RuntimeError("MoveGroup Action 服务不可用")

        self._info("目标跟随节点已启动 (joint1-only 模式)")

    def _add_ground_obstacle(self):
        """添加地面障碍物到 MoveIt planning scene。"""
        try:
            self._info("正在添加地面障碍物...")
            ws_path = os.path.expanduser("~/ros2_ws")
            script_path = os.path.join(
                os.path.dirname(__file__), "../../script/add_ground_obstacle.py"
            )
            result = subprocess.run(
                [
                    "bash",
                    "-lc",
                    f"cd {ws_path} && source install/setup.bash >/dev/null 2>&1 && "
                    f"python3 {script_path}",
                ],
                capture_output=True,
                text=True,
                timeout=15,
            )
            if result.returncode != 0:
                self._warn(f"添加地面障碍物脚本执行失败: {result.stderr}")
            else:
                self._info("地面障碍物添加完成")
        except Exception as e:
            self._warn(f"添加地面障碍物异常: {e}")

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

    def _compute_follow_joints(self, target_x: float, target_y: float) -> list[float] | None:
        """计算跟随关节角: 只改 joint1 指向目标, 其余保持 View 位姿。

        arm 在 joint1=0 时面向 base_link 的 -Y 方向,
        所以 yaw = atan2(target_x, -target_y).

        Args:
            target_x: 目标在 base_link 下的 X 坐标
            target_y: 目标在 base_link 下的 Y 坐标

        Returns:
            6个关节角度列表 (弧度), 或 None 如果偏转角过大
        """
        yaw_rad = math.atan2(target_x, -target_y)

        max_yaw = math.radians(90)
        if abs(yaw_rad) > max_yaw:
            self._warn(f"偏转角过大 {math.degrees(yaw_rad):.1f}°, 跳过")
            return None

        joints = list(VIEW_POSE_RAD)
        joints[0] = yaw_rad
        return joints

    def _publish_follow_target_pose(self, target_x: float, target_y: float, target_z: float):
        """发布跟随目标点（调试用）。"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.pose.position.x = target_x
        msg.pose.position.y = target_y
        msg.pose.position.z = target_z
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
        motion_plan_request.num_planning_attempts = 3
        motion_plan_request.allowed_planning_time = 2.0
        motion_plan_request.max_velocity_scaling_factor = 0.5
        motion_plan_request.max_acceleration_scaling_factor = 0.5

        goal.request = motion_plan_request
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        return goal

    def _send_move_goal(self, joints_rad: list[float]):
        """非阻塞发送 MoveGroup goal，使用 generation 过滤过期回调。

        on_result 只处理当前 generation 的结果，旧 generation 的 result 静默丢弃。
        goal 完成后清除 _active_goal_handle，follow_step 即可发送下一个 goal。
        """
        goal = self.create_move_group_goal(joints_rad)

        with self._motion_lock:
            self._goal_generation += 1
            gen = self._goal_generation

        def on_goal_response(future):
            try:
                goal_handle = future.result()
                if not goal_handle.accepted:
                    self._warn("MoveGroup 目标被拒绝")
                    with self._motion_lock:
                        if self._goal_generation == gen:
                            self._active_goal_handle = None
                    return
                self._info("目标已接受，开始规划和执行...")
                with self._motion_lock:
                    if self._goal_generation == gen:
                        self._active_goal_handle = goal_handle
                goal_handle.get_result_async().add_done_callback(on_result)
            except Exception as e:
                self._error(f"目标响应异常: {e}")
                with self._motion_lock:
                    if self._goal_generation == gen:
                        self._active_goal_handle = None

        def on_result(future):
            with self._motion_lock:
                is_current = self._goal_generation == gen
                if is_current:
                    self._active_goal_handle = None
            if not is_current:
                return
            try:
                res = future.result()
                error_code = res.result.error_code.val
                if error_code != 1:
                    self._warn(f"运动执行失败，错误码: {error_code}")
                else:
                    self._info("规划和执行成功完成")
            except Exception as e:
                self._error(f"结果回调异常: {e}")

        self._info(f"发送目标 joint1={math.degrees(joints_rad[0]):.1f}°")
        self.move_group_client.send_goal_async(goal).add_done_callback(on_goal_response)

    def follow_step(self):
        """执行一次跟随: 单规划模型，有活跃 goal 时跳过，否则按需发送。"""
        now = time.monotonic()

        with self._motion_lock:
            has_active = self._active_goal_handle is not None
        if has_active:
            return

        if self.selected_track_id is None:
            self._last_target_time = None
            self._returning_to_view = False
            self._last_commanded_xyz = None
            return

        target = None
        for obj in self.tracked_objects:
            if obj["track_id"] == self.selected_track_id:
                target = obj
                break

        if target is not None:
            self._last_target_time = now
            self._returning_to_view = False
        else:
            if self._last_target_time is None:
                return

            elapsed = now - self._last_target_time
            if elapsed <= VIEW_POSE_TIMEOUT_SEC:
                return

            if not self._returning_to_view:
                self._info("目标丢失超过阈值，回到 View 位姿")
                self._returning_to_view = True
                self._last_commanded_xyz = None
                self._send_move_goal(VIEW_POSE_RAD)
            return

        target_x, target_y, target_z = (
            target["bbox"][0],
            target["bbox"][1],
            target["bbox"][2],
        )

        if self._last_commanded_xyz is not None:
            dx = target_x - self._last_commanded_xyz[0]
            dy = target_y - self._last_commanded_xyz[1]
            dz = target_z - self._last_commanded_xyz[2]
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist < self._DISPLACEMENT_THRESHOLD:
                return

        self._info(f"目标位置: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})")
        self._publish_follow_target_pose(target_x, target_y, target_z)

        joints = self._compute_follow_joints(target_x, target_y)
        if joints is None:
            return

        yaw_deg = math.degrees(joints[0])
        self._info(f"跟随: joint1={yaw_deg:.1f}°")

        self._send_move_goal(joints)
        self._last_commanded_xyz = (target_x, target_y, target_z)


def main(args=None):
    rclpy.init(args=args)
    node = FollowingNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
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
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()