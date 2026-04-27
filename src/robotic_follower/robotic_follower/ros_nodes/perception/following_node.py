"""目标跟随节点。

该节点订阅跟踪目标，选中目标后计算跟随位姿驱动机械臂靠近物体，
同时保持一定距离。IK无解时降级为只旋转joint1的水平跟随模式。

功能描述：
    - 订阅 /perception/tracked_objects 获取跟踪目标列表
    - 订阅 /perception/selected_target 获取选中的目标 track_id
    - 计算目标在 base_link 下的位姿，沿基座→物体径向方向靠近
    - 不超过70%臂展，距物体表面至少MIN_CLEARANCE，目标点离基座至少MIN_TARGET_DIST
    - EE朝向始终指向物体，确保相机视野覆盖目标
    - IK无解时降级为 joint1-only 水平旋转模式
    - 目标丢失超过5秒自动回到 View 位姿

架构说明：
    - 后台线程 rclpy.spin 处理 ROS 回调
    - 单规划模型: 同一时间最多一个活跃 goal，等执行完毕才发下一个
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
from geometry_msgs.msg import Pose, PoseStamped, Vector3
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header, Int32
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection3DArray

from robotic_follower.util.perf import PerfTimer
from robotic_follower.util.wrapper import NodeWrapper


# View 位姿关节角 (度): joint1=0, joint2=-36.04, joint3=-21.09, joint4=0, joint5=-89.63, joint6=0
VIEW_POSE_RAD = [math.radians(d) for d in [0, -36.04, -21.09, 0, -89.63, 0]]
VIEW_POSE_TIMEOUT_SEC = 8.0
VIEW_RETRY_INTERVAL_SEC = 2.0

MAX_REACH = 0.38
MIN_CLEARANCE = 0.15
MIN_APPROACH_DIST = 0.15
MIN_TARGET_DIST = 0.20
MIN_EE_HEIGHT = 0.18
POSITION_TOLERANCE = 0.01
ORIENTATION_TOLERANCE = 0.52
POSE_DISPLACEMENT_THRESHOLD = 0.03
NUMERIC_EPS = 1e-6
JOINT6_TOLERANCE = 0.01
DEFAULT_REASSOC_GATE_M = 0.20
DEFAULT_LOST_HOLD_SEC = 0.8


class FollowingNode(NodeWrapper):
    """目标跟随节点 - 优先位姿跟随，IK无解降级为joint1旋转。"""

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
        self.reassoc_gate_m = float(
            self.declare_and_get_parameter("reassoc_gate_m", DEFAULT_REASSOC_GATE_M)
        )
        self.lost_hold_sec = float(
            self.declare_and_get_parameter("lost_hold_sec", DEFAULT_LOST_HOLD_SEC)
        )

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
        self._selected_label: str | None = None
        self._last_seen_target: dict | None = None
        self.tracked_objects: list[dict] = []
        self.current_joint_positions: list[float] | None = None
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

        # 超时回 View 位姿
        self._last_target_time: float | None = None
        self._returning_to_view: bool = False
        self._view_goal_reached: bool = False
        self._last_view_cmd_time: float = 0.0
        self._tracked_frame_id: str = ""
        self._last_frame_mismatch_warn_time: float = 0.0

        # 运动控制状态 (单规划模型)
        self._motion_lock = threading.Lock()
        self._active_goal_handle = None
        self._goal_generation = 0
        self._last_commanded_xyz: tuple[float, float, float] | None = None
        self._last_commanded_ee_pos: np.ndarray | None = None
        self._DISPLACEMENT_THRESHOLD = 0.01  # 1cm
        self._motion_step_id = 0
        self.perf_aggregate_interval = int(
            self.declare_and_get_parameter("perf_aggregate_interval", 100)
        )
        self._motion_perf_timer = PerfTimer(
            lambda level, msg, channel: self._log(level, msg, channel=channel),
            channel="motion",
            stats_channel="motion_stats",
            aggregate_interval=self.perf_aggregate_interval,
        )
        self._goal_sent_ts: dict[int, float] = {}
        self._goal_mode: dict[int, str] = {}

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

        self._info("目标跟随节点已启动 (pose-follow 模式)")

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
        self._tracked_frame_id = msg.header.frame_id
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
            self._selected_label = None
            self._last_seen_target = None
            self._info(f"选中目标: track_id={track_id}")
        else:
            self.selected_track_id = None
            self._selected_label = None
            self._last_seen_target = None
            self._info("取消跟随，强制回归 View 位姿")
            self._force_return_to_view("cancel_tracking")

    def _force_return_to_view(self, reason: str):
        """强制回归 View 位姿。必要时取消当前目标并立即发送回位目标。"""
        now = time.monotonic()

        with self._motion_lock:
            active_handle = self._active_goal_handle
            self._active_goal_handle = None

        if active_handle is not None:
            self._warn(f"强制回位触发({reason})，取消当前运动")
            try:
                active_handle.cancel_goal_async()
            except Exception as e:
                self._warn(f"取消当前运动异常: {e}")

        self._last_target_time = None
        self._returning_to_view = True
        self._view_goal_reached = False
        self._last_view_cmd_time = now
        self._last_seen_target = None
        self._last_commanded_xyz = None
        self._last_commanded_ee_pos = None

        self._send_move_goal(VIEW_POSE_RAD, mode="view")

    def _select_target_candidate(self) -> dict | None:
        """选择本帧用于跟随的目标。

        1) 优先按 selected_track_id 精确匹配
        2) 若失配，按几何连续性进行重关联（可选标签一致）
        """
        if self.selected_track_id is None:
            return None

        exact = None
        for obj in self.tracked_objects:
            if obj["track_id"] == self.selected_track_id:
                exact = obj
                break
        if exact is not None:
            if self._selected_label is None:
                self._selected_label = str(exact.get("label", "unknown"))
            self._last_seen_target = exact
            return exact

        if self._last_seen_target is None:
            return None

        prev = np.array(self._last_seen_target["bbox"][:3], dtype=np.float64)
        best_obj = None
        best_dist = float("inf")
        for obj in self.tracked_objects:
            if self._selected_label is not None and str(obj.get("label", "unknown")) != self._selected_label:
                continue
            pos = np.array(obj["bbox"][:3], dtype=np.float64)
            dist = float(np.linalg.norm(pos - prev))
            if dist < best_dist:
                best_dist = dist
                best_obj = obj

        if best_obj is not None and best_dist <= self.reassoc_gate_m:
            new_track_id = best_obj.get("track_id")
            if new_track_id is not None and new_track_id != self.selected_track_id:
                self._warn(
                    f"重关联目标: {self.selected_track_id} -> {new_track_id}, 距离={best_dist:.3f}m"
                )
                self.selected_track_id = int(new_track_id)
            self._last_seen_target = best_obj
            return best_obj

        return None

    def _compute_follow_joints(
        self, target_x: float, target_y: float
    ) -> list[float] | None:
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

    @staticmethod
    def _rotation_matrix_to_quaternion(R):
        """旋转矩阵转四元数 (x, y, z, w)。"""
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        return (x, y, z, w)

    def _get_current_ee_position(self):
        """通过TF获取当前末端执行器位置 (base_link坐标系)。"""
        try:
            t = self.tf_buffer.lookup_transform(
                "base_link", "link6_1_1", rclpy.time.Time()
            )
            return np.array(
                [
                    t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z,
                ]
            )
        except Exception:
            return np.array([0.0, -0.295, 0.318])

    def _compute_reach_orientation(self, ee_pos, obj_pos):
        """计算EE朝向: -Y轴指向物体, z轴允许倾斜但z分量必须为正。

        Args:
            ee_pos: EE目标位置 (base_link坐标系)
            obj_pos: 物体中心位置 (base_link坐标系)

        Returns:
            四元数 (x, y, z, w)
        """
        approach = obj_pos - ee_pos
        approach_norm = np.linalg.norm(approach)
        if approach_norm < 1e-6:
            return (0.0, 0.0, 0.0, 1.0)
        approach_unit = approach / approach_norm

        ee_y = -approach_unit
        up_candidates = [
            np.array([0.0, 0.0, 1.0]),
            np.array([1.0, 0.0, 0.0]),
            np.array([0.0, 1.0, 0.0]),
        ]

        ee_x = None
        for up in up_candidates:
            candidate = np.cross(ee_y, up)
            norm = np.linalg.norm(candidate)
            if norm > NUMERIC_EPS:
                ee_x = candidate / norm
                break
        if ee_x is None:
            return (0.0, 0.0, 0.0, 1.0)

        ee_z = np.cross(ee_x, ee_y)
        ee_z_norm = np.linalg.norm(ee_z)
        if ee_z_norm < NUMERIC_EPS:
            return (0.0, 0.0, 0.0, 1.0)
        ee_z = ee_z / ee_z_norm

        if ee_z[2] < 0:
            ee_x = -ee_x
            ee_z = -ee_z

        R = np.column_stack([ee_x, ee_y, ee_z])
        return self._rotation_matrix_to_quaternion(R)

    def _compute_target_pose(self, x, y, z, dx, dy, dz):
        """计算跟随目标位姿: 在水平面保持安全距离并朝向物体。

        目标点在 XY 平面从物体朝基座方向退回，Z 不低于物体中心，
        避免规划点落到物体下方或贴近地面。
        朝向使用目标位姿→物体方向，确保相机指向物体。

        Args:
            x, y, z: 物体bbox中心 (base_link坐标系)
            dx, dy, dz: 物体bbox尺寸

        Returns:
            (position, quaternion) 或 None (降级为joint-only)
        """
        obj_pos = np.array([x, y, z], dtype=np.float64)

        # 只用 bbox 的水平尺寸计算安全距离，防止被 z 尺寸拉得过远。
        obj_radius_xy = math.hypot(dx / 2.0, dy / 2.0)
        clearance_xy = obj_radius_xy + MIN_CLEARANCE

        # 在水平面上选择从物体指向基座的退让方向。
        obj_xy = obj_pos[:2]
        obj_xy_norm = np.linalg.norm(obj_xy)
        if obj_xy_norm > NUMERIC_EPS:
            retreat_dir_xy = obj_xy / obj_xy_norm
        else:
            ee_xy = self._get_current_ee_position()[:2]
            fallback = ee_xy - obj_xy
            fallback_norm = np.linalg.norm(fallback)
            if fallback_norm < NUMERIC_EPS:
                retreat_dir_xy = np.array([0.0, -1.0], dtype=np.float64)
            else:
                retreat_dir_xy = fallback / fallback_norm

        target_xy = obj_xy - retreat_dir_xy * clearance_xy
        target_z = max(float(z), MIN_EE_HEIGHT)
        target_pos = np.array([target_xy[0], target_xy[1], target_z], dtype=np.float64)

        # 70% 臂展约束：优先保留高度和方向，压缩水平半径。
        max_xy = math.sqrt(max(MAX_REACH**2 - target_z**2, 0.0))
        target_xy_norm = np.linalg.norm(target_xy)
        if target_xy_norm > max_xy and target_xy_norm > NUMERIC_EPS:
            target_xy = target_xy / target_xy_norm * max_xy
            target_pos[0] = target_xy[0]
            target_pos[1] = target_xy[1]
            target_xy_norm = np.linalg.norm(target_xy)

        target_dist = np.linalg.norm(target_pos)
        if target_dist < MIN_TARGET_DIST:
            if target_xy_norm < NUMERIC_EPS:
                return None
            min_xy = math.sqrt(max(MIN_TARGET_DIST**2 - target_z**2, 0.0))
            target_xy = target_xy / target_xy_norm * min_xy
            target_pos[0] = target_xy[0]
            target_pos[1] = target_xy[1]

        # 水平面安全距离检查：避免切进 bbox。
        dist_to_obj_xy = np.linalg.norm(target_pos[:2] - obj_pos[:2])
        if dist_to_obj_xy < clearance_xy:
            self._warn(
                f"目标点水平距离过近 ({dist_to_obj_xy:.3f}m < {clearance_xy:.3f}m), 降级"
            )
            return None

        orientation = self._compute_reach_orientation(target_pos, obj_pos)
        return (target_pos, orientation)

    def _publish_follow_target_pose(
        self, target_x: float, target_y: float, target_z: float
    ):
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

    def create_reach_goal(self, position, orientation_quat):
        """创建位姿跟随 MoveGroup goal (PositionConstraint + OrientationConstraint)。

        Args:
            position: 目标位置 np.array([x,y,z]) (base_link坐标系)
            orientation_quat: 目标姿态四元数 (x,y,z,w)
        """
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

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "base_link"
        pos_constraint.link_name = "link6_1_1"
        pos_constraint.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)

        pos_constraint.constraint_region.primitives.append(SolidPrimitive())
        pos_constraint.constraint_region.primitives[0].type = SolidPrimitive.SPHERE
        pos_constraint.constraint_region.primitives[0].dimensions = [POSITION_TOLERANCE]

        pos_constraint.constraint_region.primitive_poses.append(Pose())
        pos_constraint.constraint_region.primitive_poses[0].position.x = float(
            position[0]
        )
        pos_constraint.constraint_region.primitive_poses[0].position.y = float(
            position[1]
        )
        pos_constraint.constraint_region.primitive_poses[0].position.z = float(
            position[2]
        )
        pos_constraint.constraint_region.primitive_poses[0].orientation.w = 1.0
        pos_constraint.weight = 1.0

        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = "base_link"
        ori_constraint.link_name = "link6_1_1"
        ori_constraint.orientation.x = float(orientation_quat[0])
        ori_constraint.orientation.y = float(orientation_quat[1])
        ori_constraint.orientation.z = float(orientation_quat[2])
        ori_constraint.orientation.w = float(orientation_quat[3])
        ori_constraint.absolute_x_axis_tolerance = ORIENTATION_TOLERANCE
        ori_constraint.absolute_y_axis_tolerance = ORIENTATION_TOLERANCE
        ori_constraint.absolute_z_axis_tolerance = ORIENTATION_TOLERANCE
        ori_constraint.weight = 1.0

        joint6_constraint = JointConstraint()
        joint6_constraint.joint_name = "joint6"
        joint6_constraint.position = float(VIEW_POSE_RAD[5])
        joint6_constraint.tolerance_above = JOINT6_TOLERANCE
        joint6_constraint.tolerance_below = JOINT6_TOLERANCE
        joint6_constraint.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.position_constraints.append(pos_constraint)
        goal_constraints.orientation_constraints.append(ori_constraint)
        goal_constraints.joint_constraints.append(joint6_constraint)
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

    def _send_reach_goal(self, position, orientation_quat, perf=None, mode: str = "pose"):
        """非阻塞发送位姿跟随 MoveGroup goal。"""
        t_goal_create_start = time.monotonic()
        goal = self.create_reach_goal(position, orientation_quat)
        if perf is not None:
            perf.record("goal_create", t_goal_create_start)

        with self._motion_lock:
            self._goal_generation += 1
            gen = self._goal_generation
        self._goal_sent_ts[gen] = time.monotonic()
        self._goal_mode[gen] = mode

        def _emit_async_result(success: bool, error_code: int):
            sent_ts = self._goal_sent_ts.pop(gen, None)
            elapsed = max(0.0, time.monotonic() - sent_ts) if sent_ts else 0.0
            mode_val = self._goal_mode.pop(gen, mode)
            self._motion_perf_timer.emit(
                {
                    "step_id": gen,
                    "mode": mode_val,
                    "async_plan_exec": elapsed,
                    "success": success,
                    "error_code": error_code,
                },
                level="debug",
            )

        def on_goal_response(future):
            try:
                goal_handle = future.result()
                if not goal_handle.accepted:
                    self._warn("MoveGroup 位姿目标被拒绝")
                    _emit_async_result(False, -2)
                    with self._motion_lock:
                        if self._goal_generation == gen:
                            self._active_goal_handle = None
                    return
                self._info("位姿目标已接受，开始规划和执行...")
                with self._motion_lock:
                    if self._goal_generation == gen:
                        self._active_goal_handle = goal_handle
                goal_handle.get_result_async().add_done_callback(on_result)
            except Exception as e:
                self._error(f"位姿目标响应异常: {e}")
                _emit_async_result(False, -3)
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
                    self._warn(f"位姿运动执行失败，错误码: {error_code}")
                    if mode == "view":
                        self._view_goal_reached = False
                    _emit_async_result(False, int(error_code))
                else:
                    self._info("位姿规划和执行成功完成")
                    if mode == "view":
                        self._view_goal_reached = True
                    _emit_async_result(True, int(error_code))
            except Exception as e:
                self._error(f"位姿结果回调异常: {e}")
                if mode == "view":
                    self._view_goal_reached = False
                _emit_async_result(False, -4)

        ee_dist = np.linalg.norm(position)
        self._info(
            f"发送位姿目标: ({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}) "
            f"距基座={ee_dist:.3f}m"
        )
        t_goal_send_start = time.monotonic()
        self.move_group_client.send_goal_async(goal).add_done_callback(on_goal_response)
        if perf is not None:
            perf.record("goal_send", t_goal_send_start)

    def _send_move_goal(self, joints_rad: list[float], perf=None, mode: str = "joint"):
        """非阻塞发送 MoveGroup goal，使用 generation 过滤过期回调。

        on_result 只处理当前 generation 的结果，旧 generation 的 result 静默丢弃。
        goal 完成后清除 _active_goal_handle，follow_step 即可发送下一个 goal。
        """
        t_goal_create_start = time.monotonic()
        goal = self.create_move_group_goal(joints_rad)
        if perf is not None:
            perf.record("goal_create", t_goal_create_start)

        with self._motion_lock:
            self._goal_generation += 1
            gen = self._goal_generation
        self._goal_sent_ts[gen] = time.monotonic()
        self._goal_mode[gen] = mode

        def _emit_async_result(success: bool, error_code: int):
            sent_ts = self._goal_sent_ts.pop(gen, None)
            elapsed = max(0.0, time.monotonic() - sent_ts) if sent_ts else 0.0
            mode_val = self._goal_mode.pop(gen, mode)
            self._motion_perf_timer.emit(
                {
                    "step_id": gen,
                    "mode": mode_val,
                    "async_plan_exec": elapsed,
                    "success": success,
                    "error_code": error_code,
                },
                level="debug",
            )

        def on_goal_response(future):
            try:
                goal_handle = future.result()
                if not goal_handle.accepted:
                    self._warn("MoveGroup 目标被拒绝")
                    _emit_async_result(False, -2)
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
                _emit_async_result(False, -3)
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
                    if mode == "view":
                        self._view_goal_reached = False
                    _emit_async_result(False, int(error_code))
                else:
                    self._info("规划和执行成功完成")
                    if mode == "view":
                        self._view_goal_reached = True
                    _emit_async_result(True, int(error_code))
            except Exception as e:
                self._error(f"结果回调异常: {e}")
                if mode == "view":
                    self._view_goal_reached = False
                _emit_async_result(False, -4)

        self._info(f"发送目标 joint1={math.degrees(joints_rad[0]):.1f}°")
        t_goal_send_start = time.monotonic()
        self.move_group_client.send_goal_async(goal).add_done_callback(on_goal_response)
        if perf is not None:
            perf.record("goal_send", t_goal_send_start)

    def follow_step(self):
        """执行一次跟随: 单规划模型，有活跃 goal 时跳过，否则按需发送。"""
        now = time.monotonic()
        perf = self._motion_perf_timer.frame()
        perf.mark("start")
        self._motion_step_id += 1
        mode = "none"
        command_sent = False

        with self._motion_lock:
            has_active = self._active_goal_handle is not None
        if has_active:
            perf.record("total_follow", "start")
            perf.flush(
                extra={
                    "step_id": self._motion_step_id,
                    "mode": mode,
                    "command_sent": command_sent,
                    "has_active_goal": True,
                }
            )
            return

        if self.selected_track_id is None:
            self._last_target_time = None
            self._returning_to_view = False
            self._view_goal_reached = False
            self._last_view_cmd_time = 0.0
            self._last_commanded_xyz = None
            self._last_commanded_ee_pos = None
            perf.record("total_follow", "start")
            perf.flush(
                extra={
                    "step_id": self._motion_step_id,
                    "mode": mode,
                    "command_sent": command_sent,
                    "selected": False,
                }
            )
            return

        if self._tracked_frame_id and self._tracked_frame_id != "base_link":
            if now - self._last_frame_mismatch_warn_time > 1.0:
                self._warn(
                    "tracked_objects frame_id 非 base_link: "
                    f"{self._tracked_frame_id}, 跳过本次跟随"
                )
                self._last_frame_mismatch_warn_time = now
            perf.record("total_follow", "start")
            perf.flush(
                extra={
                    "step_id": self._motion_step_id,
                    "mode": mode,
                    "command_sent": command_sent,
                    "frame_mismatch": True,
                }
            )
            return

        t_lookup_start = perf.mark("target_lookup_start")
        target = self._select_target_candidate()
        perf.record("target_lookup", t_lookup_start)

        if target is not None:
            self._last_target_time = now
            self._returning_to_view = False
            self._view_goal_reached = False
        else:
            if self._last_target_time is None:
                perf.record("total_follow", "start")
                perf.flush(
                    extra={
                        "step_id": self._motion_step_id,
                        "mode": mode,
                        "command_sent": command_sent,
                        "target_visible": False,
                    }
                )
                return

            elapsed = now - self._last_target_time
            if elapsed <= self.lost_hold_sec:
                perf.record("total_follow", "start")
                perf.flush(
                    extra={
                        "step_id": self._motion_step_id,
                        "mode": mode,
                        "command_sent": command_sent,
                        "target_visible": False,
                        "lost_stage": "hold",
                        "elapsed_since_last_target": elapsed,
                    }
                )
                return

            if elapsed <= VIEW_POSE_TIMEOUT_SEC:
                perf.record("total_follow", "start")
                perf.flush(
                    extra={
                        "step_id": self._motion_step_id,
                        "mode": mode,
                        "command_sent": command_sent,
                        "target_visible": False,
                        "lost_stage": "wait_view",
                        "elapsed_since_last_target": elapsed,
                    }
                )
                return

            if not self._returning_to_view:
                self._info("目标丢失超过阈值，回到 View 位姿")
                self._returning_to_view = True
                self._view_goal_reached = False
                self._last_view_cmd_time = now
                self._last_commanded_xyz = None
                self._last_commanded_ee_pos = None
                mode = "view"
                self._send_move_goal(VIEW_POSE_RAD, perf=perf, mode=mode)
                command_sent = True
            elif (
                not self._view_goal_reached
                and now - self._last_view_cmd_time >= VIEW_RETRY_INTERVAL_SEC
            ):
                self._warn("View 位姿未完成，重试发送回位目标")
                self._last_view_cmd_time = now
                mode = "view"
                self._send_move_goal(VIEW_POSE_RAD, perf=perf, mode=mode)
                command_sent = True
            perf.record("total_follow", "start")
            perf.flush(
                extra={
                    "step_id": self._motion_step_id,
                    "mode": mode,
                    "command_sent": command_sent,
                    "target_visible": False,
                    "elapsed_since_last_target": elapsed,
                }
            )
            return

        target_x, target_y, target_z, dx, dy, dz = (
            target["bbox"][0],
            target["bbox"][1],
            target["bbox"][2],
            target["bbox"][3],
            target["bbox"][4],
            target["bbox"][5],
        )

        t_pose_compute_start = perf.mark("target_pose_compute_start")
        target_pose = self._compute_target_pose(target_x, target_y, target_z, dx, dy, dz)
        perf.record("target_pose_compute", t_pose_compute_start)

        if self._last_commanded_xyz is not None and target_pose is not None:
            dx_disp = target_x - self._last_commanded_xyz[0]
            dy_disp = target_y - self._last_commanded_xyz[1]
            dz_disp = target_z - self._last_commanded_xyz[2]
            obj_disp = math.sqrt(dx_disp**2 + dy_disp**2 + dz_disp**2)
            ee_disp = np.linalg.norm(target_pose[0] - self._last_commanded_ee_pos) if self._last_commanded_ee_pos is not None else float("inf")
            if obj_disp < self._DISPLACEMENT_THRESHOLD and ee_disp < POSE_DISPLACEMENT_THRESHOLD:
                perf.record("total_follow", "start")
                perf.flush(
                    extra={
                        "step_id": self._motion_step_id,
                        "mode": mode,
                        "command_sent": command_sent,
                        "target_displacement": obj_disp,
                    }
                )
                return

        self._info(f"目标位置: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})")
        self._publish_follow_target_pose(target_x, target_y, target_z)

        if target_pose is not None:
            pos, quat = target_pose
            if self._last_commanded_ee_pos is not None:
                ee_disp = np.linalg.norm(pos - self._last_commanded_ee_pos)
                if ee_disp < POSE_DISPLACEMENT_THRESHOLD:
                    perf.record("total_follow", "start")
                    perf.flush(
                        extra={
                            "step_id": self._motion_step_id,
                            "mode": mode,
                            "command_sent": command_sent,
                            "ee_displacement": float(ee_disp),
                        }
                    )
                    return
            self._info(f"跟随位姿: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
            mode = "pose"
            self._send_reach_goal(pos, quat, perf=perf, mode=mode)
            command_sent = True
            self._last_commanded_ee_pos = pos.copy()
        else:
            t_joint_compute_start = perf.mark("joint_compute_start")
            joints = self._compute_follow_joints(target_x, target_y)
            perf.record("joint_compute", t_joint_compute_start)
            if joints is None:
                perf.record("total_follow", "start")
                perf.flush(
                    extra={
                        "step_id": self._motion_step_id,
                        "mode": mode,
                        "command_sent": command_sent,
                        "joint_compute_ok": False,
                    }
                )
                return
            yaw_deg = math.degrees(joints[0])
            self._info(f"跟随(降级): joint1={yaw_deg:.1f}°")
            mode = "joint"
            self._send_move_goal(joints, perf=perf, mode=mode)
            command_sent = True
            self._last_commanded_ee_pos = None

        self._last_commanded_xyz = (target_x, target_y, target_z)
        perf.record("total_follow", "start")
        perf.flush(
            extra={
                "step_id": self._motion_step_id,
                "mode": mode,
                "command_sent": command_sent,
                "selected_track_id": int(self.selected_track_id),
            }
        )


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
