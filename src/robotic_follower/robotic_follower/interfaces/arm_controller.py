"""机械臂运动控制接口。

通过 MoveGroup action 控制机械臂运动到指定关节角度。
"""

import json
import math
import os
import subprocess
import time
from collections.abc import Callable

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_srvs.srv import SetBool


def _load_calibration_poses_from_targets() -> list:
    """从 targets.json 加载 p1~p15 标定位姿。

    Returns:
        标定位姿列表，每个元素为 6 个关节角度（度）的列表
    """
    # targets.json 位于 ros2_dummy_arm_810 包中
    package_share = os.path.expanduser("~/ros2_ws/src/ros2_dummy_arm_810")
    targets_path = os.path.join(package_share, "targets.json")

    if not os.path.exists(targets_path):
        raise FileNotFoundError(f"targets.json 不存在: {targets_path}")

    with open(targets_path) as f:
        targets = json.load(f)

    poses = []
    for i in range(1, 16):  # p1 ~ p15
        key = f"p{i}"
        if key not in targets:
            raise ValueError(f"targets.json 中未找到 {key}")

        pose_data = targets[key]
        unit = pose_data.get("unit", "deg")
        joints = pose_data.get("joints", [])

        if len(joints) != 6:
            raise ValueError(f"{key} joints 长度不为 6: {joints}")

        if unit == "rad":
            joints = [math.degrees(j) for j in joints]
        elif unit != "deg":
            raise ValueError(f"{key} unit 必须为 'deg' 或 'rad'，当前为: {unit}")

        poses.append(joints)

    return poses


def get_calibration_poses() -> list:
    """延迟加载标定位姿（避免模块级副作用）。

    Returns:
        标定位姿列表，每个元素为 6 个关节角度（度）的列表
    """
    return _load_calibration_poses_from_targets()


# 保留向后兼容的常量，但使用延迟加载
class _LazyCalibrationPoses:
    """延迟加载的标定位姿。"""

    _poses = None

    def __iter__(self):
        if self._poses is None:
            self._poses = _load_calibration_poses_from_targets()
        return iter(self._poses)

    def __len__(self):
        if self._poses is None:
            self._poses = _load_calibration_poses_from_targets()
        return len(self._poses)


CALIBRATION_POSES_DEG = _LazyCalibrationPoses()


class ArmController:
    """机械臂运动控制器。

    通过 MoveGroup action 发送关节目标，控制机械臂运动。

    Attributes:
        node: ROS2 节点
        move_group_client: MoveGroup action 客户端
        current_joint_positions: 当前关节角度列表
    """

    def __init__(self, node: Node) -> None:
        """初始化控制器。

        Args:
            node: ROS2 节点
        """
        self.node = node
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

        # 订阅关节状态
        self.joint_state_sub = node.create_subscription(
            JointState, "joint_states", self._joint_state_callback, 10
        )

        # MoveGroup action 客户端
        self.move_group_client = ActionClient(node, MoveGroup, "/move_action")

        # 机械臂使能服务客户端
        self.robot_enable_client = node.create_client(SetBool, "dummy_arm/enable")

        # 夹爪使能服务客户端（用于在使能机械臂后取消夹爪使能）
        self.gripper_enable_client = node.create_client(
            SetBool, "dummy_arm/gripper_enable"
        )

        # 夹爪打开服务客户端
        self.gripper_open_client = node.create_client(SetBool, "dummy_arm/gripper_open")

        # 等待服务
        self.node.get_logger().info("等待 MoveGroup action 服务...")
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.node.get_logger().error("MoveGroup action 服务不可用")
        else:
            self.node.get_logger().info("MoveGroup action 服务已连接")

        self.current_joint_positions = None

        # 添加地面障碍物
        self.run_add_obstacles()

    def run_add_obstacles(self) -> None:
        """添加地面障碍物到 MoveIt planning scene。

        地面以世界原点为中心，边长5m的矩形，厚度0.01m。
        """
        try:
            self.node.get_logger().info("正在添加地面障碍物...")

            ws_path = os.path.expanduser("~/ros2_ws")
            script_path = os.path.join(
                os.path.dirname(__file__), "../script/add_ground_obstacle.py"
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
                self.node.get_logger().warning(
                    f"添加地面障碍物脚本执行失败: {result.stderr}"
                )
            else:
                self.node.get_logger().info("地面障碍物添加完成")

        except subprocess.TimeoutExpired:
            self.node.get_logger().warning("添加地面障碍物超时")
        except FileNotFoundError as e:
            self.node.get_logger().warning(f"脚本文件不存在: {e}")
        except OSError as e:
            self.node.get_logger().warning(f"系统错误: {e}")

    def _joint_state_callback(self, msg: JointState) -> None:
        """接收关节状态更新。

        Args:
            msg: JointState 消息
        """
        if len(msg.position) >= 6:
            self.current_joint_positions = list(msg.position[:6])

    def wait_for_joint_state(self, timeout: float = 5.0) -> bool:
        """等待获取关节状态。

        Args:
            timeout: 超时时间（秒）

        Returns:
            True if 获取成功
        """
        start = time.time()
        while self.current_joint_positions is None:
            if time.time() - start > timeout:
                return False
            time.sleep(0.1)
        return True

    def enable_robot(self, timeout: float = 10.0) -> bool:
        """使能机械臂。

        Args:
            timeout: 超时时间（秒）

        Returns:
            True if 使能成功
        """
        self.node.get_logger().info("请求使能机械臂...")
        if not self.robot_enable_client.wait_for_service(timeout_sec=timeout):
            self.node.get_logger().error("机械臂使能服务不可用")
            return False

        request = SetBool.Request()
        request.data = True
        future = self.robot_enable_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)

        if future.result() is None:
            self.node.get_logger().error("机械臂使能请求超时")
            return False

        response = future.result()
        if response.success:
            self.node.get_logger().info("机械臂使能成功")
            # 显式关闭夹爪使能并打开夹爪，避免固件联动使能夹爪
            try:
                # 取消夹爪使能
                if self.gripper_enable_client.wait_for_service(timeout_sec=2.0):
                    gripper_req = SetBool.Request()
                    gripper_req.data = False
                    gripper_future = self.gripper_enable_client.call_async(gripper_req)
                    rclpy.spin_until_future_complete(
                        self.node, gripper_future, timeout_sec=2.0
                    )
                # 打开夹爪
                if self.gripper_open_client.wait_for_service(timeout_sec=2.0):
                    open_req = SetBool.Request()
                    open_req.data = True
                    open_future = self.gripper_open_client.call_async(open_req)
                    rclpy.spin_until_future_complete(
                        self.node, open_future, timeout_sec=2.0
                    )
                    self.node.get_logger().info("夹爪已打开")
            except RuntimeError as e:
                self.node.get_logger().warning(f"夹爪控制运行时错误（已忽略）: {e}")
            except OSError as e:
                self.node.get_logger().warning(f"夹爪控制系统错误（已忽略）: {e}")
            return True

        self.node.get_logger().error(f"机械臂使能失败: {response.message}")
        return False

    def move_to_joints_deg(self, joints_deg: list, wait: bool = True) -> bool:
        """移动机械臂到指定关节角度（度）。

        Args:
            joints_deg: 6 个关节角度（度）
            wait: 是否等待运动完成

        Returns:
            True if 运动成功
        """
        joints_rad = [math.radians(d) for d in joints_deg]
        return self.move_to_joints_rad(joints_rad, wait)

    def move_to_joints_rad(self, joints_rad: list, wait: bool = True) -> bool:
        """移动机械臂到指定关节角度（弧度）。

        Args:
            joints_rad: 6 个关节角度（弧度）
            wait: 是否等待运动完成

        Returns:
            True if 运动成功
        """
        goal = self._create_move_group_goal(joints_rad)
        goal_handle = self._send_goal_and_wait(goal)
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error("目标被拒绝")
            return False

        if wait:
            return self._wait_for_result(goal_handle)

        return True

    def _create_move_group_goal(self, target_positions: list) -> MoveGroup.Goal:
        """创建 MoveGroup goal。

        Args:
            target_positions: 目标关节角度（弧度）

        Returns:
            MoveGroup.Goal
        """
        goal = MoveGroup.Goal()
        motion_plan_request = MotionPlanRequest()

        motion_plan_request.group_name = "dummy_arm"
        motion_plan_request.planner_id = "RRTConnectkConfigDefault"
        motion_plan_request.num_planning_attempts = 10
        motion_plan_request.allowed_planning_time = 5.0
        motion_plan_request.max_velocity_scaling_factor = 0.3
        motion_plan_request.max_acceleration_scaling_factor = 0.3

        # 起始状态
        motion_plan_request.start_state.joint_state.header = Header()
        motion_plan_request.start_state.joint_state.header.stamp = (
            self.node.get_clock().now().to_msg()
        )
        motion_plan_request.start_state.joint_state.name = self.joint_names
        motion_plan_request.start_state.joint_state.position = (
            self.current_joint_positions or [0.0] * 6
        )

        # 目标约束
        joint_constraints = []
        for name, position in zip(self.joint_names, target_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = position
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            joint_constraints.append(jc)

        goal_constraints = Constraints()
        goal_constraints.joint_constraints = joint_constraints
        motion_plan_request.goal_constraints = [goal_constraints]

        goal.request = motion_plan_request
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 10

        return goal

    def _send_goal_and_wait(self, goal) -> "MoveGroup.GoalHandle | None":
        """发送 goal 并等待接受。

        Args:
            goal: MoveGroup.Goal

        Returns:
            GoalHandle 或 None
        """
        self.node.get_logger().info("发送运动目标...")
        send_future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=5.0)

        if send_future.result() is None:
            self.node.get_logger().error("目标发送超时")
            return None

        return send_future.result()

    def _wait_for_result(self, goal_handle) -> bool:
        """等待运动结果。

        Args:
            goal_handle: GoalHandle

        Returns:
            True if 运动成功
        """
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, get_result_future, timeout_sec=60.0)

        if get_result_future.result() is None:
            self.node.get_logger().error("运动执行超时")
            return False

        result = get_result_future.result()
        if result.result.error_code.val == 1:  # SUCCESS
            self.node.get_logger().info("运动执行成功")
            return True

        self.node.get_logger().error(
            f"运动执行失败，错误码: {result.result.error_code.val}"
        )
        return False

    def execute_calibration_poses(
        self,
        on_pose_reached: Callable[[int, list], None] | None = None,
        stable_wait: float = 1.0,
    ) -> int:
        """执行预定义的标定位姿序列。

        依次移动到预定义位姿，到达后调用回调函数。

        Args:
            on_pose_reached: 到达每个位姿后的回调函数，签名为 (pose_index, joints_deg)
            stable_wait: 到达后等待稳定的时间（秒）

        Returns:
            成功到达的位姿数量
        """
        # 延迟加载标定位姿
        poses = get_calibration_poses()
        success_count = 0

        for i, joints_deg in enumerate(poses):
            self.node.get_logger().info(
                f"移动到标定位姿 {i + 1}/{len(poses)}: {joints_deg}"
            )

            if self.move_to_joints_deg(joints_deg, wait=True):
                success_count += 1
                time.sleep(stable_wait)

                if on_pose_reached is not None:
                    on_pose_reached(i, joints_deg)
            else:
                self.node.get_logger().warn(f"位姿 {i + 1} 运动失败，跳过")

        return success_count
