"""机械臂运动控制接口。

通过 MoveGroup action 控制机械臂运动到指定关节角度。
"""

import math
import time
from typing import Callable

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    WorkspaceParameters,
    Constraints,
    JointConstraint,
    RobotState,
)
from moveit_msgs.srv import GetMotionPlan
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header


# 预定义的标定位姿序列（角度，度）
CALIBRATION_POSES_DEG = [
    [0, 0, 0, 0, 0, 0],          # home
    [-18.90, -3.20, 41.33, -159.28, -39.21, -166.98],  # target1
    [0, -36.04, -21.09, 0, -89.63, 0],  # view
    [-12.09, -27.57, 57.24, -144.14, -35.39, -152.68],  # pre
    [-68.25, -14.88, 66.72, -71.77, 76.99, -36.87],  # escape
    [-33.87, 14.76, 36.59, -39.87, 59.11, -24.48],  # output
    [-147.77, -19.41, -8.12, 159.16, 83.79, 151.15],  # preget
    [-164.47, -34.70, 36.44, 168.89, 49.07, 159.27],  # get
    [123.36, -27.37, 72.08, -93.87, 90.00, -44.41],   # put
    [0, -60, -30, 0, -90, 0],      # 额外位姿1
    [-45, -30, 45, -120, -45, -90],  # 额外位姿2
    [45, -20, 30, -60, 30, -45],   # 额外位姿3
    [-90, -45, 60, -90, 60, -135], # 额外位姿4
    [30, -45, -30, 90, -60, 45],   # 额外位姿5
    [-60, -20, 20, -45, 45, -60],  # 额外位姿6
    [0, -90, 0, 0, -90, 0],        # 额外位姿7
    [60, -30, 45, -150, 30, -30],  # 额外位姿8
    [-30, -60, 30, -30, 60, -90],  # 额外位姿9
    [0, -45, -45, 0, -90, 0],      # 额外位姿10
]


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
        self.joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']

        # 订阅关节状态
        self.joint_state_sub = node.create_subscription(
            JointState,
            'joint_states',
            self._joint_state_callback,
            10
        )

        # MoveGroup action 客户端
        self.move_group_client = ActionClient(node, MoveGroup, '/move_action')

        # 等待服务
        self.node.get_logger().info('等待 MoveGroup action 服务...')
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.node.get_logger().error('MoveGroup action 服务不可用')
        else:
            self.node.get_logger().info('MoveGroup action 服务已连接')

        self.current_joint_positions = None

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
            self.node.get_logger().error('目标被拒绝')
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
        motion_plan_request.start_state.joint_state.header.stamp = self.node.get_clock().now().to_msg()
        motion_plan_request.start_state.joint_state.name = self.joint_names
        motion_plan_request.start_state.joint_state.position = self.current_joint_positions or [0.0] * 6

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

    def _send_goal_and_wait(self, goal) -> None:
        """发送 goal 并等待接受。

        Args:
            goal: MoveGroup.Goal

        Returns:
            GoalHandle 或 None
        """
        self.node.get_logger().info('发送运动目标...')
        send_future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=5.0)

        if send_future.result() is None:
            self.node.get_logger().error('目标发送超时')
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
            self.node.get_logger().error('运动执行超时')
            return False

        result = get_result_future.result()
        if result.result.error_code.val == 1:  # SUCCESS
            self.node.get_logger().info('运动执行成功')
            return True

        self.node.get_logger().error(f'运动执行失败，错误码: {result.result.error_code.val}')
        return False

    def execute_calibration_poses(
        self,
        on_pose_reached: Callable[[int, list], None] | None = None,
        stable_wait: float = 1.0
    ) -> int:
        """执行预定义的标定位姿序列。

        依次移动到预定义位姿，到达后调用回调函数。

        Args:
            on_pose_reached: 到达每个位姿后的回调函数，签名为 (pose_index, joints_deg)
            stable_wait: 到达后等待稳定的时间（秒）

        Returns:
            成功到达的位姿数量
        """
        success_count = 0

        for i, joints_deg in enumerate(CALIBRATION_POSES_DEG):
            self.node.get_logger().info(f'移动到标定位姿 {i+1}/{len(CALIBRATION_POSES_DEG)}: {joints_deg}')

            if self.move_to_joints_deg(joints_deg, wait=True):
                success_count += 1
                time.sleep(stable_wait)

                if on_pose_reached is not None:
                    on_pose_reached(i, joints_deg)
            else:
                self.node.get_logger().warn(f'位姿 {i+1} 运动失败，跳过')

        return success_count
