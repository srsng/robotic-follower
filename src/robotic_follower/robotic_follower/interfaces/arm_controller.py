"""机械臂运动控制接口。

通过 MoveGroup action 控制机械臂运动到指定关节角度。
"""
# TODO: 调整文件到节点目录

import json
import math
import os
import subprocess
import time

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    DisplayTrajectory,
    JointConstraint,
    MotionPlanRequest,
)
from moveit_msgs.srv import GetMotionPlan
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_srvs.srv import SetBool

from robotic_follower.util.wrapper import NodeWrapper


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


class ArmMoveServerNode(NodeWrapper):
    """机械臂运动控制器

    通过 MoveGroup action 发送关节目标，控制机械臂运动。

    Attributes:
        move_group_client: MoveGroup action 客户端
        current_joint_positions: 当前关节角度列表
    """

    def __init__(self) -> None:
        """初始化控制器。

        Args:
            node: ROS2 节点
        """
        super().__init__(
            node_name="arm-move-server-node",
            parent_node=self,
        )

        # MoveIt规划服务客户端 - 这个会使用RViz中的障碍物
        self.plan_service = self.create_client(GetMotionPlan, "/plan_kinematic_path")

        # MoveGroup Action客户端 - 用于执行规划好的轨迹
        self.move_group_client = ActionClient(self, MoveGroup, "/move_action")

        # 夹爪控制服务客户端
        self.gripper_open_service = self.create_client(
            SetBool, "dummy_arm/gripper_open"
        )
        self.gripper_close_service = self.create_client(
            SetBool, "dummy_arm/gripper_close"
        )

        # 整机使能与夹爪使能服务客户端
        self.robot_enable_service = self.create_client(SetBool, "dummy_arm/enable")
        self.gripper_enable_service = self.create_client(
            SetBool, "dummy_arm/gripper_enable"
        )

        # 发布规划结果到RViz显示
        self.display_trajectory_publisher = self.create_publisher(
            DisplayTrajectory, "/move_group/display_planned_path", 10
        )

        # 订阅当前关节状态
        self.joint_state_subscriber = self.create_subscription(
            JointState, "joint_states", self._joint_state_callback, 10
        )

        self.current_joint_positions = None
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

        # 等待MoveIt服务连接
        if not self.wait_for_services():
            self._fatal("❌ MoveIt服务连接失败，请确认已启动demo_real_arm.launch.py")
            raise RuntimeError("MoveIt服务连接失败")

        # 添加地面障碍物
        self.run_add_obstacles()

    def wait_for_services(self, timeout=10.0):
        """等待MoveIt服务连接"""
        self._info("等待MoveIt服务连接...")
        if not self.plan_service.wait_for_service(timeout_sec=timeout):
            self._error("❌ MoveIt规划服务不可用")
            return False

        if not self.move_group_client.wait_for_server(timeout_sec=timeout):
            self._error("❌ MoveGroup Action服务不可用")
            return False

        if not self.gripper_open_service.wait_for_service(timeout_sec=timeout):
            self._error("❌ 夹爪打开服务不可用")
            return False

        if not self.gripper_close_service.wait_for_service(timeout_sec=timeout):
            self._error("❌ 夹爪关闭服务不可用")
            return False

        if not self.robot_enable_service.wait_for_service(timeout_sec=timeout):
            self._error("❌ 整机使能服务不可用")
            return False
        if not self.gripper_enable_service.wait_for_service(timeout_sec=timeout):
            self._error("❌ 夹爪使能服务不可用")
            return False

        self._info("✅ MoveIt服务连接成功")
        return True

    def _joint_state_callback(self, msg):
        """接收当前关节状态"""
        if len(msg.position) >= 6:
            self.current_joint_positions = list(msg.position[:6])

    def run_add_obstacles(self) -> None:
        """添加地面障碍物到 MoveIt planning scene。

        地面以世界原点为中心，边长5m的矩形，厚度0.01m。
        """
        try:
            self._info("正在添加地面障碍物...")

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
                self._warn(f"添加地面障碍物脚本执行失败: {result.stderr}")
            else:
                self._info("地面障碍物添加完成")

        except subprocess.TimeoutExpired:
            self._warn("添加地面障碍物超时")
        except FileNotFoundError as e:
            self._warn(f"脚本文件不存在: {e}")
        except OSError as e:
            self._warn(f"系统错误: {e}")

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

    def set_robot_enable(self, enable: bool):
        """整机使能/去使能"""
        try:
            request = SetBool.Request()
            request.data = bool(enable)
            action = "使能" if enable else "去使能"
            self._info(f"🟢 请求{action}机械臂...")
            future = self.robot_enable_service.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.result() is None:
                self._error(f"❌ 机械臂{action}请求超时")
                return False
            resp = future.result()
            if resp.success:  # type: ignore
                self._info(f"✅ 机械臂{action}成功")
                # 当机械臂被使能时，确保夹爪不会被联动使能/闭合
                if enable:
                    # 取消夹爪使能，并尝试保持夹爪为打开状态
                    try:
                        self.set_gripper_enable(False)
                        # 确保夹爪打开（某些底层在上电会默认闭合）
                        self.control_gripper(close_gripper=False)
                    except Exception as _:
                        pass
                return True
            self._error(f"❌ 机械臂{action}失败: {resp.message}")  # type: ignore
            return False
        except Exception as e:
            self._error(f"❌ 机械臂使能控制异常: {e}")
            return False

    def set_gripper_enable(self, enable: bool):
        """夹爪使能/去使能"""
        try:
            request = SetBool.Request()
            request.data = bool(enable)
            action = "使能" if enable else "去使能"
            self._info(f"🤏 请求{action}夹爪...")
            future = self.gripper_enable_service.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.result() is None:
                self._error(f"❌ 夹爪{action}请求超时")
                return False
            resp = future.result()
            if resp.success:  # type: ignore
                self._info(f"✅ 夹爪{action}成功")
                return True
            self._error(f"❌ 夹爪{action}失败: {resp.message}")  # type: ignore
            return False
        except Exception as e:
            self._error(f"❌ 夹爪使能控制异常: {e}")
            return False

    def control_gripper(self, close_gripper=True):
        """控制夹爪开关"""
        try:
            request = SetBool.Request()
            request.data = True  # 服务只需要触发，数据内容不重要

            action_name = "关闭" if close_gripper else "打开"
            service_client = (
                self.gripper_close_service
                if close_gripper
                else self.gripper_open_service
            )

            self._info(f"🤏 {action_name}夹爪...")

            future = service_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            if future.result() is None:
                self._error(f"❌ 夹爪{action_name}请求超时")
                return False

            response = future.result()
            if response.success:  # type: ignore
                self._info(f"✅ 夹爪{action_name}成功")
                return True

            self._error(f"❌ 夹爪{action_name}失败: {response.message}")  # type: ignore
            return False

        except Exception as e:
            self._error(f"❌ 夹爪控制异常: {e}")
            return False

    def get_current_joint_angles_degrees(self):
        """获取当前关节角度（度数）"""
        if self.current_joint_positions:
            return [math.degrees(j) for j in self.current_joint_positions]
        return None

    def move_to_joints_rad(self, joints_rad: list[float]):
        """规划并自动执行到指定关节角度（弧度）

        Args:
            joints_rad: 6 个关节角度（弧度）

        Returns:
            True if 运动成功
        """
        try:
            # 创建MoveGroup目标
            goal = self._create_move_group_goal(joints_rad)

            # 发送目标到MoveGroup
            self._info("发送目标到MoveGroup...")
            send_goal_future = self.move_group_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)

            if send_goal_future.result() is None:
                self._error("目标发送超时")
                return False

            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                self._error("目标被拒绝")
                return False

            self._info("✅ 目标已接受，开始规划和执行...")

            # 等待执行完成 - 增加超时时间
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=60.0)

            if get_result_future.result() is None:
                self._error("运动执行超时")
                return False

            result = get_result_future.result()

            if result.result.error_code.val == 1:
                self._info("规划和执行成功完成")
                return True
            else:  # noqa: RET505
                error_code = result.result.error_code.val
                self._error(f"执行失败，错误码: {error_code}")
                self._handle_execute_err_code(error_code)
                return False

        except Exception as e:
            self._error(f"规划执行异常: {e}")
            import traceback

            self._error(traceback.format_exc())
            return False

    def _handle_execute_err_code(self, error_code: int):
        if error_code == -1:
            self._error("可能原因：目标位置不可达或存在碰撞")
        elif error_code == -2:
            self._error("可能原因：规划超时")
        elif error_code == -3:
            self._error("可能原因：无效的机器人状态")

    def move_to_joints_deg(self, joints_deg: list) -> bool:
        """移动机械臂到指定关节角度（度）。

        Args:
            joints_deg: 6 个关节角度（度）

        Returns:
            True if 运动成功
        """
        joints_rad = [math.radians(d) for d in joints_deg]
        return self.move_to_joints_rad(joints_rad)

    def _create_move_group_goal(self, target_positions: list) -> MoveGroup.Goal:
        """创建 MoveGroup goal

        Args:
            target_positions: 目标关节角度（6D, 弧度）

        Returns:
            MoveGroup.Goal
        """
        goal = MoveGroup.Goal()

        # 创建规划请求
        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = "dummy_arm"

        # 设置起始状态
        motion_plan_request.start_state.joint_state.header = Header()
        motion_plan_request.start_state.joint_state.header.stamp = (
            self.get_clock().now().to_msg()
        )
        motion_plan_request.start_state.joint_state.name = self.joint_names
        motion_plan_request.start_state.joint_state.position = (
            self.current_joint_positions or [0.0] * 6
        )

        # 设置目标关节约束
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

        # 设置规划器参数
        motion_plan_request.planner_id = "RRTConnectkConfigDefault"
        motion_plan_request.num_planning_attempts = 10
        motion_plan_request.allowed_planning_time = 10.0  # 增加规划时间
        motion_plan_request.max_velocity_scaling_factor = 0.3  # 提高速度
        motion_plan_request.max_acceleration_scaling_factor = 0.3  # 提高加速度

        goal.request = motion_plan_request
        goal.planning_options.plan_only = False  # 规划并执行
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 10  # 增加重新规划次数

        return goal
