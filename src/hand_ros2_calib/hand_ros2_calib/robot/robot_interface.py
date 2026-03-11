"""
机器人接口

提供与 MoveIt 集成的机器人接口，用于获取位姿和控制运动。
"""

import math
import numpy as np
from typing import Optional, Tuple, List
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from coordinate_transform.pose_utils import PoseCalculator


class RobotInterface:
    """
    机器人接口

    功能：
    - 获取当前机械臂位姿
    - 控制机械臂运动
    - 与 MoveIt 集成（可选）
    - 提供位姿查询服务
    """

    def __init__(
        self,
        node: Node,
        moveit_group: str = "arm",
        base_frame: str = "base_link",
        end_effector_frame: str = "link6_1_1",
    ):
        """
        初始化机器人接口

        Args:
            node: ROS2 节点
            moveit_group: MoveIt 规划组名称
            base_frame: 基座坐标系
            end_effector_frame: 末端执行器坐标系
        """
        self.node = node
        self.moveit_group = moveit_group
        self.base_frame = base_frame
        self.end_effector_frame = end_effector_frame

        # TF 监听器
        self._tf_buffer = None
        self._tf_listener = None
        self._setup_tf_listener()

        # 关节轨迹发布器（直接控制）
        self.joint_trajectory_pub = self.node.create_publisher(
            JointTrajectory,
            f"/{moveit_group}/joint_trajectory",
            10,
        )

        # 当前位姿缓存
        self.current_pose: Optional[Pose] = None
        self.current_joints: List[float] = []

        # 订阅关节状态
        self.joint_state_sub = None

        self.node.get_logger().info(
            f"机器人接口已初始化: {base_frame} -> {end_effector_frame}"
        )

    def _setup_tf_listener(self):
        """设置 TF 监听器"""
        from tf2_ros import Buffer, TransformListener

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self.node)

    # ==================== 位姿获取 ====================

    def get_current_pose(self, timeout: float = 1.0) -> Optional[Pose]:
        """
        获取当前末端执行器位姿（在基座坐标系中）

        Args:
            timeout: TF 超时时间

        Returns:
            Optional[Pose]: 当前位姿，失败返回 None
        """
        try:
            transform = self._tf_buffer.lookup_transform(
                self.base_frame,
                self.end_effector_frame,
                rclpy.time.Time(),
                timeout=r=rclpy.duration.Duration(seconds=timeout)
            )

            pose = Pose(
                position=Point(
                    x=transform.transform.translation.x,
                    y=transform.transform.translation.y,
                    z=transform.transform.translation.z,
                ),
                orientation=transform.transform.rotation,
            )

            self.current_pose = pose
            return pose

        except Exception as e:
            self.node.get_logger().error(f"获取位姿失败: {e}")
            return None

    def get_current_pose_tuple(
        self,
        timeout: float = 1.0,
    ) -> Optional[Tuple[float, float, float, float, float, float]]:
        """
        获取当前末端执行器位姿（元组形式）

        Args:
            timeout: TF 超时时间

        Returns:
            Optional[Tuple[float, ...]]: (x, y, z, rx, ry, rz) 米和度
        """
        pose = self.get_current_pose(timeout)

        if pose is None:
            return None

        # 转换为 mm 和度
        position_mm = (
            pose.position.x * 1000.0,
            pose.position.y * 1000.0,
            pose.position.z * 1000.0,
        )

        # 四元数转欧拉角（ZYX 顺序）
        euler = self._quaternion_to_euler(pose.orientation)
        rotation_deg = (
            math.degrees(euler[0]),
            math.degrees(euler[1]),
            math.degrees(euler[2]),
        )

        return position_mm + rotation_deg

    @staticmethod
    def _quaternion_to_euler(q: Quaternion) -> Tuple[float, float, float]:
        """四元数转欧拉角（ZYX 顺序）"""
        w, x, y, z = q.w, q.x, q.y, q.z

        # Roll (X-axis)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (Y-axis)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (Z-axis)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return (roll, pitch, yaw)

    # ==================== 运动控制 ====================

    def move_to_pose(
        self,
        target_pose: Pose,
        planning_time: float = 5.0,
    ) -> bool:
        """
        移动到目标位姿（需要 MoveIt）

        Args:
            target_pose: 目标位姿
            planning_time: 规划时间（秒）

        Returns:
            bool: 是否成功运动
        """
        # TODO: 实现 MoveIt 运动规划
        # 这里需要集成 pymoveit2 或调用 MoveIt2 action

        self.node.get_logger().warn(
            "move_to_pose 需要实现 MoveIt 集成"
        )
        return False

    def move_joint_trajectory(
        self,
        joint_positions: List[float],
        duration: float = 2.0,
    ) -> bool:
        """
        按关节轨迹运动

        Args:
            joint_positions: 关节位置列表
            duration: 运动时间（秒）

        Returns:
            bool: 是否成功
        """
        try:
            traj = JointTrajectory()
            traj.joint_names = [
                "joint1",
                "joint2",
                "joint3",
                "joint4",
                "joint5",
                "joint6",
            ]

            point = JointTrajectoryPoint()
            point.positions = joint_positions
            point.time_from_start.sec = int(duration)
            point.time_from_start.nanosec = int((duration % 1.0) * 1e9)

            traj.points.append(point)

            # 发布轨迹
            self.joint_trajectory_pub.publish(traj)

            # 等待运动完成
            rclpy.spin_once(self.node, timeout_sec=duration)

            return True

        except Exception as e:
            self.node.get_logger().error(f"关节轨迹发送失败: {e}")
            return False

    # ==================== 位姿计算 ====================

    @staticmethod
    def calculate_relative_pose(
        base_pose: Pose,
        target_pose: Pose,
    ) -> Pose:
        """
        计算目标位姿相对于基准位姿的相对位姿

        Args:
            base_pose: 基准位姿
            target_pose: 目标位姿

        Returns:
            Pose: 相对位姿
        """
        return PoseCalculator.calculate_relative_pose(base_pose, target_pose)

    @staticmethod
    def apply_relative_pose(
        base_pose: Pose,
        relative_pose: Pose,
    ) -> Pose:
        """
        将相对位姿应用到基准位姿

        Args:
            base_pose: 基准位姿
            relative_pose: 相对位姿

        Returns:
            Pose: 结果位姿
        """
        return PoseCalculator.apply_relative_pose(base_pose, relative_pose)

    # ==================== 等待和验证 ====================

    def wait_for_robot_ready(self, timeout: float = 5.0) -> bool:
        """
        等待机器人准备就绪

        Args:
            timeout: 超时时间（秒）

        Returns:
            bool: 机器人是否就绪
        """
        start_time = self.node.get_clock().now()

        while (self.node.get_clock().now() - start_time).nanoseconds / 1e9 < timeout:
            pose = self.get_current_pose(timeout=0.5)
            if pose is not None:
                self.node.get_logger().info("机器人已就绪")
                return True

            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.get_logger().warn(f"等待机器人就绪超时（{timeout}秒）")
        return False

    def is_pose_reachable(
        self,
        target_pose: Pose,
    ) -> bool:
        """
        检查目标位姿是否可达（需要 IK）

        Args:
            target_pose: 目标位姿

        Returns:
            bool: 是否可达
        """
        # TODO: 实现 IK 求解
        return True
