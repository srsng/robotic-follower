"""机器人接口。"""

import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from typing import Optional, List
import tf2_ros


class RobotInterface:
    """机器人接口（增强版，支持主动控制）。"""

    def __init__(
        self,
        node: Node,
        base_frame: str = "base_link",
        end_effector_frame: str = "link6_1_1",
        group_name: str = "dummy_arm",
        joint_names: List[str] | None = None
    ):
        """初始化机器人接口。

        Args:
            node: ROS2节点
            base_frame: 基座坐标系
            end_effector_frame: 末端坐标系
            group_name: MoveIt2规划组名称
            joint_names: 关节名称列表
        """
        self.node = node
        self.base_frame = base_frame
        self.end_effector_frame = end_effector_frame
        self.group_name = group_name
        self.joint_names = joint_names or ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']

        # 创建TF缓冲区和监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

        # MoveIt2 接口
        self.moveit_interface = None
        self._init_move_moveit()

        self.node.get_logger().info(
            f"机器人接口已初始化: {base_frame} -> {end_effector_frame}"
        )

    def _init_move_moveit(self):
        """初始化 MoveIt2 接口。"""
        try:
            from rclpy.callback_groups import ReentrantCallbackGroup
            from pymoveit2 import MoveIt2

            callback_group = ReentrantCallbackGroup()

            self.moveit_interface = MoveIt2(
                node=self.node,
                joint_names=self.joint_names,
                base_link_name=self.base_frame,
                end_effector_name=self.end_effector_frame,
                group_name=self.group_name,
                callback_group=callback_group,
            )

            # 设置保守的运动参数（标定场景）
            self.moveit_interface.max_velocity = 0.3
            self.moveit_interface.max_acceleration = 0.3

            self.node.get_logger().info('MoveIt2 接口初始化成功')

        except Exception as e:
            self.node.get_logger().warn(f'MoveIt2 初始化失败: {e}')
            self.moveit_interface = None

    def get_current_pose(self, timeout: float = 1.0) -> np.ndarray:
        """获取当前末端位姿。

        Args:
            timeout: TF查询超时时间（秒）

        Returns:
            4x4齐次变换矩阵

        Raises:
            TimeoutError: TF查询超时
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.end_effector_frame,
                self.node.get_clock().now().to_msg(),
                timeout=timeout
            )

            # 提取平移
            t = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ])

            # 提取旋转（四元数转旋转矩阵）
            q = transform.transform.rotation
            R = self._quaternion_to_matrix(q.x, q.y, q.z, q.w)

            # 构建4x4齐次变换矩阵
            pose = np.eye(4)
            pose[:3, :3] = R
            pose[:3, 3] = t

            return pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.node.get_logger().error(f"获取位姿失败: {e}")
            raise TimeoutError(f"获取位姿失败: {e}")

    def _quaternion_to_matrix(
        self,
        x: float,
        y: float,
        z: float,
        w: float
    ) -> np.ndarray:
        """四元数转旋转矩阵。

        Args:
            x, y, z, w: 四元数分量

        Returns:
            3x3旋转矩阵
        """
        # 归一化四元数
        norm = np.sqrt(x**2 + y**2 + z**2 + w**2)
        if norm == 0:
            return np.eye(3)
        x, y, z, w = x / norm, y / norm, z / norm, w / norm

        # 转换为旋转矩阵
        xx, yy, zz = x**2, y**2, z**2
        xy, xz, yz = x*y, x*z, y*z
        xw, yw, zw = x*w, y*w, z*w

        R = np.array([
            [1 - 2*(yy + zz), 2*(xy - zw), 2*(xz + yw)],
            [2*(xy + zw), 1 - 2*(xx + zz), 2*(yz - xw)],
            [2*(xz - yw), 2*(yz + xw), 1 - 2*(xx + yy)]
        ])

        return R

    def move_to_pose(
        self,
        position: List[float],
        orientation: List[float],
        wait: bool = True
    ) -> bool:
        """移动到目标位姿

        Args:
            position: [x, y, z]
            orientation: [x, y, z, w] 四元数
            wait: 是否等待执行完成

        Returns:
            是否成功
        """
        if self.moveit_interface is None:
            self.node.get_logger().warn('MoveIt2 未初始化')
            return False

        try:
            self.moveit_interface.move_to_pose(
                position=position,
                quat_xyzw=orientation
            )
            if wait:
                self.moveit_interface.wait_until_executed()
            return True
        except Exception as e:
            self.node.get_logger().error(f'移动失败: {e}')
            return False

    def move_to_joint_positions(
        self,
        positions: List[float],
        wait: bool = True
    ) -> bool:
        """移动到关节位置

        Args:
            positions: 关节角度列表
            wait: 是否等待执行完成

        Returns:
            是否成功
        """
        if self.moveit_interface is None:
            return False

        try:
            self.moveit_interface.move_to_configuration(positions)
            if wait:
                self.moveit_interface.wait_until_executed()
            return True
        except Exception as e:
            self.node.get_logger().error(f'关节控制失败: {e}')
            return False

    def start(self):
        """启动机器人接口。"""
        self.node.get_logger().info("机器人接口已启动")

    def stop(self):
        """停止机器人接口。"""
        self.node.get_logger().info("机器人接口已停止")
