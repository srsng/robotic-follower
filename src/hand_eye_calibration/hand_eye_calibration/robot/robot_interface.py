"""机器人接口。"""

import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import tf2_ros


class RobotInterface:
    """机器人接口。"""

    def __init__(
        self,
        node: Node,
        base_frame: str = "base_link",
        end_effector_frame: str = "end_effector"
    ):
        """初始化机器人接口。

        Args:
            node: ROS2节点
            base_frame: 基座坐标系
            end_effector_frame: 末端坐标系
        """
        self.node = node
        self.base_frame = base_frame
        self.end_effector_frame = end_effector_frame

        # 创建TF缓冲区和监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

        self.node.get_logger().info(
            f"机器人接口已初始化: {base_frame} -> {end_effector_frame}"
        )

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
        xx, yy, zz = x*x, y*y, z*z
        xy, xz, yz = x*y, x*z, y*z
        xw, yw, zw = x*w, y*w, z*w

        R = np.array([
            [1 - 2*(yy + zz), 2*(xy - zw), 2*(xz + yw)],
            [2*(xy + zw), 1 - 2*(xx + zz), 2*(yz - xw)],
            [2*(xz - yw), 2*(yz + xw), 1 - 2*(xx + yy)]
        ])

        return R

    def start(self):
        """启动机器人接口。"""
        self.node.get_logger().info("机器人接口已启动")

    def stop(self):
        """停止机器人接口。"""
        self.node.get_logger().info("机器人接口已停止")
