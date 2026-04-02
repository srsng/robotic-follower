"""机械臂末端位姿接口。

通过 PyMoveIt2 获取机械臂末端执行器相对基座的实时位姿。
"""

import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState


class RobotPoseInterface:
    """机械臂末端位姿接口。

    使用 PyMoveIt2 获取末端执行器位姿，为手眼标定提供 robot_pose 数据。

    Attributes:
        moveit2: PyMoveIt2 实例
    """

    def __init__(self, moveit2) -> None:
        """初始化接口。

        Args:
            moveit2: 已初始化的 PyMoveIt2 实例
        """
        self.moveit2 = moveit2

    def get_current_pose(self) -> PoseStamped:
        """获取当前末端执行器位姿。

        Returns:
            geometry_msgs/PoseStamped，末端相对基座的位姿
        """
        return self.moveit2.compute_fk(joint_state=None, fk_link_names=["link6_1_1"])

    def get_pose_as_matrix(self) -> np.ndarray:
        """获取当前末端执行器位姿为 4x4 齐次变换矩阵。

        Returns:
            4x4 numpy 数组 (gripper2base)
        """
        pose_stamped = self.get_current_pose()
        pose = pose_stamped.pose

        matrix = np.eye(4)
        matrix[0, 3] = pose.position.x
        matrix[1, 3] = pose.position.y
        matrix[2, 3] = pose.position.z
        # 标准四元数转旋转矩阵公式
        matrix[0, 0] = 1 - 2 * (pose.orientation.y**2 + pose.orientation.z**2)
        matrix[0, 1] = 2 * (
            pose.orientation.x * pose.orientation.y
            - pose.orientation.z * pose.orientation.w
        )
        matrix[0, 2] = 2 * (
            pose.orientation.x * pose.orientation.z
            + pose.orientation.y * pose.orientation.w
        )
        matrix[1, 0] = 2 * (
            pose.orientation.x * pose.orientation.y
            + pose.orientation.z * pose.orientation.w
        )
        matrix[1, 1] = 1 - 2 * (pose.orientation.x**2 + pose.orientation.z**2)
        matrix[1, 2] = 2 * (
            pose.orientation.y * pose.orientation.z
            - pose.orientation.x * pose.orientation.w
        )
        matrix[2, 0] = 2 * (
            pose.orientation.x * pose.orientation.z
            - pose.orientation.y * pose.orientation.w
        )
        matrix[2, 1] = 2 * (
            pose.orientation.y * pose.orientation.z
            + pose.orientation.x * pose.orientation.w
        )
        matrix[2, 2] = 1 - 2 * (pose.orientation.x**2 + pose.orientation.y**2)

        return matrix

    def get_joint_states(self) -> JointState:
        """获取当前关节状态。

        Returns:
            sensor_msgs/JointState
        """
        return self.moveit2.joint_state
