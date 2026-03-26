#!/usr/bin/env python3
"""机械臂控制节点：执行运动指令。

接收规划节点的轨迹指令，通过 MoveIt2 执行机械臂运动。
支持关节空间和任务空间两种控制方式。

订阅话题：
    - /robotic_follower/joint_states (sensor_msgs/JointState)
        真实关节状态（来自 remapper 节点）
    - /motion_plan/trajectory (trajectory_msgs/JointTrajectory)
        关节轨迹
    - /motion_plan/pose_goal (geometry_msgs/PoseStamped)
        目标位姿
    - /motion_plan/joint_goal (sensor_msgs/JointState)
        目标关节角度

发布话题：
    - /motion_plan/execution_status (std_msgs/String)
        执行状态
    - /robot/pose (geometry_msgs/PoseStamped)
        当前末端位姿（用于标定采样）
    - /joint_states (sensor_msgs/JointState)
        当前关节状态（relay from /robotic_follower/joint_states）

服务：
    - /arm/enable (std_srvs/SetBool)
        启用/禁用机械臂
    - /arm/home (std_srvs/Trigger)
        回零位
    - /arm/stop (std_srvs/Trigger)
        紧急停止

参数：
    - max_velocity (float, 默认: 0.3)
        最大速度比例
    - max_acceleration (float, 默认: 0.3)
        最大加速度比例
"""

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from pymoveit2 import MoveIt2
from pymoveit2.robots import dummy as robot
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory


class ArmControlNode(Node):
    """机械臂控制节点。"""

    def __init__(self):
        super().__init__("arm_control")

        # 运动参数
        self.declare_parameter("max_velocity", 0.3)
        self.declare_parameter("max_acceleration", 0.3)

        self.max_velocity = self.get_parameter("max_velocity").value
        self.max_acceleration = self.get_parameter("max_acceleration").value

        # 回调组
        callback_group = ReentrantCallbackGroup()

        # 关节名称常量 (使用 SRDF 中的实际名称，小写)
        self.joint_names = [
            "joint1", "joint2", "joint3",
            "joint4", "joint5", "joint6",
        ]
        self.base_link_name = robot.base_link_name()
        self.end_effector_name = robot.end_effector_name()

        # MoveIt2 接口
        self.moveit_interface = MoveIt2(
            node=self,
            joint_names=self.joint_names,
            base_link_name=self.base_link_name,
            end_effector_name=self.end_effector_name,
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )
        self.moveit_interface.max_velocity = self.max_velocity
        self.moveit_interface.max_acceleration = self.max_acceleration

        # TF 缓冲区
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 状态
        self.is_enabled = True
        self.is_executing = False
        self.current_joint_state: JointState | None = None

        # 订阅 - 真实关节状态
        self.joint_state_sub = self.create_subscription(
            JointState,
            "/robotic_follower/joint_states",
            self._joint_state_callback,
            10,
        )

        # 订阅 - 运动规划指令
        self.traj_sub = self.create_subscription(
            JointTrajectory,
            "/motion_plan/trajectory",
            self.trajectory_callback,
            10,
        )
        self.pose_goal_sub = self.create_subscription(
            PoseStamped,
            "/motion_plan/pose_goal",
            self.pose_goal_callback,
            10,
        )
        self.joint_goal_sub = self.create_subscription(
            JointState,
            "/motion_plan/joint_goal",
            self.joint_goal_callback,
            10,
        )

        # 发布
        self.status_pub = self.create_publisher(
            String,
            "/motion_plan/execution_status",
            10,
        )
        self.pose_pub = self.create_publisher(
            PoseStamped,
            "/robot/pose",
            10,
        )
        self.joint_pub = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )

        # 服务
        self.enable_srv = self.create_service(
            SetBool,
            "/arm/enable",
            self.enable_callback,
        )
        self.home_srv = self.create_service(
            Trigger,
            "/arm/home",
            self.home_callback,
        )
        self.stop_srv = self.create_service(
            Trigger,
            "/arm/stop",
            self.stop_callback,
        )

        # 定时发布当前状态
        self.state_timer = self.create_timer(0.1, self.publish_state)

        self.get_logger().info("机械臂控制节点已启动")

    def _joint_state_callback(self, msg: JointState) -> None:
        """处理真实关节状态。"""
        self.current_joint_state = msg

    def trajectory_callback(self, msg: JointTrajectory) -> None:
        """处理关节轨迹。"""
        if not self.is_enabled:
            self.get_logger().warn("机械臂未启用")
            return

        self.is_executing = True
        self.publish_status("executing")

        try:
            if len(msg.points) > 0:
                positions = msg.points[-1].positions
                if len(positions) == 6:
                    self.moveit_interface.move_to_configuration(list(positions))
                    self.moveit_interface.wait_until_executed()
                    self.publish_status("succeeded")
                else:
                    self.get_logger().warn(f"关节数不匹配: {len(positions)}")
                    self.publish_status("failed")
        except Exception as e:
            self.get_logger().error(f"轨迹执行失败: {e}")
            self.publish_status("failed")
        finally:
            self.is_executing = False

    def pose_goal_callback(self, msg: PoseStamped) -> None:
        """处理目标位姿。"""
        if not self.is_enabled:
            self.get_logger().warn("机械臂未启用")
            return

        self.is_executing = True
        self.publish_status("executing")

        try:
            position = [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ]
            orientation = [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]

            self.moveit_interface.move_to_pose(
                position=position, quat_xyzw=orientation
            )
            self.moveit_interface.wait_until_executed()
            self.publish_status("succeeded")
        except Exception as e:
            self.get_logger().error(f"位姿移动失败: {e}")
            self.publish_status("failed")
        finally:
            self.is_executing = False

    def joint_goal_callback(self, msg: JointState) -> None:
        """处理目标关节角度。"""
        if not self.is_enabled:
            self.get_logger().warn("机械臂未启用")
            return

        if len(msg.position) > 0:
            try:
                self.moveit_interface.move_to_configuration(list(msg.position))
                self.moveit_interface.wait_until_executed()
                self.publish_status("succeeded")
            except Exception as e:
                self.get_logger().error(f"关节控制失败: {e}")
                self.publish_status("failed")

    def publish_status(self, status: str) -> None:
        """发布执行状态。"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def publish_state(self) -> None:
        """发布当前状态。"""
        # 发布当前末端位姿（通过 TF 查询）
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_link_name,
                self.end_effector_name,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )

            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = self.base_link_name
            pose_msg.pose.position.x = transform.transform.translation.x
            pose_msg.pose.position.y = transform.transform.translation.y
            pose_msg.pose.position.z = transform.transform.translation.z
            pose_msg.pose.orientation.x = transform.transform.rotation.x
            pose_msg.pose.orientation.y = transform.transform.rotation.y
            pose_msg.pose.orientation.z = transform.transform.rotation.z
            pose_msg.pose.orientation.w = transform.transform.rotation.w
            self.pose_pub.publish(pose_msg)
        except Exception:
            pass

        # Relay 真实关节状态
        if self.current_joint_state is not None:
            self.joint_pub.publish(self.current_joint_state)

    def enable_callback(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
        """启用/禁用机械臂。"""
        self.is_enabled = request.data
        response.success = True
        response.message = "启用" if self.is_enabled else "禁用"
        self.get_logger().info(f"机械臂已{response.message}")
        return response

    def home_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """回零位。"""
        try:
            # 回零位: [0, -1.2589, 1.5707, 0, 0, 0]
            home_positions = [0.0, -1.2589, 1.5707, 0.0, 0.0, 0.0]
            self.moveit_interface.move_to_configuration(home_positions)
            self.moveit_interface.wait_until_executed()
            response.success = True
            response.message = "已回零位"
        except Exception as e:
            response.success = False
            response.message = f"回零位异常: {e}"
        return response

    def stop_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """紧急停止。"""
        self.is_executing = False
        response.success = True
        response.message = "已停止"
        self.get_logger().warn("机械臂已停止")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ArmControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到中断信号")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
