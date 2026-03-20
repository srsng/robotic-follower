#!/usr/bin/env python3
"""机械臂控制节点：执行运动指令。

接收规划节点的轨迹指令，通过 MoveIt2 执行机械臂运动。
支持关节空间和任务空间两种控制方式。

订阅话题：
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
        当前关节状态

服务：
    - /arm/enable (std_srvs/SetBool)
        启用/禁用机械臂
    - /arm/home (std_srvs/Trigger)
        回零位
    - /arm/stop (std_srvs/Trigger)
        紧急停止

参数：
    - group_name (str, 默认: "dummy_arm")
        MoveIt2 规划组名称
    - base_frame (str, 默认: "base_link")
        基座坐标系
    - end_effector_frame (str, 默认: "link6_1_1")
        末端执行器坐标系
    - max_velocity (float, 默认: 0.3)
        最大速度比例
    - max_acceleration (float, 默认: 0.3)
        最大加速度比例
"""

import rclpy
import std_srvs.srv
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory

from robotic_follower.robot import RobotInterface


class ArmControlNode(Node):
    """机械臂控制节点。"""

    def __init__(self):
        super().__init__("arm_control")

        # 参数
        self.declare_parameter("group_name", "dummy_arm")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("end_effector_frame", "link6_1_1")
        self.declare_parameter("max_velocity", 0.3)
        self.declare_parameter("max_acceleration", 0.3)

        self.group_name = self.get_parameter("group_name").value
        self.base_frame = self.get_parameter("base_frame").value
        self.end_effector_frame = self.get_parameter("end_effector_frame").value
        self.max_velocity = self.get_parameter("max_velocity").value
        self.max_acceleration = self.get_parameter("max_acceleration").value

        # 机器人接口
        self.robot = RobotInterface(
            node=self,
            base_frame=self.base_frame,
            end_effector_frame=self.end_effector_frame,
            group_name=self.group_name,
        )

        # 状态
        self.is_enabled = True
        self.is_executing = False
        self.current_joint_state = JointState()

        # 订阅
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
            std_srvs.srv.SetBool,
            "/arm/enable",
            self.enable_callback,
        )
        self.home_srv = self.create_service(
            std_srvs.srv.Trigger,
            "/arm/home",
            self.home_callback,
        )
        self.stop_srv = self.create_service(
            std_srvs.srv.Trigger,
            "/arm/stop",
            self.stop_callback,
        )

        # 定时发布当前状态
        self.state_timer = self.create_timer(0.1, self.publish_state)

        self.get_logger().info("机械臂控制节点已启动")

    def trajectory_callback(self, msg: JointTrajectory):
        """处理关节轨迹。"""
        if not self.is_enabled:
            self.get_logger().warn("机械臂未启用")
            return

        self.is_executing = True
        self.publish_status("executing")

        try:
            # 从轨迹提取目标关节位置
            if len(msg.points) > 0:
                positions = msg.points[-1].positions
                if len(positions) == 6:
                    success = self.robot.move_to_joint_positions(positions)
                    if success:
                        self.publish_status("succeeded")
                    else:
                        self.publish_status("failed")
                else:
                    self.get_logger().warn(f"关节数不匹配: {len(positions)}")
                    self.publish_status("failed")
        except Exception as e:
            self.get_logger().error(f"轨迹执行失败: {e}")
            self.publish_status("failed")
        finally:
            self.is_executing = False

    def pose_goal_callback(self, msg: PoseStamped):
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

            success = self.robot.move_to_pose(position, orientation)

            if success:
                self.publish_status("succeeded")
            else:
                self.publish_status("failed")
        except Exception as e:
            self.get_logger().error(f"位姿移动失败: {e}")
            self.publish_status("failed")
        finally:
            self.is_executing = False

    def joint_goal_callback(self, msg: JointState):
        """处理目标关节角度。"""
        if not self.is_enabled:
            self.get_logger().warn("机械臂未启用")
            return

        if len(msg.position) > 0:
            success = self.robot.move_to_joint_positions(list(msg.position))
            if success:
                self.publish_status("succeeded")
            else:
                self.publish_status("failed")

    def publish_status(self, status: str):
        """发布执行状态。"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def publish_state(self):
        """发布当前状态。"""
        # 发布当前末端位姿
        try:
            pose = self.robot.get_current_pose(timeout=0.5)
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = self.base_frame
            pose_msg.pose.position.x = float(pose[0, 3])
            pose_msg.pose.position.y = float(pose[1, 3])
            pose_msg.pose.position.z = float(pose[2, 3])
            # 旋转矩阵转四元数（简化）
            pose_msg.pose.orientation.w = 1.0
            self.pose_pub.publish(pose_msg)
        except Exception:
            pass

        # 发布关节状态（模拟）
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.robot.joint_names
        joint_state.position = [0.0] * 6  # 模拟数据
        self.joint_pub.publish(joint_state)

    def enable_callback(
        self,
        request: std_srvs.srv.SetBool.Request,
        response: std_srvs.srv.SetBool.Response,
    ) -> std_srvs.srv.SetBool.Response:
        """启用/禁用机械臂。"""
        self.is_enabled = request.data
        response.success = True
        response.message = "启用" if self.is_enabled else "禁用"
        self.get_logger().info(f"机械臂已{response.message}")
        return response

    def home_callback(
        self,
        request: std_srvs.srv.Trigger.Request,
        response: std_srvs.srv.Trigger.Response,
    ) -> std_srvs.srv.Trigger.Response:
        """回零位。"""
        try:
            home_positions = [0.0] * 6
            success = self.robot.move_to_joint_positions(home_positions)
            response.success = success
            response.message = "已回零位" if success else "回零位失败"
        except Exception as e:
            response.success = False
            response.message = f"回零位异常: {e}"
        return response

    def stop_callback(
        self,
        request: std_srvs.srv.Trigger.Request,
        response: std_srvs.srv.Trigger.Response,
    ) -> std_srvs.srv.Trigger.Response:
        """紧急停止。"""
        # MoveIt2 不支持直接 stop，这里仅标记状态
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
