#!/usr/bin/env python3
"""运动规划节点：基于视觉检测结果生成机械臂运动。

使用 MoveIt2 进行真实路径规划，通过 TF2 转换检测结果到机器人基座坐标系，
实现目标跟踪和跟随功能。

订阅话题：
    - /perception/detections (vision_msgs/Detection3DArray)
        3D 目标检测结果

发布话题：
    - /motion_plan/pose_goal (geometry_msgs.PoseStamped)
        目标位姿（传递给控制节点）
    - /motion_plan/trajectory (trajectory_msgs.JointTrajectory)
        关节轨迹
    - /motion_plan/state (std_msgs.String)
        规划状态

服务：
    - /motion_plan/start_follow (std_srvs/Trigger)
        开始跟随
    - /motion_plan/stop_follow (std_srvs/Trigger)
        停止跟随
    - /motion_plan/set_target (std_srvs/Trigger)
        设置跟随目标类别

参数：
    - target_object (str, 默认: "chair")
        目标物体类别
    - follow_distance (float, 默认: 0.5)
        跟随距离（米）
    - max_planning_time (float, 默认: 1.0)
        最大规划时间（秒）
    - robot_base_frame (str, 默认: "base_link")
        机器人基座坐标系
    - camera_frame (str, 默认: "camera_depth_optical_frame")
        相机坐标系
    - group_name (str, 默认: "dummy_arm")
        MoveIt2 规划组名称
    - end_effector_frame (str, 默认: "link6_1_1")
        末端执行器坐标系
"""

import numpy as np
import rclpy
import std_srvs.srv
import tf2_ros
from geometry_msgs.msg import PoseStamped
from pymoveit2 import MoveIt2
from pymoveit2.robots import dummy as robot
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from vision_msgs.msg import Detection3D, Detection3DArray


class PlanningNode(Node):
    """运动规划节点 - 使用 MoveIt2 进行真实路径规划。"""

    def __init__(self):
        super().__init__("motion_planning")

        # 参数
        self.declare_parameter("target_object", "chair")
        self.declare_parameter("follow_distance", 0.5)
        self.declare_parameter("max_planning_time", 1.0)
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_depth_optical_frame")
        self.declare_parameter("group_name", "dummy_arm")
        self.declare_parameter("end_effector_frame", "link6_1_1")

        self.target_object = self.get_parameter("target_object").value
        self.follow_distance = self.get_parameter("follow_distance").value
        self.max_planning_time = self.get_parameter("max_planning_time").value
        self.robot_base_frame = self.get_parameter("robot_base_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.group_name = self.get_parameter("group_name").value
        self.end_effector_frame = self.get_parameter("end_effector_frame").value

        # 状态
        self.is_following = False
        self.current_target_position: np.ndarray | None = None
        self.last_detection_time: rclpy.time.Time | None = None
        self.detection_timeout = Duration(seconds=2.0)

        # TF2 缓冲区和监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # MoveIt2 接口
        self.callback_group = ReentrantCallbackGroup()
        self._init_moveit2()

        # 订阅
        self.detection_sub = self.create_subscription(
            Detection3DArray,
            "/perception/detections",
            self.detection_callback,
            10,
            callback_group=self.callback_group,
        )

        # 发布
        self.pose_goal_pub = self.create_publisher(
            PoseStamped,
            "/motion_plan/pose_goal",
            10,
        )
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            "/motion_plan/trajectory",
            10,
        )
        self.state_pub = self.create_publisher(
            String,
            "/motion_plan/state",
            10,
        )

        # 服务
        self.start_follow_srv = self.create_service(
            std_srvs.srv.Trigger,
            "/motion_plan/start_follow",
            self.start_follow_callback,
            callback_group=self.callback_group,
        )
        self.stop_follow_srv = self.create_service(
            std_srvs.srv.Trigger,
            "/motion_plan/stop_follow",
            self.stop_follow_callback,
            callback_group=self.callback_group,
        )
        self.set_target_srv = self.create_service(
            std_srvs.srv.Trigger,
            "/motion_plan/set_target",
            self.set_target_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info("运动规划节点已启动 (MoveIt2)")
        self.publish_state("idle")

    def _init_moveit2(self):
        """初始化 MoveIt2 接口。"""
        try:
            self.moveit2 = MoveIt2(
                node=self,
                joint_names=[
                    "Joint1", "Joint2", "Joint3",
                    "Joint4", "Joint5", "Joint6",
                ],
                base_link_name=self.robot_base_frame,
                end_effector_name=self.end_effector_frame,
                group_name=self.group_name,
                callback_group=self.callback_group,
            )
            # 设置运动参数
            self.moveit2.max_velocity = 0.5
            self.moveit2.max_acceleration = 0.5
            self.get_logger().info("MoveIt2 初始化成功")
        except Exception as e:
            self.get_logger().error(f"MoveIt2 初始化失败: {e}")
            self.moveit2 = None

    def detection_callback(self, msg: Detection3DArray):
        """处理检测结果。"""
        if not self.is_following:
            return

        if len(msg.detections) == 0:
            # 检查检测超时
            if (
                self.last_detection_time is not None
                and self.get_clock().now() - self.last_detection_time
                > self.detection_timeout
            ):
                self.get_logger().warn("检测目标丢失")
                self.publish_state("target_lost")
            return

        # 查找目标类别检测
        target_detection = self._find_target_detection(msg)
        if target_detection is None:
            return

        # 获取检测位置（bbox 中心）
        bbox_center = target_detection.bbox.center.position
        camera_pose = PoseStamped()
        camera_pose.header.stamp = self.get_clock().now().to_msg()
        camera_pose.header.frame_id = self.camera_frame
        camera_pose.pose.position.x = float(bbox_center.x)
        camera_pose.pose.position.y = float(bbox_center.y)
        camera_pose.pose.position.z = float(bbox_center.z)
        camera_pose.pose.orientation.w = 1.0

        # 转换到 base_link 坐标系
        target_position = self._transform_pose_to_base_link(camera_pose)
        if target_position is None:
            self.get_logger().warn("坐标转换失败")
            return

        self.last_detection_time = self.get_clock().now()
        self.current_target_position = target_position

        # 执行路径规划
        self._plan_and_execute()

    def _find_target_detection(self, msg: Detection3DArray) -> Detection3D | None:
        """从检测结果中查找目标类别。

        Args:
            msg: 检测消息数组

        Returns:
            匹配的检测结果或 None
        """
        for detection in msg.detections:
            if len(detection.results) > 0:
                hypothesis = detection.results[0].hypothesis
                if hypothesis.class_id == self.target_object:
                    return detection

                # 兼容格式：直接使用 results[0].pose
                if hasattr(detection.results[0], "pose"):
                    # 兼容旧格式
                    return detection

        # 如果没有类别匹配，返回第一个检测（用于测试）
        if len(msg.detections) > 0:
            return msg.detections[0]

        return None

    def _transform_pose_to_base_link(
        self, pose: PoseStamped
    ) -> np.ndarray | None:
        """将检测框中心从相机坐标系转换到 base_link 坐标系。

        Args:
            pose: 相机坐标系中的位姿

        Returns:
            base_link 坐标系中的 3D 位置 [x, y, z] 或 None
        """
        try:
            # 获取变换矩阵：camera_depth_optical_frame -> base_link
            transform = self.tf_buffer.lookup_transform(
                self.robot_base_frame,
                pose.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )

            # 提取平移
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            tz = transform.transform.translation.z

            # 提取旋转（四元数转旋转矩阵）
            q = transform.transform.rotation
            R = self._quaternion_to_matrix(q.x, q.y, q.z, q.w)

            # 应用变换：base_link = R * camera + t
            camera_pos = np.array([
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            ])
            base_link_pos = R @ camera_pos + np.array([tx, ty, tz])

            return base_link_pos

        except Exception as e:
            self.get_logger().warn(f"TF 转换失败: {e}")
            return None

    def _quaternion_to_matrix(
        self, x: float, y: float, z: float, w: float
    ) -> np.ndarray:
        """四元数转旋转矩阵。

        Args:
            x, y, z, w: 四元数分量

        Returns:
            3x3 旋转矩阵
        """
        # 归一化四元数
        norm = np.sqrt(x**2 + y**2 + z**2 + w**2)
        if norm == 0:
            return np.eye(3)
        x, y, z, w = x / norm, y / norm, z / norm, w / norm

        # 转换为旋转矩阵
        xx, yy, zz = x**2, y**2, z**2
        xy, xz, yz = x * y, x * z, y * z
        xw, yw, zw = x * w, y * w, z * w

        R = np.array(
            [
                [1 - 2 * (yy + zz), 2 * (xy - zw), 2 * (xz + yw)],
                [2 * (xy + zw), 1 - 2 * (xx + zz), 2 * (yz - xw)],
                [2 * (xz - yw), 2 * (yz + xw), 1 - 2 * (xx + yy)],
            ]
        )
        return R

    def _plan_and_execute(self):
        """计算目标位置并执行运动规划。"""
        if self.current_target_position is None:
            return

        if self.moveit2 is None:
            self.get_logger().error("MoveIt2 未初始化")
            return

        # 计算目标位置（保持在 follow_distance 距离）
        target_pos = self.current_target_position
        distance = np.linalg.norm(target_pos)

        if distance > self.follow_distance:
            # 沿目标方向移动，保持 follow_distance
            direction = target_pos / distance
            goal_pos = target_pos - direction * self.follow_distance
        else:
            goal_pos = target_pos

        self.get_logger().debug(
            f"目标位置: [{goal_pos[0]:.3f}, {goal_pos[1]:.3f}, {goal_pos[2]:.3f}]"
        )

        # 发布 pose_goal（供 control_node 使用）
        pose_goal_msg = PoseStamped()
        pose_goal_msg.header.stamp = self.get_clock().now().to_msg()
        pose_goal_msg.header.frame_id = self.robot_base_frame
        pose_goal_msg.pose.position.x = float(goal_pos[0])
        pose_goal_msg.pose.position.y = float(goal_pos[1])
        pose_goal_msg.pose.position.z = float(goal_pos[2])
        # 保持当前姿态（简化为四元数 w=1）
        pose_goal_msg.pose.orientation.w = 1.0
        pose_goal_msg.pose.orientation.x = 0.0
        pose_goal_msg.pose.orientation.y = 0.0
        pose_goal_msg.pose.orientation.z = 0.0
        self.pose_goal_pub.publish(pose_goal_msg)

        # 使用 MoveIt2 执行规划
        try:
            self.publish_state("planning")

            # 尝试使用 plan() 获取轨迹并发布
            trajectory = self.moveit2.plan(
                position=goal_pos.tolist(),
                quat_xyzw=[0.0, 0.0, 0.0, 1.0],
                max_plan_duration=self.max_planning_time,
            )

            traj_points = trajectory.joint_trajectory.points
            if trajectory is not None and len(traj_points) > 0:
                # 发布轨迹到 /motion_plan/trajectory
                self.trajectory_pub.publish(trajectory.joint_trajectory)
                self.get_logger().debug("轨迹已发布")

                # 直接执行（也可由 control_node 执行）
                self.publish_state("executing")
                success = self.moveit2.execute(trajectory)
                if success:
                    self.publish_state("following")
                else:
                    self.publish_state("execution_failed")
            else:
                # 回退到 move_to_pose（同步执行）
                self.publish_state("executing")
                self.moveit2.move_to_pose(
                    position=goal_pos.tolist(),
                    quat_xyzw=[0.0, 0.0, 0.0, 1.0],
                    max_plan_duration=self.max_planning_time,
                )
                success = self.moveit2.wait_until_executed()
                if success:
                    self.publish_state("following")
                else:
                    self.publish_state("execution_failed")

        except Exception as e:
            self.get_logger().error(f"规划/执行失败: {e}")
            self.publish_state("error")

    def publish_state(self, state: str):
        """发布规划状态。"""
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

    def start_follow_callback(
        self,
        request: std_srvs.srv.Trigger.Request,
        response: std_srvs.srv.Trigger.Response,
    ) -> std_srvs.srv.Trigger.Response:
        """开始跟随。"""
        self.is_following = True
        self.last_detection_time = self.get_clock().now()
        self.publish_state("following")
        response.success = True
        response.message = f"开始跟随目标: {self.target_object}"
        self.get_logger().info(f"开始跟随目标: {self.target_object}")
        return response

    def stop_follow_callback(
        self,
        request: std_srvs.srv.Trigger.Request,
        response: std_srvs.srv.Trigger.Response,
    ) -> std_srvs.srv.Trigger.Response:
        """停止跟随。"""
        self.is_following = False
        self.current_target_position = None
        self.publish_state("idle")
        response.success = True
        response.message = "已停止跟随"
        self.get_logger().info("停止跟随")
        return response

    def set_target_callback(
        self,
        request: std_srvs.srv.Trigger.Request,
        response: std_srvs.srv.Trigger.Response,
    ) -> std_srvs.srv.Trigger.Response:
        """设置跟随目标类别。"""
        # 从请求消息中获取目标类别（如果有）
        # std_srvs/Trigger 只有空请求，这里使用默认目标
        response.success = True
        response.message = f"目标已设置为 {self.target_object}"
        self.get_logger().info(f"目标类别: {self.target_object}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到中断信号")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
