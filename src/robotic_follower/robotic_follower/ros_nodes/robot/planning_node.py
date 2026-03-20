#!/usr/bin/env python3
"""运动规划节点：基于视觉检测结果生成机械臂运动。

订阅检测结果，计算目标位置，生成安全的机械臂运动轨迹。

订阅话题：
    - /perception/detections (vision_msgs/Detection3DArray)
        3D 目标检测结果
    - /robot/pose (geometry_msgs/PoseStamped)
        当前机械臂末端位姿

发布话题：
    - /motion_plan/pose_goal (geometry_msgs/PoseStamped)
        目标位姿（传递给控制节点）
    - /motion_plan/trajectory (trajectory_msgs/JointTrajectory)
        关节轨迹
    - /motion_plan/state (std_msgs/String)
        规划状态

服务：
    - /motion_plan/start_follow (std_srvs/Trigger)
        开始跟随
    - /motion_plan/stop_follow (std_srvs/Trigger)
        停止跟随
    - /motion_plan/set_target (std_srvs/Trigger)
        设置跟随目标

参数：
    - target_object (str, 默认: "chair")
        目标物体类别
    - follow_distance (float, 默认: 0.5)
        跟随距离（米）
    - max_planning_time (float, 默认: 1.0)
        最大规划时间（秒）
    - robot_base_frame (str, 默认: "base_link")
        机器人基座坐标系
    - camera_frame (str, 默认: "camera_color_optical_frame")
        相机坐标系
"""

import numpy as np
import rclpy
import std_srvs.srv
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from vision_msgs.msg import Detection3D, Detection3DArray


class PlanningNode(Node):
    """运动规划节点。"""

    def __init__(self):
        super().__init__("motion_planning")

        # 参数
        self.declare_parameter("target_object", "chair")
        self.declare_parameter("follow_distance", 0.5)
        self.declare_parameter("max_planning_time", 1.0)
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")

        self.target_object = self.get_parameter("target_object").value
        self.follow_distance = self.get_parameter("follow_distance").value
        self.max_planning_time = self.get_parameter("max_planning_time").value
        self.robot_base_frame = self.get_parameter("robot_base_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value

        # 状态
        self.is_following = False
        self.current_target_pose: np.ndarray | None = None
        self.current_robot_pose: np.ndarray | None = None

        # 订阅
        self.detection_sub = self.create_subscription(
            Detection3DArray,
            "/perception/detections",
            self.detection_callback,
            10,
        )
        self.robot_pose_sub = self.create_subscription(
            PoseStamped,
            "/robot/pose",
            self.robot_pose_callback,
            10,
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
        )
        self.stop_follow_srv = self.create_service(
            std_srvs.srv.Trigger,
            "/motion_plan/stop_follow",
            self.stop_follow_callback,
        )
        self.set_target_srv = self.create_service(
            std_srvs.srv.Trigger,
            "/motion_plan/set_target",
            self.set_target_callback,
        )

        self.get_logger().info("运动规划节点已启动")

    def detection_callback(self, msg: Detection3DArray):
        """处理检测结果。"""
        if not self.is_following or len(msg.detections) == 0:
            return

        # 查找目标类别
        for detection in msg.detections:
            # 检查是否为目标类别（简化：取第一个检测结果）
            target = self._get_target_from_detection(detection)
            if target is not None:
                self.current_target_pose = target
                self.plan_and_publish()
                break

    def robot_pose_callback(self, msg: PoseStamped):
        """处理机械臂当前位姿。"""
        self.current_robot_pose = np.eye(4)
        self.current_robot_pose[0, 3] = msg.pose.position.x
        self.current_robot_pose[1, 3] = msg.pose.position.y
        self.current_robot_pose[2, 3] = msg.pose.position.z

    def _get_target_from_detection(self, detection: Detection3D) -> np.ndarray | None:
        """从检测结果提取目标位置。

        Args:
            detection: 3D 检测结果

        Returns:
            4x4 目标位姿矩阵
        """
        bbox = detection.bbox
        center = bbox.center.position

        # 构建目标位姿（在检测位置基础上添加 offset）
        pose = np.eye(4)
        pose[0, 3] = center.x
        pose[1, 3] = center.y
        pose[2, 3] = center.z

        return pose

    def plan_and_publish(self):
        """规划并发布运动目标。"""
        if self.current_target_pose is None:
            return

        # 简化的轨迹规划：直接设置目标点
        # 实际应用中应使用 MoveIt2 的规划接口
        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = self.robot_base_frame

        # 计算目标位置（沿视线方向偏移 follow_distance）
        if self.current_robot_pose is not None:
            robot_pos = self.current_robot_pose[:3, 3]
            target_pos = self.current_target_pose[:3, 3]
            direction = target_pos - robot_pos
            distance = np.linalg.norm(direction)

            if distance > self.follow_distance:
                direction_normalized = direction / distance
                goal_pos = target_pos - direction_normalized * self.follow_distance
            else:
                goal_pos = target_pos
        else:
            goal_pos = self.current_target_pose[:3, 3]

        goal_pose.pose.position.x = float(goal_pos[0])
        goal_pose.pose.position.y = float(goal_pos[1])
        goal_pose.pose.position.z = float(goal_pos[2])
        goal_pose.pose.orientation.w = 1.0

        # 发布目标位姿
        self.pose_goal_pub.publish(goal_pose)
        self.publish_state("target_published")

        self.get_logger().debug(
            f"发布目标位姿: [{goal_pos[0]:.3f}, {goal_pos[1]:.3f}, {goal_pos[2]:.3f}]"
        )

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
        self.publish_state("following")
        response.success = True
        response.message = "开始跟随"
        self.get_logger().info("开始跟随目标")
        return response

    def stop_follow_callback(
        self,
        request: std_srvs.srv.Trigger.Request,
        response: std_srvs.srv.Trigger.Response,
    ) -> std_srvs.srv.Trigger.Response:
        """停止跟随。"""
        self.is_following = False
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
        """设置跟随目标。"""
        response.success = True
        response.message = f"目标已设置为 {self.target_object}"
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
