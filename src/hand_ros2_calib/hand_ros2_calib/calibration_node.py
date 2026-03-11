#!/usr/bin/env python3
"""
手眼标定节点

提供相机内参标定和 3D 手眼标定服务。
"""

import os
import time
import asyncio
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import cv2
import numpy as np

from hand_ros2_calib.calibration.calibration_types import (
    CalibrationConfig,
    CalibrationState,
    PatternType,
    HandEyeMode,
)
from hand_ros2_calib.calibration.calibration_manager import CalibrationManager
from hand_ros2_calib.camera.realsense_camera import RealsenseCamera
from hand_ros2_calib.robot.robot_interface import RobotInterface
from hand_ros2_calib.utils.visualization import VisualizationHelper
from hand_ros2_calib.utils.file_io import FileIO


class CalibrationNode(Node):
    """
    手眼标定节点

    功能：
    - 启动相机和机器人接口
    - 执行内参标定
    - 执行外参标定
    - 提供用户交互界面
    """

    def __init__(self):
        super().__init__('hand_eye_calibration_node')

        # 加载配置
        self._load_config()

        # 初始化组件
        self.camera = RealsenseCamera(self)
        self.robot = RobotInterface(
            self,
            moveit_group="arm",
            base_frame="base_link",
            end_effector_frame="link6_1_1",
        )
        self.visualization = VisualizationHelper(self)

        # 标定管理器
        self.calib_manager = CalibrationManager(self.config)

        # 设置回调
        self.calib_manager.on_state_change = self._on_state_change
        self.calib_manager.on_image_added = self._on_image_added
        self.calib_manager.on_progress = self._on_progress

        # 启动相机
        self.camera.start()
        self.camera.configure_for_calibration()

        # 等待机器人就绪
        self.get_logger().info("等待机器人就绪...")
        self.robot.wait_for_robot_ready(timeout=10.0)

        # 状态变量
        self.current_state = CalibrationState.IDLE

        # 消息订阅
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/calibration_pose_command',
            self._pose_command_callback,
            10
        )

        self.get_logger().info('手眼标定节点已启动')

    def _load_config(self):
        """加载标定配置"""
        self.config = CalibrationConfig()

        # 从参数服务器加载（如果配置了）
        self.declare_parameter('pattern_type', 'circles_asymmetric')
        self.declare_parameter('board_cols', 4)
        self.declare_parameter('board_rows', 5)
        self.declare_parameter('square_size_mm', 20.0)
        self.declare_parameter('intrinsic_num_images', 20)
        self.declare_parameter('extrinsic_num_poses', 15)
        self.declare_parameter('hand_eye_mode', 'EIH')

        # 读取参数
        pattern_type = self.get_parameter('pattern_type').value
        self.config.pattern_type = self._parse_pattern_type(pattern_type)
        self.config.board_cols = self.get_parameter('board_cols').value
        self.config.board_rows = self.get_parameter('board_rows').value
        self.config.square_size_mm = self.get_parameter('square_size_mm').value
        self.config.intrinsic_num_images = self.get_parameter('intrinsic_num_images').value
        self.config.extrinsic_num_poses = self.get_parameter('extrinsic_num_poses').value

        hand_eye_mode = self.get_parameter('hand_eye_mode').value
        self.config.hand_eye_mode = self._parse_hand_eye_mode(hand_eye_mode)

    def _parse_pattern_type(self, type_str: str) -> PatternType:
        """解析标定板类型"""
        type_map = {
            'chessboard': PatternType.CHESSBOARD,
            'circles_symmetric': PatternType.CIRCLES_SYMMETRIC,
            'circles_asymmetric': PatternType.CIRCLES_ASYMMETRIC,
        }
        return type_map.get(type_str, PatternType.CIRCLES_ASYMMETRIC)

    def _parse_hand_eye_mode(self, mode_str: str) -> HandEyeMode:
        """解析手眼标定模式"""
        mode_map = {
            'EIH': HandEyeMode.EIH,
            'ETH': HandEyeMode.ETH,
        }
        return mode_map.get(mode_str.upper(), HandEyeMode.EIH)

    # ==================== 回调函数 ====================

    def _on_state_change(self, state: CalibrationState):
        """标定状态改变回调"""
        self.current_state = state
        self.get_logger().info(f"标定状态: {state.value}")

    def _on_image_added(self, current: int, total: int):
        """图像添加回调"""
        self.get_logger().info(f"图像进度: {current}/{total}")

    def _on_progress(self, stage: str, current: int, total: int):
        """进度回调"""
        self.get_logger().info(f"{stage} 进度: {current}/{total}")

    def _pose_command_callback(self, msg: PoseStamped):
        """位姿命令回调"""
        self.get_logger().info(
            f"收到位姿命令: {msg.header.frame_id} -> "
            f"({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, "
            f"{msg.pose.position.z:.3f})"
        )

    # ==================== 标定流程 ====================

    async def run_intrinsic_calibration(self):
        """运行内参标定"""
        self.get_logger().info("=== 开始相机内参标定 ===")

        # 启动内参标定
        self.calib_manager.start_intrinsic_calibration()

        # 采集图像
        for i in range(self.config.intrinsic_num_images):
            self.get_logger().info(f"请拍摄第 {i+1}/{self.config.intrinsic_num_images} 张图像")
            self.get_logger().info("按 's' 拍摄，按 'q' 退出")

            # 等待用户输入
            key = await self._wait_for_key(['s', 'q'])

            if key == 'q':
                self.get_logger().info("用户取消标定")
                return False

            # 获取图像
            image = self.camera.get_color_image()

            if image is None:
                self.get_logger().warn("无法获取相机图像")
                continue

            # 调整图像大小
            image = FileIO.resize_image(image, max_width=1280)

            # 添加图像
            success = self.calib_manager.add_intrinsic_image(image)

            if not success:
                self.get_logger().warn("添加图像失败")
                continue

            # 显示进度
            vis_image = self.visualization.draw_progress(
                image, i+1, self.config.intrinsic_num_images, "内参标定"
            )
            self.visualization.show_image(vis_image, "Calibration")

        # 完成标定
        self.get_logger().info("正在执行内参标定...")
        result = self.calib_manager.complete_intrinsic_calibration()

        if result and result.success:
            self.get_logger().info(f"内参标定成功！RMS 误差: {result.intrinsic_rms_error:.4f} 像素")
            return True
        else:
            self.get_logger().error("内参标定失败")
            return False

    async def run_extrinsic_calibration(self):
        """运行外参标定"""
        self.get_logger().info("=== 开始 3D 手眼标定 ===")

        # 启动外参标定
        self.calib_manager.start_extrinsic_calibration()

        # 推荐拍摄序列
        pose_prompts = [
            "初始位置",
            "前移 + 绕 X 轴旋转",
            "后移 + 绕 X 轴负向旋转",
            "左移 + 绕 Y 轴旋转",
            "右移 + 绕 Y 轴负向旋转",
            "上移 + 绕 Z 轴旋转",
            "下移 + 绕 Z 轴负向旋转",
        ]

        # 采集数据对
        for i in range(self.config.extrinsic_num_poses):
            prompt = pose_prompts[i % len(pose_prompts)]
            self.get_logger().info(f"位姿 {i+1}/{self.config.extrinsic_num_poses}: {prompt}")
            self.get_logger().info("按 's' 拍摄，按 'q' 退出")

            # 等待用户输入
            key = await self._wait_for_key(['s', 'q'])

            if key == 'q':
                self.get_logger().info("用户取消标定")
                return False

            # 获取机器人位姿
            robot_pose_tuple = self.robot.get_current_pose_tuple(timeout=1.0)

            if robot_pose_tuple is None:
                self.get_logger().warn("无法获取机器人位姿")
                continue

            # 获取图像
            image = self.camera.get_color_image()

            if image is None:
                self.get_logger().warn("无法获取相机图像")
                continue

            # 调整图像大小
            image = FileIO.resize_image(image, max_width=1280)

            # 添加数据对
            success = self.calib_manager.add_extrinsic_pair(robot_pose_tuple, image)

            if not success:
                self.get_logger().warn("添加数据对失败")
                continue

            # 显示进度
            vis_image = self.visualization.draw_pose_info(image, robot_pose_tuple)
            vis_image = self.visualization.draw_progress(
                vis_image, i+1, self.config.extrinsic_num_poses, "手眼标定"
            )
            self.visualization.show_image(vis_image, "Calibration")

        # 完成标定
        self.get_logger().info("正在执行手眼标定...")
        result = self.calib_manager.complete_extrinsic_calibration()

        if result and result.success:
            self.get_logger().info(f"手眼标定成功！RMS 误差: {result.extrinsic_rms_error:.4f}")
            return True
        else:
            self.get_logger().error("手眼标定失败")
            return False

    async def _wait_for_key(self, keys: list) -> str:
        """等待用户按键"""
        loop = asyncio.get_event_loop()
        future = loop.create_future()

        def key_check():
            key = cv2.waitKey(50) & 0xFF
            for k in keys:
                if key == ord(k):
                    loop.call_soon_threadsafe(future.set_result, k)
                    return

        # 定期检查按键
        timer = loop.create_timer(0.05, key_check)
        timer.start()

        try:
            return await asyncio.wait_for(future, timeout=300.0)
        except asyncio.TimeoutError:
            return 'q'  # 超时退出
        finally:
            timer.cancel()

    # ==================== 主要入口 ====================

    async def run_full_calibration(self):
        """运行完整标定流程"""
        self.get_logger().info("========================================")
        self.get_logger().info("   手眼标定流程")
        self.get_logger().info("========================================")

        # 阶段 1：内参标定
        success = await self.run_intrinsic_calibration()

        if not success:
            self.get_logger().error("内参标定失败，终止流程")
            return

        # 等待用户继续
        self.get_logger().info("按 'c' 继续，按 'q' 退出")
        key = await self._wait_for_key(['c', 'q'])

        if key == 'q':
            return

        # 阶段 2：外参标定
        success = await self.run_extrinsic_calibration()

        if success:
            self.get_logger().info("========================================")
            self.get_logger().info("   标定完成！")
            self.get_logger().info("========================================")
        else:
            self.get_logger().error("外参标定失败")

        # 关闭图像窗口
        self.visualization.close_all_windows()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)

    try:
        node = CalibrationNode()
        # 异步运行标定
        loop = asyncio.get_event_loop()
        loop.run_until_complete(node.run_full_calibration())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
