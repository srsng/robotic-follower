"""
标定管理器

管理整个标定流程，包括相机内参标定和外参标定。
"""

import os
import time
import asyncio
from typing import Optional, Callable
import numpy as np
import cv2

from hand_ros2_calib.calibration.calibration_types import (
    CalibrationState,
    CalibrationConfig,
    CalibrationResult,
    CameraIntrinsics,
    HandEyeTransform,
)
from hand_ros2_calib.calibration.intrinsic_calibrator import IntrinsicCalibrator
from hand_ros2_calib.calibration.extrinsic_calibrator import ExtrinsicCalibrator


class CalibrationManager:
    """
    标定管理器

    功能：
    - 管理标定状态
    - 协调内参和外参标定流程
    - 提供用户回调接口
    - 保存/加载标定结果
    """

    def __init__(self, config: CalibrationConfig):
        """
        初始化标定管理器

        Args:
            config: 标定配置
        """
        self.config = config
        self.state = CalibrationState.IDLE

        # 标定器
        self.intrinsic_calibrator: Optional[IntrinsicCalibrator] = None
        self.extrinsic_calibrator: Optional[ExtrinsicCalibrator] = None

        # 标定结果
        self.intrinsics: Optional[CameraIntrinsics] = None
        self.hand_eye_transform: Optional[HandEyeTransform] = None

        # 回调函数
        self.on_state_change: Optional[Callable] = None
        self.on_image_added: Optional[Callable] = None
        self.on_progress: Optional[Callable] = None

        # 输出目录
        os.makedirs(self.config.output_directory, exist_ok=True)

    def _set_state(self, state: CalibrationState):
        """设置标定状态"""
        self.state = state
        if self.on_state_change:
            self.on_state_change(state)

    # ==================== 内参标定 ====================

    def start_intrinsic_calibration(self):
        """启动内参标定"""
        self._set_state(CalibrationState.INTRINSIC_CALIBRATING)
        self.intrinsic_calibrator = IntrinsicCalibrator(self.config)
        print("内参标定已启动")

    def add_intrinsic_image(self, image: np.ndarray) -> bool:
        """
        添加内参标定图像

        Args:
            image: 标定图像

        Returns:
            bool: 是否成功添加
        """
        if self.state != CalibrationState.INTRINSIC_CALIBRATING:
            print("错误：当前不在内参标定状态")
            return False

        # 检测标定板
        detected, corners = self.intrinsic_calibrator.detect_board(image, visualize=False)

        if not detected or corners is None:
            print("警告：图像中未检测到标定板")
            return False

        # 添加图像
        success = self.intrinsic_calibrator.add_image(image, corners)

        if success:
            count = len(self.intrinsic_calibrator.images)
            print(f"已添加内参图像 {count}/{self.config.intrinsic_num_images}")

            if self.on_image_added:
                self.on_image_added(count, self.config.intrinsic_num_images)

            if self.on_progress:
                self.on_progress(
                    "intrinsic",
                    count,
                    self.config.intrinsic_num_images
                )

        return success

    def complete_intrinsic_calibration(self) -> Optional[CalibrationResult]:
        """
        完成内参标定

        Returns:
            Optional[CalibrationResult]: 标定结果
        """
        if self.intrinsic_calibrator is None:
            return None

        # 执行标定
        result, rms = self.intrinsic_calibrator.calibrate()

        if result is None:
            self._set_state(CalibrationState.FAILED)
            return CalibrationResult(success=False, error_message="内参标定失败")

        # 保存结果
        output_path = os.path.join(
            self.config.output_directory,
            self.config.intrinsic_output_file
        )
        self.intrinsic_calibrator.save_to_xml(result, output_path)

        self.intrinsics = result

        self._set_state(CalibrationState.IDLE)

        return CalibrationResult(
            success=True,
            intrinsics=result,
            intrinsic_rms_error=rms,
        )

    # ==================== 外参标定 ====================

    def start_extrinsic_calibration(self, intrinsics: Optional[CameraIntrinsics] = None):
        """
        启动外参标定

        Args:
            intrinsics: 相机内参，如果为 None 使用之前的结果
        """
        if intrinsics is None:
            if self.intrinsics is None:
                raise ValueError("需要先完成内参标定或提供内参")
            intrinsics = self.intrinsics

        self._set_state(CalibrationState.EXTRINSIC_CALIBRATING)
        self.extrinsic_calibrator = ExtrinsicCalibrator(self.config, intrinsics)
        print("外参标定已启动")

    def add_extrinsic_pair(
        self,
        robot_pose: tuple,  # (x, y, z, rx, ry, rz) mm 和度
        image: np.ndarray,
    ) -> bool:
        """
        添加外参标定数据对（机器人位姿 + 图像）

        Args:
            robot_pose: 机器人位姿 (x, y, z, rx, ry, rz)，单位 mm 和度
            image: 标定板图像

        Returns:
            bool: 是否成功添加
        """
        if self.state != CalibrationState.EXTRINSIC_CALIBRATING:
            print("错误：当前司在外参标定状态")
            return False

        # 添加数据对
        success = self.extrinsic_calibrator.add_calibration_pair(robot_pose, image)

        if success:
            count = len(self.extrinsic_calibrator.calibration_data)
            print(f"已添加外参数据对 {count}/{self.config.extrinsic_num_poses}")

            if self.on_image_added:
                self.on_image_added(count, self.config.extrinsic_num_poses)

            if self.on_progress:
                self.on_progress(
                    "extrinsic",
                    count,
                    self.config.extrinsic_num_poses
                )

        return success

    def complete_extrinsic_calibration(self) -> Optional[CalibrationResult]:
        """
        完成外参标定

        Returns:
            Optional[CalibrationResult]: 标定结果
        """
        if self.extrinsic_calibrator is None:
            return None

        # 执行标定
        result, rms = self.extrinsic_calibrator.calibrate()

        if result is None:
            self._set_state(CalibrationState.FAILED)
            return CalibrationResult(success=False, error_message="外参标定失败")

        # 保存结果
        output_path = os.path.join(
            self.config.output_directory,
            self.config.extrinsic_output_file
        )
        self.extrinsic_calibrator.save_to_xml(result, output_path)

        # 同时导出为 C++ 格式
        self.extrinsic_calibrator.export_to_cpp_format(
            os.path.join(self.config.output_directory, "cpp_data")
        )

        self.hand_eye_transform = result

        self._set_state(CalibrationState.IDLE)

        return CalibrationResult(
            success=True,
            hand_eye_transform=result,
            extrinsic_rms_error=rms,
        )

    # ==================== 完整标定流程 ====================

    async def run_full_calibration(
        self,
        get_image_fn: Callable[[], np.ndarray],
        get_robot_pose_fn: Callable[[], tuple],
        prompt_fn: Optional[Callable[[str], None]] = None,
    ) -> Optional[CalibrationResult]:
        """
        运行完整标定流程（内参 + 外参）

        Args:
            get_image_fn: 获取图像的函数
            get_robot_pose_fn: 获取机器人位姿的函数
            prompt_fn: 提示用户的函数

        Returns:
            Optional[CalibrationResult]: 标定结果
        """
        # 阶段 1：内参标定
        await self._run_intrinsic_calibration_async(
            get_image_fn, prompt_fn
        )

        if self.intrinsics is None:
            return CalibrationResult(success=False, error_message="内参标定失败")

        # 阶段 2：外参标定
        await self._run_extrinsic_calibration_async(
            get_image_fn, get_robot_pose_fn, prompt_fn
        )

        if self.hand_eye_transform is None:
            return CalibrationResult(success=False, error_message="外参标定失败")

        self._set_state(CalibrationState.COMPLETED)

        return CalibrationResult(
            success=True,
            intrinsics=self.intrinsics,
            hand_eye_transform=self.hand_eye_transform,
        )

    async def _run_intrinsic_calibration_async(
        self,
        get_image_fn: Callable,
        prompt_fn: Optional[Callable],
    ):
        """异步运行内参标定"""
        self.start_intrinsic_calibration()

        if prompt_fn:
            prompt_fn("=== 开始相机内参标定 ===")
            prompt_fn(f"请手持标定板，准备拍摄 {self.config.intrinsic_num_images} 张图像")

        for i in range(self.config.intrinsic_num_images):
            if prompt_fn:
                prompt_fn(f"请拍摄第 {i+1}/{self.config.intrinsic_num_images} 张图像")

            # 等待用户确认
            await asyncio.sleep(1.0)

            # 获取图像
            image = get_image_fn()

            # 添加图像
            self.add_intrinsic_image(image)

            # 短暂延迟
            await asyncio.sleep(0.5)

        if prompt_fn:
            prompt_fn("正在执行内参标定...")

        return self.complete_intrinsic_calibration()

    async def _run_extrinsic_calibration_async(
        self,
        get_image_fn: Callable,
        get_robot_pose_fn: Callable,
        prompt_fn: Optional[Callable],
    ):
        """异步运行外参标定"""
        self.start_extrinsic_calibration()

        if prompt_fn:
            prompt_fn("=== 开始 3D 手眼标定 ===")
            prompt_fn(f"请移动机器人并拍摄 {self.config.extrinsic_num_poses} 组图像")
            prompt_fn("重要：必须包含足够的旋转，纯平移无法准确标定")

        # 推荐拍摄序列（确保包含旋转）
        pose_commands = [
            "初始位置",
            "前移 + 绕 X 轴旋转",
            "后移 + 绕 X 轴负向旋转",
            "左移 + 绕 Y 轴旋转",
            "右移 + 绕 Y 轴负向旋转",
            "上移 + 绕 Z 轴旋转",
            "下移 + 绕 Z 轴负向旋转",
        ]

        for i in range(self.config.extrinsic_num_poses):
            if prompt_fn:
                pose_desc = pose_commands[i % len(pose_commands)]
                prompt_fn(f"位姿 {i+1}/{self.config.extrinsic_num_poses}: {pose_desc}")

            # 等待用户确认
            await asyncio.sleep(1.0)

            # 获取机器人位姿
            robot_pose = get_robot_pose_fn()

            # 获取图像
            image = get_image_fn()

            # 添加数据对
            self.add_extrinsic_pair(robot_pose, image)

            # 短暂延迟
            await asyncio.sleep(0.5)

        if prompt_fn:
            prompt_fn("正在执行手眼标定...")

        return self.complete_extrinsic_calibration()

    # ==================== 加载/保存标定结果 ====================

    def load_calibration_results(
        self,
        intrinsic_path: Optional[str] = None,
        extrinsic_path: Optional[str] = None,
    ) -> bool:
        """
        加载标定结果

        Args:
            intrinsic_path: 内参文件路径
            extrinsic_path: 外参文件路径

        Returns:
            bool: 是否成功加载
        """
        success = True

        if intrinsic_path is None:
            intrinsic_path = os.path.join(
                self.config.output_directory,
                self.config.intrinsic_output_file
            )

        if extrinsic_path is None:
            extrinsic_path = os.path.join(
                self.config.output_directory,
                self.config.extrinsic_output_file
            )

        # 加载内参
        if os.path.exists(intrinsic_path):
            self.intrinsics = IntrinsicCalibrator.load_from_xml(intrinsic_path)
            if self.intrinsics is not None:
                print(f"已加载内参: {intrinsic_path}")
            else:
                success = False
        else:
            print(f"内参文件不存在: {intrinsic_path}")

        # 加载外参
        if os.path.exists(extrinsic_path):
            self.hand_eye_transform = ExtrinsicCalibrator.load_from_xml(extrinsic_path)
            if self.hand_eye_transform is not None:
                print(f"已加载手眼变换: {extrinsic_path}")
            else:
                success = False
        else:
            print(f"外参文件不存在: {extrinsic_path}")

        return success

    def clear(self):
        """清除所有标定数据"""
        if self.intrinsic_calibrator:
            self.intrinsic_calibrator.clear()
        if self.extrinsic_calibrator:
            self.extrinsic_calibrator.clear()

        self.intrinsics = None
        self.hand_eye_transform = None
        self._set_state(CalibrationState.IDLE)
