"""手眼标定管理器。"""

import time
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional

import numpy as np
import yaml
from rclpy.node import Node


class CalibrationState(Enum):
    """标定状态。"""
    IDLE = "idle"
    COLLECTING = "collecting"
    CALIBRATING = "calibrating"
    COMPLETED = "completed"
    FAILED = "failed"


@dataclass
class CalibrationConfig:
    """标定配置。"""
    board_type: str = "circles_asymmetric"
    board_cols: int = 4
    board_rows: int = 5
    circle_diameter: float = 0.020  # meters
    min_samples: int = 15
    max_samples: int = 50


@dataclass
class CalibrationSample:
    """标定样本。"""
    image: np.ndarray
    robot_pose: np.ndarray  # 4x4 齐次变换矩阵
    camera_pose: np.ndarray  # 4x4 齐次变换矩阵
    timestamp: float


class CalibrationManager:
    """标定管理器。"""

    def __init__(self, config: CalibrationConfig, node: Optional[Node] = None):
        """初始化标定管理器。

        Args:
            config: 标定配置
            node: ROS2节点（可选，用于日志）
        """
        self.config = config
        self.node = node
        self.state = CalibrationState.IDLE
        self.samples: List[CalibrationSample] = []
        self.last_result: Optional[dict] = None

        # 组件将在外部注入
        self.robot_interface = None
        self.camera_capture = None
        self.calibrator = None
        self.validator = None

    def _log(self, message: str, level: str = "info"):
        """日志输出。"""
        if self.node:
            getattr(self.node.get_logger(), level)(message)
        else:
            print(f"[{level.upper()}] {message}")

    def start_calibration(self):
        """开始标定收集。"""
        self.state = CalibrationState.COLLECTING
        self.samples = []
        self._log("开始标定样本收集")
        if self.robot_interface:
            self.robot_interface.start()

    def add_sample(self, robot_pose: np.ndarray, image: np.ndarray, camera_pose: np.ndarray) -> bool:
        """添加标定样本。

        Args:
            robot_pose: 机械臂位姿（4x4齐次变换矩阵）
            image: 标定板图像
            camera_pose: 相机位姿（4x4齐次变换矩阵）

        Returns:
            是否成功添加
        """
        if self.state != CalibrationState.COLLECTING:
            self._log("未处于收集状态", "warn")
            return False

        if len(self.samples) >= self.config.max_samples:
            self._log(f"已达到最大样本数 {self.config.max_samples}", "warn")
            return False

        # 验证输入
        if robot_pose.shape != (4, 4) or camera_pose.shape != (4, 4):
            self._log("位姿矩阵形状错误，应为4x4", "error")
            return False

        # 保存样本
        sample = CalibrationSample(
            image=image,
            robot_pose=robot_pose.copy(),
            camera_pose=camera_pose.copy(),
            timestamp=time.time()
        )
        self.samples.append(sample)

        self._log(f"已添加第 {len(self.samples)} 个样本")

        return True

    def execute_calibration(self) -> dict:
        """执行标定。

        Returns:
            标定结果字典

        Raises:
            ValueError: 样本数量不足或标定失败
        """
        if len(self.samples) < self.config.min_samples:
            raise ValueError(f"样本数量不足：{len(self.samples)} < {self.config.min_samples}")

        if self.calibrator is None:
            raise ValueError("标定器未初始化")

        self.state = CalibrationState.CALIBRATING
        self._log("开始执行标定...")

        # 提取位姿对
        robot_poses = [s.robot_pose for s in self.samples]
        camera_poses = [s.camera_pose for s in self.samples]

        # 求解手眼变换
        result = self.calibrator.calibrate(robot_poses, camera_poses)

        # 验证标定
        if self.validator:
            validation = self.validator.validate(result, self.samples)
            result.update(validation)

            if validation.get('max_error', 0) > 0.01:  # 1cm误差阈值
                self.state = CalibrationState.FAILED
                error_msg = f"标定精度不足：{validation['max_error']:.4f}m"
                self._log(error_msg, "error")
                raise ValueError(error_msg)

        self.state = CalibrationState.COMPLETED
        self.last_result = result
        self._log(f"标定完成，误差：{result['error']:.6f}m")

        return result

    def reset(self) -> bool:
        """重置标定。"""
        self.state = CalibrationState.IDLE
        self.samples = []
        self.last_result = None
        self._log("标定已重置")
        return True

    def save_results(self, filepath: str) -> bool:
        """保存标定结果。

        Args:
            filepath: 保存路径

        Returns:
            是否成功保存
        """
        if self.state != CalibrationState.COMPLETED or self.last_result is None:
            self._log("无有效标定结果", "warn")
            return False

        try:
            # 转换numpy数组为列表
            result_to_save = {}
            for key, value in self.last_result.items():
                if isinstance(value, np.ndarray):
                    result_to_save[key] = value.tolist()
                else:
                    result_to_save[key] = value

            with open(filepath, 'w') as f:
                yaml.dump(result_to_save, f, default_flow_style=False, allow_unicode=True)

            self._log(f"标定结果已保存至 {filepath}")
            return True
        except Exception as e:
            self._log(f"保存结果失败：{e}", "error")
            return False

    def get_status(self) -> dict:
        """获取标定状态。

        Returns:
            状态字典
        """
        return {
            'state': self.state.value,
            'sample_count': len(self.samples),
            'min_samples': self.config.min_samples,
            'max_samples': self.config.max_samples,
        }
