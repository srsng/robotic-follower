"""
手眼标定 ROS2 模块

提供相机内参标定和 3D 手眼标定功能。
"""

__version__ = "0.0.1"
__author__ = "Your Name"

from hand_ros2_calib.calibration.calibration_manager import CalibrationManager
from hand_ros2_calib.calibration.calibration_types import CalibrationState, CalibrationConfig

__all__ = ["CalibrationManager", "CalibrationState", "CalibrationConfig"]
