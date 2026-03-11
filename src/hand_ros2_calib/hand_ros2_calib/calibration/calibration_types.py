"""
标定类型定义

定义标定状态、配置和结果的数据结构。
"""

from enum import Enum
from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Optional
from geometry_msgs.msg import Pose
from sensor_msgs.msg import CameraInfo


class CalibrationState(Enum(Enum):
    """标定状态枚举"""
    IDLE = "idle"                     # 空闲
    INITIALIZING = "initializing"       # 初始化中
    INTRINSIC_CALIBRATING = "intrinsic_calibrating"  # 内参标定中
    EXTRINSIC_CALIBRATING = "extrinsic_calibrating"  # 外参标定中
    VALIDATING = "validating"           # 验证中
    COMPLETED = "completed"             # 已完成
    FAILED = "failed"                  # 失败


class CalibrationType(Enum(Enum)):
    """标定类型枚举"""
    INTRINSIC = "intrinsic"           # 内参标定
    EXTRINSIC_3D_BOARD = "extrinsic_3d_board"  # 3D 标定板标定
    EXTRINSIC_3D_BALL = "extrinsic_3d_ball"      # 3D 标定球标定
    EXTRINSIC_2D_4POINT = "extrinsic_2d_4point"  # 2D 4 点标定
    EXTRINSIC_2D_12POINT = "extrinsic_2d_12point"  # 2D 12 点标定


class HandEyeMode(Enum(Enum)):
    """手眼标定模式"""
    EIH = "EIH"     # Eye-in-Hand（眼在手上）
    ETH = "ETH"     # Eye-to-Hand（眼在手外）


class PatternType(Enum(Enum)):
    """标定板图案类型"""
    CHESSBOARD = "chessboard"     # 棋盘格
    CIRCLES_SYMMETRIC = "circles_symmetric"   # 对称圆点
    CIRCLES_ASYMMETRIC = "circles_asymmetric"  # 非对称圆点


@dataclass
class CalibrationConfig:
    """标定配置"""
    # 标定板配置
    pattern_type: PatternType = PatternType.CIRCLES_ASYMMETRIC
    board_cols: int = 4
    board_rows: int = 5
    square_size_mm: float = 20.0
    marker_length_mm: float = 50.0

    # 内参标定配置
    intrinsic_num_images: int = 20
    intrinsic_min_distance: float = 0.5
    intrinsic_max_distance: float = 3.0

    # 外参标定配置
    hand_eye_mode: HandEyeMode = HandEyeMode.EIH
    extrinsic_num_poses: int = 15
    extrinsic_min_rotation: float = 20.0  # 度
    extrinsic_min_translation: float = 0.05  # 米

    # 验证配置
    validation_num_test_images: int = 5
    max_pixel_error: float = 0.3
    max_physical_error_mm: float = 1.0

    # 文件输出配置
    output_directory: str = "./calibration_results"
    intrinsic_output_file: str = "calib_intrinsic.xml"
    extrinsic_output_file: str = "calib_extrinsic.xml"


@dataclass
class CameraIntrinsics:
    """相机内参"""
    camera_matrix: List[List[float]] = field(default_factory=list)
    distortion_coeffs: List[float] = field(default_factory=list)
    image_width: int = 0
    image_height: int = 0

    def to_camera_info(self, frame_id: str = "camera_optical") -> CameraInfo:
        """转换为 CameraInfo 消息"""
        camera_info = CameraInfo()

        camera_info.header.frame_id = frame_id
        camera_info.width = self.image_width
        camera_info.height = self.image_height

        # 设置相机矩阵 K (3x3)
        if len(self.camera_matrix) == 3:
            camera_info.k = [
            0] * 9
            for i in range(3):
                for j in range(3):
                    camera_info.k[i * 3 + j] = self.camera_matrix[i][j]

        # 设置畸变系数
        camera_info.d = self.distortion_coeffs

        return camera_info


@dataclass
class HandEyeTransform:
    """手眼变换矩阵"""
    translation: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # 米
    rotation: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # 欧拉角（弧度）
    quaternion: Optional[Tuple[float, float, float, float]] = None  # (w, x, y, z)

    def to_pose(self) -> Pose:
        """转换为 Pose 消息"""
        pose = Pose()

        pose.position.x = self.translation[0]
        pose.position.y = self.translation[1]
        pose.position.z = self.translation[2]

        if self.quaternion is not None:
            pose.orientation.w = self.quaternion[0]
            pose.orientation.x = self.quaternion[1]
            pose.orientation.y = self.quaternion[2]
            pose.orientation.z = self.quaternion[3]

        return pose


@dataclass
class CalibrationData:
    """标定数据对（机器人位姿 + 图像/标定板检测点）"""
    robot_pose: Pose
    image_path: Optional[str] = None
    detected_corners: Optional[List[Tuple[float, float]]] = None
    timestamp: float = 0.0


@dataclass
class CalibrationResult:
    """标定结果"""
    success: bool = False
    error_message: str = ""

    # 内参
    intrinsics: Optional[CameraIntrinsics] = None
    intrinsic_rms_error: float = 0.0

    # 外参
    hand_eye_transform: Optional[HandEyeTransform] = None
    extrinsic_rms_error: float = 0.0

    # 验证结果
    validation_pixel_errors: List[float] = field(default_factory=list)
    validation_physical_errors: List[float] = field(default_factory=list)
    avg_pixel_error: float = 0.0
    avg_physical_error: float = 0.0

    def print_summary(self):
        """打印标定结果摘要"""
        if self.success:
            print("\n标定成功！")
            print("=" * 50)

            if self.intrinsics is not None:
                print("\n内参标定结果:")
                print(f"  图像尺寸: {self.intrinsics.image_width} x {self.intrinsics.image_height}")
                print(f"  RMS 误差: {self.intrinsic_rms_error:.4f} 像素")

            if self.hand_eye_transform is not None:
                print("\n手眼标定结果:")
                print(f"  平移: ({self.hand_eye_transform.translation[0]:.4f}, "
                      f"{self.hand_eye_transform.translation[1]:.4f}, "
                      f"{self.hand_eye_transform.translation[2]:.4f}) m")
                print(f"  旋转: ({self.hand_eye_transform.rotation[0]:.4f}, "
                      f"{self.hand_eye_transform.rotation[1]:.4f}, "
                      f"{self.hand_eye_transform.rotation[2]:.4f}) rad")
                print(f"  RMS 误差: {self.extrinsic_rms_error:.4f}")

            if self.validation_pixel_errors:
                print("\n验证结果:")
                print(f"  平均像素误差: {self.avg_pixel_error:.4f} 像素")
                print(f"  平均物理误差: {self.avg_physical_error:.4f} mm")

            print("=" * 50)
        else:
            print(f"\n标定失败: {self.error_message}")
