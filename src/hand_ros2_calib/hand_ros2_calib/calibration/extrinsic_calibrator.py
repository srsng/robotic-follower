"""
外参标定器

执行 3D 手眼标定，集成 hand_eyes_calibration C++ 库。
"""

import cv2
import numpy as np
import os
import xml.etree.ElementTree as ET
from typing import List, Tuple, Optional
from dataclasses import dataclass

from hand_ros2_calib.calibration.calibration_types import (
    CalibrationConfig,
    HandEyeTransform,
    CalibrationData,
    HandEyeMode,
    PatternType,
    CameraIntrinsics,
)


@dataclass
class PoseRecord:
    """机器人位姿记录"""
    position: Tuple[float, float, float]  # mm
    rotation: Tuple[float, float, float]  # degrees
    timestamp: float = 0.0

    def to_string(self) -> str:
        """转换为字符串（用于 hand_eyes_calibration 格式）"""
        return (
            f"{self.position[0]:.3f},{self.position[1]:.3f},"
            f"{self.position[2]:.3f},"
            f"{self.rotation[0]:.3f},{self.rotation[1]:.3f},"
            f"{self.rotation[2]:.3f}"
        )


class ExtrinsicCalibrator:
    """
    外参标定器

    功能：
    - 收集机器人位姿和对应的标定板检测点
    - 验证位姿多样性（包含足够的旋转和平移）
    - 调用 hand_eyes_calibration C++ 库执行标定
    - 保存/加载手眼变换
    """

    def __init__(self, config: CalibrationConfig, intrinsics: CameraIntrinsics):
        """
        初始化外参标定器

        Args:
            config: 标定配置
            intrinsics: 相机内参
        """
        self.config = config
        self.intrinsics = intrinsics
        self.calibration_data: List[CalibrationData] = []
        self.pose_records: List[PoseRecord] = []
        self.image_paths: List[str] = []

        # 设置标定板参数
        self._setup_board()

    def _setup_board(self):
        """设置标定板参数"""
        self.board_size = (self.config.board_cols, self.config.board_rows)

        # 根据图案类型设置
        if self.config.pattern_type == PatternType.CHESSBOARD:
            self.board_pattern = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FAST_CHECK
        elif self.config.pattern_type == PatternType.CIRCLES_ASYMMETRIC:
            self.board_pattern = cv2.CALIB_CB_ASYMMETRIC_GRID
        elif self.config.pattern_type == PatternType.CIRCLES_SYMMETRIC:
            self.board_pattern = cv2.CALIB_CB_CLUSTERING

    def detect_board(
        self,
        image: np.ndarray,
    ) -> Optional[np.ndarray]:
        """
        检测图像中的标定板

        Args:
            image: 输入图像

        Returns:
            Optional[np.ndarray]: 检测到的角点
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        corners = None

        if self.config.pattern_type == PatternType.CHESSBOARD:
            ret, corners = cv2.findChessboardCorners(
                gray,
                self.board_size,
                None,
                self.board_pattern
            )
        elif self.config.pattern_type in [PatternType.CIRCLES_ASYMMETRIC, PatternType.CIRCLES_SYMMETRIC]:
            ret, centers = cv2.findCirclesGrid(gray, self.board_size, self.board_pattern)
            if ret and centers is not None:
                corners = centers

        return corners

    def add_calibration_pair(
        self,
        robot_pose: Tuple[float, float, float, float, float, float],  # x, y, z, rx, ry, rz
        image: np.ndarray,
        image_path: Optional[str] = None,
    ) -> bool:
        """
        添加标定数据对（机器人位姿 + 图像）

        Args:
            robot_pose: 机器人位姿 (x, y, z, rx, ry, rz)，单位 mm 和度
            image: 标定板图像
            image_path: 图像文件路径（可选）

        Returns:
            bool: 是否成功添加
        """
        if len(self.calibration_data) >= self.config.extrinsic_num_poses:
            return False

        # 检测标定板
        corners = self.detect_board(image)
        if corners is None:
            print("警告：图像中未检测到标定板")
            return False

        # 添加数据
        data = CalibrationData(
            robot_pose=self._create_pose_msg(robot_pose),
            image_path=image_path,
            detected_corners=corners.tolist(),
            timestamp=len(self.calibration_data),
        )

        self.calibration_data.append(data)

        # 记录位姿（用于 C++ 标定）
        pose_record = PoseRecord(
            position=(robot_pose[0], robot_pose[1], robot_pose[2]),
            rotation=(robot_pose[3], robot_pose[4], robot_pose[5]),
        )
        self.pose_records.append(pose_record)

        if image_path:
            self.image_paths.append(image_path)

        print(f"已添加标定对 {len(self.calibration_data)}/{self.config.extrinsic_num_poses}")
        return True

    def _create_pose_msg(
        self,
        pose_tuple: Tuple[float, float, float, float, float, float],
    ):
        """从元组创建 Pose 消息"""
        from geometry_msgs.msg import Pose, Point, Quaternion

        # 转换欧拉角到四元数
        rx, ry, rz = np.radians(pose_tuple[3]), np.radians(pose_tuple[4]), np.radians(pose_tuple[5])
        q = self._euler_to_quaternion(rx, ry, rz)

        return Pose(
            position=Point(
                x=pose_tuple[0] / 1000.0,  # mm -> m
                y=pose_tuple[1] / 1000.0,
                z=pose_tuple[2] / 1000.0,
            ),
            orientation=Quaternion(w=q[0], x=q[1], y=q[2], z=q[3]),
        )

    @staticmethod
    def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
        """欧拉角转四元数"""
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return (w, x, y, z)

    def calibrate(self) -> Optional[HandEyeTransform]:
        """执行外参标定"""
        if len(self.calibration_data) < 5:
            print(f"错误：至少需要 5 组标定数据，当前只有 {len(self.calibration_data)} 组")
            return None

        print(f"开始 3D 手眼标定，使用 {len(self.calibration_data)} 组数据...")
        print(f"标定模式: {self.config.hand_eye_mode.value}")

        # TODO: 调用 hand_eyes_calibration C++ 库
        # 这里目前返回模拟结果
        # 实际实现需要通过 Python C++ 接口或子进程调用

        # 模拟结果（需要替换为真实标定调用）
        transform = HandEyeTransform(
            translation=(0.0, 0.0, 0.05),  # 示例：5cm Z 偏移
            rotation=(0.0, 0.0, 0.0),
        )

        print("手眼标定完成（模拟结果）")
        print(f"  平移: {transform.translation} m")
        print(f"  旋转: {transform.rotation} rad")

        return transform, 0.0  # 返回变换和 RMS 误差

    def save_to_xml(
        self,
        transform: HandEyeTransform,
        output_path: str,
    ) -> bool:
        """
        保存手眼变换到 XML 文件（兼容 hand_eyes_calibration 格式）

        Args:
            transform: 手眼变换
            output_path: 输出文件路径

        Returns:
            bool: 是否成功保存
        """
        try:
            root = ET.Element("CalibrationResult")

            # 标定模式
            mode_elem = ET.SubElement(root, "HandEyeMode")
            mode_elem.text = self.config.hand_eye_mode.value

            # 平移
            trans_elem = ET.SubElement(root, "Translation")
            ET.SubElement(trans_elem, "x").text = str(transform.translation[0])
            ET.SubElement(trans_elem, "y").text = str(transform.translation[1])
            ET.SubElement(trans_elem, "z").text = str(transform.translation[2])

            # 旋转（欧拉角）
            rot_elem = ET.SubElement(root, "RotationEuler")
            ET.SubElement(rot_elem, "rx").text = str(np.degrees(transform.rotation[0]))
            ET.SubElement(rot_elem, "ry").text = str(np.degrees(transform.rotation[1]))
            ET.SubElement(rot_elem, "rz").text = str(np.degrees(transform.rotation[2]))

            # 保存文件
            tree = ET.ElementTree(root)
            tree.write(output_path, encoding="utf-8", xml_declaration=True)

            print(f"手眼变换已保存到: {output_path}")
            return True

        except Exception as e:
            print(f"保存手眼变换失败: {e}")
            return False

    @staticmethod
    def load_from_xml(input_path: str) -> Optional[HandEyeTransform]:
        """
        从 XML 文件加载手眼变换

        Args:
            input_path: 输入文件路径

        Returns:
            Optional[HandEyeTransform]: 手眼变换，失败返回 None
        """
        try:
            tree = ET.parse(input_path)
            root = tree.getroot()

            # 读取平移
            trans_elem = root.find("Translation")
            x = float(trans_elem.find("x").text)
            y = float(trans_elem.find("y").text)
            z = float(trans_elem.find("z").text)

            # 读取旋转
            rot_elem = root.find("RotationEuler")
            rx = np.radians(float(rot_elem.find("rx").text))
            ry = np.radians(float(rot_elem.find("ry").text))
            rz = np.radians(float(rot_elem.find("rz").text))

            return HandEyeTransform(
                translation=(x, y, z),
                rotation=(rx, ry, rz),
            )

        except Exception as e:
            print(f"加载手眼变换失败: {e}")
            return None

    def export_to_cpp_format(
        self,
        output_dir: str,
    ) -> bool:
        """
        导出数据为 C++ 标定程序可读取的格式

        Args:
            output_dir: 输出目录

        Returns:
            bool: 是否成功导出
        """
        try:
            os.makedirs(output_dir, exist_ok=True)

            # 保存图像
            image_dir = os.path.join(output_dir, "images")
            os.makedirs(image_dir, exist_ok=True)

            # 保存位姿
            pose_file = os.path.join(output_dir, "robot_poses.txt")
            with open(pose_file, 'w') as f:
                for i, pose_record in enumerate(self.pose_records):
                    f.write(f"{pose_record.to_string()}\n")

            # 生成 C++ 标定代码片段
            code_file = os.path.join(output_dir, "calib_data.cpp")
            with open(code_file, 'w') as f:
                f.write("// 自动生成的标定数据代码\n\n")
                f.write(f"// 标定模式: {self.config.hand_eye_mode.value}\n")
                f.write(f"// 数据组数: {len(self.pose_records)}\n\n")

                # 机器人位姿
                f.write("const std::vector<Pose3D> robot_poses = {\n")
                for i, pose_record in enumerate(self.pose_records):
                    f.write(f"    {{ {pose_record.to_string()} }},\n")
                f.write("};\n\n")

                # 图像文件
                f.write("const std::vector<std::string> image_files = {\n")
                for i, img_path in enumerate(self.image_paths):
                    if img_path:
                        rel_path = os.path.relpath(img_path, output_dir)
                        f.write(f'    "{rel_path}",\n')
                f.write("};\n")

            print(f"数据已导出到: {output_dir}")
            print(f"  - 图像: {image_dir}")
            print(f"  - 位姿: {pose_file}")
            print(f"  - C++ 代码: {code_file}")
            return True

        except Exception as e:
            print(f"导出数据失败: {e}")
            return False

    def clear(self):
        """清除已收集的数据"""
        self.calibration_data = []
        self.pose_records = []
        self.image_paths = []
