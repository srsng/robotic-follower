"""标定板检测。"""

import cv2
import numpy as np


class BoardDetector:
    """标定板检测器。"""

    def __init__(
        self,
        board_type: str = "circles_asymmetric",
        board_cols: int = 4,
        board_rows: int = 5,
        circle_diameter: float = 0.020  # meters
    ):
        """初始化标定板检测器。

        Args:
            board_type: 标定板类型
            board_cols: 列数
            board_rows: 行数
            circle_diameter: 圆形直径（米）
        """
        self.board_type = board_type
        self.board_cols = board_cols
        self.board_rows = board_rows
        self.circle_diameter = circle_diameter

        # 创建标定定板对象
        if board_type == "circles_asymmetric":
            # 使用OpenCV 4.x API
            dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
            self.board = cv2.aruco.CharucoBoard(
                (board_cols, board_rows),
                circle_diameter,
                circle_diameter * 0.5,
                dictionary
            )
        else:
            raise ValueError(f"不支持的标定板类型: {board_type}")

    def detect(self, image: np.ndarray) -> np.ndarray:
        """检测标定板。

        Args:
            image: 输入图像

        Returns:
            标定板位姿（4x4齐次变换矩阵），检测失败返回None
        """
        # 转换为灰度图
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image

        # 检测标定板
        detector = cv2.aruco.ArucoDetector(self.board.getDictionary())
        corners, ids, rejected = detector.detectMarkers(gray)

        if len(corners) == 0:
            return None

        # 拒接插值
        charuco_obj = cv2.aruco.CharucoDetector(self.board)
        charuco_corners, charuco_ids, marker_corners, marker_ids = charuco_obj.detectBoard(gray)

        if charuco_corners is None or len(charuco_corners) < 4:
            return None

        # 估计位姿（需要相机内参）
        # 注意：这里简化处理，实际需要相机内参才能准确估计位姿
        # 返回None表示需要外部提供相机内参
        return None

    def detect_with_camera_matrix(
        self,
        image: np.ndarray,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray
    ) -> np.ndarray:
        """检测标定板并估计位姿（提供相机内参）。

        Args:
            image: 输入图像
            camera_matrix: 相机内参矩阵（3x3）
            dist_coeffs: 畸变系数

        Returns:
            标定板位姿（4x4齐次变换矩阵），检测失败返回None
        """
        # 转换为灰度图
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image

        # 检测标定板
        detector = cv2.aruco.ArucoDetector(self.board.getDictionary())
        corners, ids, rejected = detector.detectMarkers(gray)

        if len(corners) == 0:
            return None

        # 拒接插值
        charuco_obj = cv2.aruco.CharucoDetector(self.board)
        charuco_corners, charuco_ids, marker_corners, marker_ids = charuco_obj.detectBoard(gray)

        if charuco_corners is None or len(charuco_corners) < 4:
            return None

        # 估计位姿
        charuco_params = cv2.aruco.CharucoParameters()
        rvec, tvec = self.board.estimatePoseCharucoBoard(
            charuco_corners, charuco_ids,
            camera_matrix, dist_coeffs, charuco_params
        )

        if rvec is None or tvec is None:
            return None

        # 转换为旋转矩阵
        R, _ = cv2.Rodrigues(rvec)

        # 构建4x4齐次变换矩阵
        pose = np.eye(4)
        pose[:3, :3] = R
        pose[:3, 3] = tvec.flatten()

        return pose
