"""手眼标定算法实现。

使用 cv2.calibrateHandEye 求解 Eye-in-Hand (眼在手上) 手眼标定问题。

标定类型：Eye-in-Hand
- 相机固联于机械臂末端
- 标定板固定在空间中
- 求解 AX=XB，获得 camera2gripper（相机在末端坐标系下的位姿）

TF 关系：link6_1_1 (末端) → camera_link (相机)

Args:
    robot_poses: 末端执行器相对基座的位姿列表，每个为 4x4 矩阵 (gripper2base)
    camera_poses: 标定板相对相机的位姿列表，每个为 4x4 矩阵 (marker2camera)

Returns:
    rotation_matrix: 3x3 旋转矩阵
    translation_vector: 3x1 平移向量
    quaternion: [x, y, z, w] 四元数
    error: 重投影误差
"""

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from robotic_follower.util.log import log


# 可用的标定方法
HAND_EYE_METHODS = {
    "TSAI": cv2.CALIB_HAND_EYE_TSAI,
    "PARK": cv2.CALIB_HAND_EYE_PARK,
    "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
    "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
    "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def compute_motion(T1: np.ndarray, T2: np.ndarray) -> np.ndarray:
    """计算相邻位姿之间的相对变换 T_rel = T1^-1 @ T2。

    Args:
        T1: 上一时刻的位姿 4x4 矩阵
        T2: 当前时刻的位姿 4x4 矩阵

    Returns:
        T_rel: 相对变换 4x4 矩阵
    """
    return np.linalg.inv(T1) @ T2


def matrix_to_rotation_vector(R: np.ndarray) -> np.ndarray:
    """将 3x3 旋转矩阵转换为 3x1 旋转向量。

    Args:
        R: 3x3 旋转矩阵

    Returns:
        rv: 3x1 旋转向量 (罗德里格斯公式的向量形式)
    """
    return cv2.Rodrigues(R)[0]


def calibrate_handeye(
    robot_poses: list[np.ndarray],
    camera_poses: list[np.ndarray],
    method: str = "TSAI",
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """执行手眼标定。

    Args:
        robot_poses: 末端执行器相对基座的位姿列表，每个为 4x4 矩阵 (gripper2base)
        camera_poses: 标定板相对相机的位姿列表，每个为 4x4 矩阵 (marker2camera)
        method: 标定方法，可选 TSAI/PARK/HORAUD/ANDREFF/DANIILIDIS

    Returns:
        rotation_matrix: 3x3 旋转矩阵 (gripper2camera)
        translation_vector: 3x1 平移向量 (gripper2camera)
        quaternion: [x, y, z, w] 四元数
        error: 自洽性误差（标定板在基座坐标系下位姿的方差）
    """
    if len(robot_poses) != len(camera_poses):
        raise ValueError(
            f"robot_poses 和 camera_poses 数量不一致: {len(robot_poses)} vs {len(camera_poses)}"
        )

    if len(robot_poses) < 2:
        raise ValueError(f"样本数不足，至少需要2组样本，当前: {len(robot_poses)}")

    # 计算相邻位姿之间的相对变换
    A_motions = []  # 机械臂末端相对变换 (gripper2base[i]^-1 @ gripper2base[i+1])
    B_motions = []  # 相机相对标定板变换 (camera2marker[i] @ camera2marker[i+1]^-1)

    for i in range(len(robot_poses) - 1):
        # A_motion: T_{i}->{i+1} in gripper frame
        # 对于 eye-in-hand: A = gripper2base[i]^-1 @ gripper2base[i+1]
        A = compute_motion(robot_poses[i], robot_poses[i + 1])
        A_motions.append(A)

        # B_motion: T_{i}->{i+1} in marker frame
        # 对于 eye-in-hand: B = marker2camera[i]^-1 @ marker2camera[i+1]
        # 即 inv(camera_pose[i]) @ camera_pose[i+1]
        B = compute_motion(camera_poses[i], camera_poses[i + 1])
        B_motions.append(B)

    # 提取旋转矩阵和平移向量
    R_gripper2base = []  # A 的旋转部分
    t_gripper2base = []  # A 的平移部分
    R_marker2camera = []  # B 的旋转部分
    t_marker2camera = []  # B 的平移部分

    for A, B in zip(A_motions, B_motions):
        R_gripper2base.append(A[:3, :3])
        t_gripper2base.append(A[:3, 3])
        R_marker2camera.append(B[:3, :3])
        t_marker2camera.append(B[:3, 3])

    # 转换为旋转向量 (cv2.calibrateHandEye 需要旋转向量)
    R_gripper2base_vec = [matrix_to_rotation_vector(R) for R in R_gripper2base]
    R_marker2camera_vec = [matrix_to_rotation_vector(R) for R in R_marker2camera]

    # 执行标定
    if method not in HAND_EYE_METHODS:
        raise ValueError(
            f"未知标定方法: {method}，可用: {list(HAND_EYE_METHODS.keys())}"
        )

    method_id = HAND_EYE_METHODS[method]

    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base_vec,
        t_gripper2base,
        R_marker2camera_vec,
        t_marker2camera,
        method=method_id,
    )

    # 旋转向量转旋转矩阵
    R_cam2gripper_matrix = cv2.Rodrigues(R_cam2gripper)[0]

    # 转置得到 gripper2camera（标定结果表示相机在末端坐标系下的位姿）
    # 根据 cv2.calibrateHandEye 的定义，对于 eye-in-hand:
    # R_cam2gripper 是 camera 相对于 gripper 的旋转
    # 所以 R_gripper2camera = R_cam2gripper^T
    R_gripper2camera = R_cam2gripper_matrix.T
    t_gripper2camera = -R_gripper2camera @ t_cam2gripper.flatten()

    # 转换为四元数
    quaternion = Rotation.from_matrix(R_gripper2camera).as_quat()  # [x, y, z, w]

    # 计算重投影误差
    error = compute_calibration_error(
        robot_poses, camera_poses, R_gripper2camera, t_gripper2camera
    )

    return R_gripper2camera, t_gripper2camera, quaternion, error


def compute_calibration_error(
    robot_poses: list[np.ndarray],
    camera_poses: list[np.ndarray],
    R_gripper2camera: np.ndarray,
    t_gripper2camera: np.ndarray,
) -> float:
    """计算手眼标定的自洽性误差。

    通过计算标定板在机械臂基座坐标系下的位姿一致性来评估标定精度。
    标定板是固定在空间中的，所以对于所有样本：
        base2marker = gripper2base @ gripper2camera @ marker2camera
    应该保持一致。

    Args:
        robot_poses: 机械臂末端位姿列表 (gripper2base)
        camera_poses: 相机观测到的标定板位姿列表 (marker2camera)
        R_gripper2camera: gripper 相对于 camera 的旋转矩阵（标定结果）
        t_gripper2camera: gripper 相对于 camera 的平移向量（标定结果）

    Returns:
        error: 平均位姿方差 (米)
    """
    # 构造 camera2gripper（标定结果的逆）
    camera2gripper = np.eye(4)
    camera2gripper[:3, :3] = R_gripper2camera.T
    camera2gripper[:3, 3] = -R_gripper2camera.T @ t_gripper2camera

    # 计算每个样本的 base2marker
    base2marker_poses = []
    for gripper2base, marker2camera in zip(robot_poses, camera_poses):
        # base2marker = gripper2base @ gripper2camera @ marker2camera
        gripper2camera_mat = np.eye(4)
        gripper2camera_mat[:3, :3] = R_gripper2camera
        gripper2camera_mat[:3, 3] = t_gripper2camera

        base2marker = gripper2base @ gripper2camera_mat @ marker2camera
        base2marker_poses.append(base2marker)

    # 计算 base2marker 的平移部分的方差
    translations = np.array([p[:3, 3] for p in base2marker_poses])
    mean_translation = translations.mean(axis=0)
    variance = np.sum((translations - mean_translation) ** 2) / len(translations)
    std_error = np.sqrt(variance)

    return float(std_error)


def find_best_calibration_method(
    robot_poses: list[np.ndarray],
    camera_poses: list[np.ndarray],
) -> tuple[str, np.ndarray, np.ndarray, np.ndarray, float]:
    """尝试多种标定方法，返回误差最小的那一个。

    Args:
        robot_poses: 机械臂末端位姿列表
        camera_poses: 相机观测到的标定板位姿列表

    Returns:
        best_method: 最佳标定方法名称
        rotation_matrix: 最佳旋转矩阵
        translation_vector: 最佳平移向量
        quaternion: 最佳四元数
        min_error: 最小误差
    """
    results = {}

    for method_name in HAND_EYE_METHODS:
        try:
            R, t, q, error = calibrate_handeye(robot_poses, camera_poses, method_name)
            results[method_name] = {
                "rotation_matrix": R,
                "translation_vector": t,
                "quaternion": q,
                "error": error,
            }
        except Exception as e:
            log("warn", f"方法 {method_name} 失败: {e}")
            continue

    if not results:
        raise RuntimeError("所有标定方法都失败了")

    # 选择误差最小的方法
    best_method = min(results.keys(), key=lambda m: results[m]["error"])
    best_result = results[best_method]

    return (
        best_method,
        best_result["rotation_matrix"],
        best_result["translation_vector"],
        best_result["quaternion"],
        best_result["error"],
    )


class ExtrinsicCalibrator:
    """手眼标定器类。"""

    def __init__(self, min_samples: int = 15):
        """初始化标定器。

        Args:
            min_samples: 最少样本数
        """
        self.min_samples = min_samples
        self.robot_poses: list[np.ndarray] = []
        self.camera_poses: list[np.ndarray] = []

    def add_sample(self, robot_pose: np.ndarray, camera_pose: np.ndarray) -> bool:
        """添加一个样本。

        Args:
            robot_pose: 末端执行器位姿 4x4 矩阵 (gripper2base)
            camera_pose: 标定板位姿 4x4 矩阵 (marker2camera)

        Returns:
            success: 是否添加成功
        """
        if robot_pose.shape != (4, 4) or camera_pose.shape != (4, 4):
            return False

        self.robot_poses.append(robot_pose)
        self.camera_poses.append(camera_pose)
        return True

    def clear(self):
        """清空所有样本。"""
        self.robot_poses.clear()
        self.camera_poses.clear()

    @property
    def sample_count(self) -> int:
        """返回当前样本数。"""
        return len(self.robot_poses)

    def calibrate(self, method: str = "TSAI") -> dict:
        """执行标定。

        Args:
            method: 标定方法，"TSAI"/"PARK"/"HORAUD"/"ANDREFF"/"DANIILIDIS"，或 "BEST" 自动选择最佳

        Returns:
            结果字典，包含 rotation_matrix, translation_vector, quaternion, error

        Raises:
            ValueError: 样本数不足
        """
        if self.sample_count < self.min_samples:
            raise ValueError(
                f"样本数不足，需要至少 {self.min_samples} 个样本，当前: {self.sample_count}"
            )

        if method == "BEST":
            best_method, R, t, q, error = find_best_calibration_method(
                self.robot_poses, self.camera_poses
            )
        else:
            R, t, q, error = calibrate_handeye(
                self.robot_poses, self.camera_poses, method
            )
            best_method = method

        return {
            "rotation_matrix": R.tolist(),
            "translation_vector": t.tolist(),
            "quaternion": q.tolist(),  # [x, y, z, w]
            "error": float(error),
            "method": best_method,
            "sample_count": self.sample_count,
        }

    def validate(self, result: dict, max_error: float = 0.01) -> dict:
        """验证标定结果。

        Args:
            result: 标定结果
            max_error: 最大允许误差（米）

        Returns:
            验证结果字典
        """
        error = result.get("error", float("inf"))
        passed = error < max_error

        R = np.array(result["rotation_matrix"])
        t = np.array(result["translation_vector"])

        # 计算旋转误差（使用迹来评估旋转矩阵的正交性）
        I = np.eye(3)
        orthogonality_error = np.linalg.norm(R @ R.T - I, "fro")

        return {
            "passed": passed,
            "error": error,
            "max_error": max_error,
            "orthogonality_error": float(orthogonality_error),
        }
