"""
坐标系名称配置

集中管理项目中的所有坐标系命名，避免硬编码。
"""

class FrameNames:
    """坐标系名称定义"""

    # ========== 全局坐标系 ==========
    WORLD = "world"
    MAP = "map"
    ODOM = "odom"

    # ========== 机械臂坐标系 ==========
    BASE_LINK = "base_link"
    LINK1 = "link1_1_1"
    LINK2 = "link2_1_1"
    LINK3 = "link3_1_1"
    LINK4 = "link4_1_1"
    LINK5 = "link5_1_1"
    LINK6 = "link6_1_1"  # 机械臂末端执行器

    # ========== RealSense D435 相机坐标系 ==========
    D435_LINK = "d435_link"
    D435_BASE = "d435_base_link"

    # D435 红外相机坐标系
    D435_INFRA1_LINK = "d435_infra1_link"
    D435_INFRA1_FRAME = "d435_infra1_frame"
    D435_INFRA1_OPTICAL = "d435_infra1_optical_frame"

    D435_INFRA2_LINK = "d435_infra2_link"
    D435_INFRA2_FRAME = "d435_infra2_frame"
    D435_INFRA2_OPTICAL = "d435_infra2_optical_frame"

    # D435 深度相机坐标系
    D435_DEPTH_LINK = "d435_depth_link"
    D435_DEPTH_FRAME = "d435_depth_frame"
    D435_DEPTH_OPTICAL = "d435_depth_optical_frame"

    # D435 彩色相机坐标系
    D435_COLOR_LINK = "d435_color_link"
    D435_COLOR_FRAME = "d435_color_frame"
    D435_COLOR_OPTICAL = "d435_color_optical_frame"

    # D435 点云坐标系
    D435_POINTS = "d435_points"

    # ========== 工具坐标系（自定义） ==========
    TOOL_FRAME = "tool_frame"          # 工具中心点 (TCP)
    GRIPPER_FRAME = "gripper_frame"    # 夹爪中心
    WRIST_FRAME = "wrist_frame"        # 腕部

    # ========== 手眼标定相关坐标系 ==========
    CAMERA_MOUNT = "camera_mount"       # 相机安装座
    HAND_EYE_MARKER = "hand_eye_marker" # 手眼标定标记

    # ========== 物体坐标系 ==========
    OBJECT_FRAME = "object_frame"       # 被操作物体
    TARGET_POSE = "target_pose"         # 目标位姿
    GRASP_POSE = "grasp_pose"          # 抓取位姿

    # ========== 移动平台坐标系（如果存在） ==========
    MOBILE_BASE = "mobile_base"
    FOOTPRINT = "base_footprint"

    @staticmethod
    def get_robot_chain_frames() -> list:
        """获取机械臂坐标系链"""
        return [
            FrameNames.BASE_LINK,
            FrameNames.LINK1,
            FrameNames.LINK2,
            FrameNames.LINK3,
            FrameNames.LINK4,
            FrameNames.LINK5,
            FrameNames.LINK6,
        ]

    @staticmethod
    def get_d435_frames() -> list:
        """获取 D435 相机所有坐标系"""
        return [
            FrameNames.D435_LINK,
            FrameNames.D435_BASE,
            FrameNames.D435_INFRA1_LINK,
            FrameNames.D435_INFRA1_FRAME,
            FrameNames.D435_INFRA1_OPTICAL,
            FrameNames.D435_INFRA2_LINK,
            FrameNames.D435_INFRA2_FRAME,
            FrameNames.D435_INFRA2_OPTICAL,
            FrameNames.D435_DEPTH_LINK,
            FrameNames.D435_DEPTH_FRAME,
            FrameNames.D435_DEPTH_OPTICAL,
            FrameNames.D435_COLOR_LINK,
            FrameNames.D435_COLOR_FRAME,
            FrameNames.D435_COLOR_OPTICAL,
        ]

    @staticmethod
    def get_all_frames() -> list:
        """获取所有定义的坐标系"""
        return [
            # 全局坐标系
            FrameNames.WORLD,
            FrameNames.MAP,
            FrameNames.ODOM,
            # 机械臂坐标系
            *FrameNames.get_robot_chain_frames(),
            # D435 相机坐标系
            *FrameNames.get_d435_frames(),
            # 工具坐标系
            FrameNames.TOOL_FRAME,
            FrameNames.GRIPPER_FRAME,
            FrameNames.WRIST_FRAME,
            # 手眼标定坐标系
            FrameNames.CAMERA_MOUNT,
            FrameNames.HAND_EYE_MARKER,
            # 物体坐标系
            FrameNames.OBJECT_FRAME,
            FrameNames.TARGET_POSE,
            FrameNames.GRASP_POSE,
            # 移动平台坐标系
            FrameNames.MOBILE_BASE,
            FrameNames.FOOTPRINT,
        ]


class TFTransformTimeout:
    """TF 变换超时配置"""

    DEFAULT_TIMEOUT = 1.0  # 默认超时 1 秒
    LONG_TIMEOUT = 5.0     # 长超时 5 秒
    SHORT_TIMEOUT = 0.1     # 短超时 0.1 秒
