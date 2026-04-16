#!/usr/bin/env python3
"""基于 easy_handeye2 的手眼标定 launch 文件。

使用棋盘格进行眼在手上（eye-in-hand）手眼标定。

启动组件：
1. 机械臂 MoveIt + 控制器 + robot_state_publisher
2. RealSense 相机
3. 棋盘格检测 → 发布 TF (供 easy_handeye2 使用)
4. easy_handeye2 标定服务
5. rqt_image_view (图像可视化)

启动命令：
    ros2 launch robotic_follower hand_eye_easy_calibration.launch.py
"""

from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def generate_launch_description():
    # 标定命名空间
    calib_name = "dummy_handeye"

    # TF 帧名
    robot_base_frame = "base_link"
    robot_effector_frame = "link6_1_1"
    tracking_base_frame = "camera_color_optical_frame"
    tracking_marker_frame = "chessboard_marker"

    # 2. Include demo_real_arm.launch.py (MoveIt + RViz + robot_state_publisher 发布机械臂 TF)
    demo_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("dummy_moveit_config"), "/launch/demo_real_arm.launch.py"]
        )
    )

    # 3. 机械臂控制器
    arm_controller_node = Node(
        package="dummy_controller",
        executable="dummy_arm_controller",
        name="dummy_arm_controller",
        output="screen",
    )

    # 4. RealSense 相机（与 perception_real.launch.py 保持一致）
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("realsense2_camera"), "/launch/rs_launch.py"]
        ),
        launch_arguments={
            "align_depth.enable": "true",
            "pointcloud.enable": "true",
            "enable_color": "true",
            "enable_depth": "true",
            "depth_module.profile": "640x480x30",
            "rgb_camera.profile": "640x480x30",
            "camera_namespace": "camera",
            "publish_tf": "true",
        }.items(),
    )

    # 5. 棋盘格检测 → 发布 TF
    # 订阅话题: /camera/color/image_raw, /camera/color/camera_info
    chessboard_tf_node = Node(
        package="robotic_follower",
        executable="chessboard_tf",
        name="chessboard_tf_node",
        output="screen",
        parameters=[
            {
                "chessboard_cols": 11,
                "chessboard_rows": 8,
                "square_size": 0.02,
                "tracking_base_frame": tracking_base_frame,
                "tracking_marker_frame": tracking_marker_frame,
            }
        ],
    )

    # 6. rqt_image_view 图像可视化
    rqt_image_view_node = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="rqt_image_view",
        arguments=["/camera/camera/color/image_raw"],
        output="screen",
    )

    # 7. easy_handeye2 标定
    easy_calibrate_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("easy_handeye2"), "/launch/calibrate.launch.py"]
        ),
        launch_arguments={
            "name": calib_name,
            "calibration_type": "eye_in_hand",
            "robot_base_frame": robot_base_frame,
            "robot_effector_frame": robot_effector_frame,
            "tracking_base_frame": tracking_base_frame,
            "tracking_marker_frame": tracking_marker_frame,
        }.items(),
    )

    return LaunchDescription(
        [
            # 机械臂 + MoveIt + RViz（包含 robot_state_publisher 发布机械臂 TF）
            demo_arm_launch,
            TimerAction(period=5.0, actions=[arm_controller_node]),
            # 相机（publish_tf=true，发布相机 TF）
            realsense_launch,
            # 棋盘格检测
            TimerAction(period=7.0, actions=[chessboard_tf_node]),
            # rqt_image_view 图像可视化
            TimerAction(period=2.0, actions=[rqt_image_view_node]),
            # easy_handeye2 标定 GUI (包含 handeye_server + rqt_calibrator)
            TimerAction(period=10.0, actions=[easy_calibrate_launch]),
        ]
    )
