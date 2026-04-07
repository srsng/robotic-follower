#!/usr/bin/env python3
"""手眼标定启动文件。

启动完整的手眼标定系统：
1. 机械臂驱动 + MoveIt (demo_real_arm.launch.py)
2. RealSense 相机 (写死参数)
3. 棋盘格标定板检测 (GP290: 12x9, 单格2cm)
4. 标定节点 (sampler/calculator/result_manager/tf_publisher)

启动命令：
    ros2 launch robotic_follower hand_eye_calibration.launch.py
"""

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def generate_launch_description():
    """生成手眼标定系统的 Launch 描述。"""

    # 1. Include demo_real_arm.launch.py (MoveIt + RViz + TF，不包含机械臂控制器)
    demo_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("dummy_moveit_config"), "/launch/demo_real_arm.launch.py"]
        )
    )

    # 1.5 启动机械臂控制器（必须在 MoveIt 之前启动，以便接收关节状态）
    arm_controller_node = Node(
        package="dummy_controller",
        executable="dummy_arm_controller",
        name="dummy_arm_controller",
        output="screen",
        parameters=[
            {
                "publish_rate": 100,
            }
        ],
    )

    # 2. RealSense 相机（写死参数：640x480x30、点云启用、对齐深度）
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
            "camera_namespace": "",
        }.items(),
    )

    # 3. 棋盘格标定板检测节点 (GP290: 12x9, 单格2cm)
    chessboard_pose_node = Node(
        package="robotic_follower",
        executable="chessboard_pose",
        name="chessboard_pose_node",
        output="screen",
        parameters=[
            {
                "chessboard_cols": 11,
                "chessboard_rows": 8,
                "square_size": 0.02,
            }
        ],
    )

    # 4. 标定采样节点
    sampler_node = Node(
        package="robotic_follower",
        executable="calibration_sampler",
        name="calibration_sampler",
        output="screen",
        parameters=[
            {
                "min_samples": 15,
                "max_samples": 50,
                "stable_wait": 1.0,
                "position_threshold": 0.05,
                "rotation_threshold": 5.0,
            }
        ],
    )

    # 5. 标定计算节点
    calculator_node = Node(
        package="robotic_follower",
        executable="calibration_calculator",
        name="calibration_calculator",
        output="screen",
        parameters=[
            {
                "min_samples": 15,
                "max_samples": 50,
            }
        ],
    )

    # 6. 标定结果管理节点
    result_manager_node = Node(
        package="robotic_follower",
        executable="calibration_result_manager",
        name="calibration_result_manager",
        output="screen",
    )

    # 7. TF 发布节点
    tf_publisher_node = Node(
        package="robotic_follower",
        executable="calibration_tf_publisher",
        name="calibration_tf_publisher",
        output="screen",
        parameters=[
            {
                "parent_frame": "link6_1_1",
                "child_frame": "camera_link",
                "publish_rate": 10.0,
                "config_namespace": "hand_eye_calibration",
            }
        ],
    )

    # 8. 标定 info UI 窗口
    calibration_ui_node = Node(
        package="robotic_follower",
        executable="calibration_ui",
        name="calibration_ui",
        output="screen",
    )

    return LaunchDescription(
        [
            # 声明启动参数
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Use simulation time"
            ),
            # 启动组件
            arm_controller_node,
            demo_arm_launch,
            realsense_launch,
            chessboard_pose_node,
            sampler_node,
            calculator_node,
            result_manager_node,
            tf_publisher_node,
            calibration_ui_node,
        ]
    )
