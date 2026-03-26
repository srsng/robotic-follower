#!/usr/bin/env python3
"""手眼标定启动文件。

启动完整的手眼标定流水线（Eye-in-Hand 模式）：
1. RealSense D435i 相机
2. ArUco 标定板检测（ros2_aruco）
3. 机械臂控制节点
4. 标定采样节点
5. 标定计算节点
6. 标定结果管理节点
7. TF 发布节点

使用场景：
    - 机械臂与相机之间的手眼标定
    - 标定完成后自动发布 TF 变换

启动命令：
    ros2 launch robotic_follower hand_eye_calibration.launch.py

标定流程：
    1. 启动标定：ros2 service call /hand_eye_calibration/start_sampling std_srvs/Trigger "{}"
    2. 移动机械臂采集样本（建议 15-50 组）
    3. 停止采集：ros2 service call /hand_eye_calibration/stop_sampling std_srvs/Trigger "{}"
    4. 执行标定：ros2 service call /hand_eye_calibration/execute std_srvs/Trigger "{}"
    5. 保存结果：ros2 service call /hand_eye_calibration/save_result std_srvs/Trigger "{}"
"""

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def generate_launch_description():
    """生成手眼标定系统的 Launch 描述。"""

    # 声明启动参数
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    # 标定板参数
    board_type = LaunchConfiguration("board_type", default="circles_asymmetric")
    board_cols = LaunchConfiguration("board_cols", default="4")
    board_rows = LaunchConfiguration("board_rows", default="5")
    circle_diameter = LaunchConfiguration("circle_diameter", default="0.020")

    # TF 坐标系
    parent_frame = LaunchConfiguration("parent_frame", default="link6_1_1")
    child_frame = LaunchConfiguration("child_frame", default="camera_link")

    # 1. RealSense 相机启动
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
                )
            ]
        ),
        launch_arguments={
            "align_depth.enable": "true",
            "enable_color": "true",
            "enable_depth": "true",
            "depth_module.profile": "640x480x30",
            "rgb_camera.profile": "640x480x30",
            "camera_namespace": "",
        }.items(),
    )

    # 2. ArUco 标定板检测
    aruco_node = Node(
        package="ros2_aruco",
        executable="aruco_node",
        name="aruco_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "marker_size": 0.020,
                "markers_ids": [0],
                "reference_frame": "camera_link",
            }
        ],
    )

    # 3. 机械臂控制节点
    arm_control_node = Node(
        package="robotic_follower",
        executable="arm_control",
        name="arm_control",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # 4. 标定采样节点
    calibration_sampler_node = Node(
        package="robotic_follower",
        executable="calibration_sampler",
        name="calibration_sampler",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "board_type": board_type,
                "board_cols": board_cols,
                "board_rows": board_rows,
                "min_sample_interval": 1.0,
                "auto_sample": True,
            }
        ],
    )

    # 5. 标定计算节点
    calibration_calculator_node = Node(
        package="robotic_follower",
        executable="calibration_calculator",
        name="calibration_calculator",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "min_samples": 15,
                "max_samples": 50,
                "method": "auto",
            }
        ],
    )

    # 6. 标定结果管理节点
    calibration_result_manager_node = Node(
        package="robotic_follower",
        executable="calibration_result_manager",
        name="calibration_result_manager",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "auto_save": True,
                "result_file": "",
            }
        ],
    )

    # 7. TF 发布节点
    calibration_tf_publisher_node = Node(
        package="robotic_follower",
        executable="calibration_tf_publisher",
        name="calibration_tf_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "parent_frame": parent_frame,
                "child_frame": child_frame,
                "publish_rate": 10.0,
                "config_namespace": "hand_eye_calibration",
            }
        ],
    )

    # 8. Open3D 可视化节点（带标定面板）
    open3d_visualizer_node = Node(
        package="robotic_follower",
        executable="open3d_visualizer",
        name="open3d_visualizer",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "enable_calibration_panel": True,
            }
        ],
    )

    return LaunchDescription(
        [
            # 声明参数
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Use simulation time"
            ),
            DeclareLaunchArgument(
                "board_type",
                default_value="circles_asymmetric",
                description="Calibration board type",
            ),
            DeclareLaunchArgument(
                "board_cols", default_value="4", description="Board columns"
            ),
            DeclareLaunchArgument(
                "board_rows", default_value="5", description="Board rows"
            ),
            DeclareLaunchArgument(
                "circle_diameter",
                default_value="0.020",
                description="Circle diameter (meters)",
            ),
            DeclareLaunchArgument(
                "parent_frame",
                default_value="link6_1_1",
                description="Parent frame for TF",
            ),
            DeclareLaunchArgument(
                "child_frame",
                default_value="camera_link",
                description="Child frame for TF",
            ),
            # 启动节点
            realsense_launch,
            aruco_node,
            arm_control_node,
            calibration_sampler_node,
            calibration_calculator_node,
            calibration_result_manager_node,
            calibration_tf_publisher_node,
            open3d_visualizer_node,
        ]
    )
