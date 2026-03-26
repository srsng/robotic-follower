#!/usr/bin/env python3
"""手眼标定启动文件。

启动完整的手眼标定流水线（Eye-in-Hand 模式）：
1. 静态 TF (world → base_link)
2. Robot State Publisher
3. MoveGroup (MoveIt2)
4. RealSense D435i 相机
5. ArUco 标定板检测
6. 关节状态重映射节点
7. 机械臂控制节点
8. 标定采样节点
9. 标定计算节点
10. 标定结果管理节点
11. TF 发布节点
12. Open3D 可视化

使用：
    ros2 launch robotic_follower hand_eye_calibration.launch.py

标定流程：
    1. 启动标定：ros2 service call /hand_eye_calibration/start_sampling std_srvs/Trigger "{}"
    2. 在 RViz 中移动机械臂采集样本（建议 15-50 组）
    3. 停止采集：ros2 service call /hand_eye_calibration/stop_sampling std_srvs/Trigger "{}"
    4. 执行标定：ros2 service call /hand_eye_calibration/execute std_srvs/Trigger "{}"
    5. 保存结果：ros2 service call /hand_eye_calibration/save_result std_srvs/Trigger "{}"
"""

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

from launch import LaunchDescription


def generate_launch_description():
    """生成手眼标定系统的 Launch 描述。"""

    # 声明启动参数
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    # 标定板参数
    board_type = LaunchConfiguration("board_type", default="aruco")
    marker_size = LaunchConfiguration("marker_size", default="0.05")
    marker_id = LaunchConfiguration("marker_id", default="0")

    # TF 坐标系
    parent_frame = LaunchConfiguration("parent_frame", default="link6_1_1")
    child_frame = LaunchConfiguration("child_frame", default="camera_link")

    # MoveIt2 配置
    moveit_config = (
        MoveItConfigsBuilder("dummy-ros2", package_name="dummy_moveit_config")
        .robot_description(file_path="config/dummy-ros2.urdf.xacro")
        .robot_description_semantic(file_path="config/dummy-ros2.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # 1. 静态 TF: world -> base_link
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # 2. Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # 3. MoveGroup node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    # 4. RealSense 相机启动
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

    # 5. ArUco 标定板检测
    aruco_node = Node(
        package="ros2_aruco",
        executable="aruco_node",
        name="aruco_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "marker_size": 0.05,
                "markers_ids": [0],
                "reference_frame": "camera_link",
            }
        ],
    )

    # 6. 关节状态重映射节点
    joint_state_remapper = Node(
        package="robotic_follower",
        executable="joint_state_remapper",
        name="joint_state_remapper",
        output="screen",
        parameters=[
            {"input_topic": "/joint_states"},
            {"output_topic": "/robotic_follower/joint_states"},
        ],
    )

    # 7. 机械臂控制节点
    arm_control_node = Node(
        package="robotic_follower",
        executable="arm_control",
        name="arm_control",
        output="screen",
        parameters=[
            {
                "group_name": "dummy_arm",
                "base_frame": "base_link",
                "end_effector_frame": "link6_1_1",
                "max_velocity": 0.3,
                "max_acceleration": 0.3,
            }
        ],
    )

    # 8. 标定采样节点
    calibration_sampler_node = Node(
        package="robotic_follower",
        executable="calibration_sampler",
        name="calibration_sampler",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "board_type": "aruco",
                "min_sample_interval": 1.5,
                "auto_sample": True,
            }
        ],
    )

    # 9. 标定计算节点
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

    # 10. 标定结果管理节点
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

    # 11. TF 发布节点
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

    return LaunchDescription(
        [
            # 声明参数
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Use simulation time"
            ),
            DeclareLaunchArgument(
                "board_type",
                default_value="aruco",
                description="Calibration board type",
            ),
            DeclareLaunchArgument(
                "marker_size",
                default_value="0.05",
                description="ArUco marker size (meters)",
            ),
            DeclareLaunchArgument(
                "marker_id",
                default_value="0",
                description="ArUco marker ID",
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
            # MoveIt2 基础设施
            static_tf,
            robot_state_publisher,
            move_group_node,
            # 相机与标定
            realsense_launch,
            aruco_node,
            # 机械臂控制
            joint_state_remapper,
            arm_control_node,
            # 标定模块
            calibration_sampler_node,
            calibration_calculator_node,
            calibration_result_manager_node,
            calibration_tf_publisher_node,
            # 可视化
            Node(
                package="robotic_follower",
                executable="open3d_visualizer",
                name="open3d_visualizer",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"enable_calibration_panel": True},
                ],
            ),
        ]
    )
