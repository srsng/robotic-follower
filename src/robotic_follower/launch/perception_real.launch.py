#!/usr/bin/env python3
"""实机感知启动文件。

启动完整的感知流水线用于实机测试：
1. RealSense D435i 相机
2. 点云处理器
3. 3D 检测器
4. RViz 可视化

使用场景：
    - 实机环境下的感知算法测试
    - 验证相机和检测算法
    - 性能基准测试

启动命令：
    ros2 launch robotic_follower perception_real.launch.py
"""

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def generate_launch_description():
    """生成感知系统的 Launch 描述。"""

    # 声明启动参数
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

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

    # 2. 点云处理器
    pointcloud_processor_node = Node(
        package="robotic_follower",
        executable="pointcloud_processor",
        name="pointcloud_processor",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # 3. 3D 检测器
    detection_node = Node(
        package="robotic_follower",
        executable="detection_node",
        name="detection_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "config_file": config_file,
            }
        ],
    )

    # 4. RViz 可视化
    rviz_visualizer_node = Node(
        package="robotic_follower",
        executable="rviz_visualizer",
        name="rviz_visualizer",
        output="screen",
    )

    return LaunchDescription(
        [
            # 声明参数
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Use simulation time"
            ),
            # 启动节点
            realsense_launch,
            pointcloud_processor_node,
            detection_node,
            rviz_visualizer_node,
        ]
    )
