#!/usr/bin/env python3
"""实机感知启动文件。

启动完整的感知流水线用于实机测试：
1. RealSense D435i 相机（原生点云 + TF 模型发布）
2. 3D 检测器（消费 realsense 原生点云）
3. RViz 可视化（点云 + 检测包围盒）

架构说明：
    realsense2_camera 发布点云 /camera/camera/depth/color/points
    → detection_node 订阅，执行 3D 检测
    → rviz_visualizer_node 转为 MarkerArray
    → RViz 显示点云和包围盒

TF 说明：
    robot_state_publisher 根据 D435i URDF 发布相机 TF
    → 供 rviz2 正确关联点云坐标系

启动命令：
    ros2 launch robotic_follower perception_real.launch.py
"""

import os
import tempfile
import xacro

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def to_urdf(xacro_path, parameters=None):
    """将 xacro 文件转换为 URDF 文件。"""
    urdf_path = tempfile.mktemp(prefix="%s_" % os.path.basename(xacro_path))
    doc = xacro.process_file(xacro_path, mappings=parameters)
    out = xacro.open_output(urdf_path)
    out.write(doc.toprettyxml(indent='  '))
    return urdf_path


def generate_launch_description():
    """生成感知系统的 Launch 描述。"""

    # 声明启动参数
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    # D435i URDF 路径（用于 robot_state_publisher 发布相机 TF）
    xacro_path = os.path.join(
        get_package_share_directory('realsense2_description'),
        'urdf', 'test_d435i_camera.urdf.xacro'
    )
    camera_urdf = to_urdf(xacro_path, {'use_nominal_extrinsics': 'true', 'add_plug': 'true'})

    # 1. RealSense 相机启动（启用原生点云）
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
            "pointcloud.enable": "true",
            "enable_color": "true",
            "enable_depth": "true",
            "depth_module.profile": "640x480x30",
            "rgb_camera.profile": "640x480x30",
            "camera_namespace": "",
        }.items(),
    )

    # 1.5 robot_state_publisher 发布 D435i 相机 TF（world → camera_link 等）
    robot_state_publisher_node = Node(
        name='camera_model_node',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='',
        output='screen',
        arguments=[camera_urdf],
    )

    # 2. 3D 检测器（订阅 realsense 原生点云）
    detection_node = Node(
        package="robotic_follower",
        executable="detection_node",
        name="detection_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                # todo: 减少数据量
                "pointcloud_topic": "/camera/camera/depth/color/points",
            }
        ],
    )

    # 3. RViz 可视化（使用 perception_rviz.rviz，包含 MarkerArray 显示）
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("robotic_follower"), "/launch/rviz_perception.launch.py"
        ]),
        launch_arguments={
            "rviz_config": "perception_rviz",
        }.items(),
    )

    return LaunchDescription(
        [
            # 声明参数
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Use simulation time"
            ),
            # 启动节点
            realsense_launch,
            robot_state_publisher_node,
            detection_node,
            rviz_launch,
        ]
    )
