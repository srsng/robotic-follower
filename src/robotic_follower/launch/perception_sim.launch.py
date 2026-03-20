#!/usr/bin/env python3
"""仿真感知启动文件。

使用模拟相机数据进行感知测试：
1. 模拟相机节点（从 .bin 文件发布数据）
2. 点云处理器（可选，跳过，因为 camera_sim 已处理）
3. 3D 检测器
4. 可视化 (可选 Open3D 或 RViz)

使用场景：
    - 离线数据测试
    - 无相机环境下的算法开发
    - SUNRGBD 数据集验证

启动命令：
    # 使用 SUNRGBD 索引 + Open3D 可视化（默认）
    ros2 launch robotic_follower perception_sim.launch.py sunrgbd_idx:=0

    # 使用 .bin 文件 + RViz 可视化
    ros2 launch robotic_follower perception_sim.launch.py bin_file:=/path/to/data.bin visualizer_type:=rviz

    # 仅点云处理，不运行检测
    ros2 launch robotic_follower perception_sim.launch.py run_detection:=false

    # Open3D 可视化（带窗口）
    ros2 launch robotic_follower perception_sim.launch.py visualizer_type:=open3d

    # RViz 可视化
    ros2 launch robotic_follower perception_sim.launch.py visualizer_type:=rviz
"""

from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription


def get_visualizer_node(context, visualizer_type):
    """根据 visualizer_type 返回对应的可视化节点列表。"""
    viz_type = context.perform_substitution(visualizer_type)
    assert viz_type in ("open3d", "rivz")
    
    if viz_type == "open3d":
        return [
            Node(
                package="robotic_follower",
                executable="open3d_visualizer",
                name="open3d_visualizer",
                output="screen",
            )
        ]
    # rviz
    return [
        Node(
            package="robotic_follower",
            executable="rviz_visualizer",
            name="rviz_visualizer",
            output="screen",
        )
    ]


def generate_launch_description():
    """生成感知仿真系统的 Launch 描述。"""

    # 声明启动参数
    bin_file = LaunchConfiguration("bin_file", default="")
    sunrgbd_idx = LaunchConfiguration("sunrgbd_idx", default=-1)
    run_detection = LaunchConfiguration("run_detection", default="true")
    publish_rate = LaunchConfiguration("publish_rate", default="1.0")
    visualizer_type = LaunchConfiguration("gui", default="open3d")

    # 1. 模拟相机节点
    camera_sim_node = Node(
        package="robotic_follower",
        executable="camera_sim_node",
        name="camera_sim_node",
        output="screen",
        parameters=[
            {
                "bin_file": bin_file,
                "sunrgbd_idx": sunrgbd_idx,
                "run_detection": run_detection,
                "publish_rate": publish_rate,
            }
        ],
    )

    # # 2. 点云处理器（可选，因为 camera_sim 已经处理）
    # pointcloud_processor_node = Node(
    #     package='robotic_follower',
    #     executable='pointcloud_processor',
    #     name='pointcloud_processor',
    #     output='screen',
    # )

    # 3. 3D 检测器
    detection_node = Node(
        package="robotic_follower",
        executable="detection_node",
        name="detection_node",
        output="screen",
    )

    # 4. 可视化节点（根据 visualizer_type 参数选择）
    visualizer_node = OpaqueFunction(
        function=get_visualizer_node, args=[visualizer_type]
    )

    return LaunchDescription(
        [
            # 声明参数
            DeclareLaunchArgument(
                "bin_file",
                default_value="",
                description="Path to the .bin point cloud file",
            ),
            DeclareLaunchArgument(
                "sunrgbd_idx",
                default_value="-1",
                description="SUNRGBD dataset sample index number (1 ~ 10335)",
            ),
            DeclareLaunchArgument(
                "run_detection",
                default_value="true",
                description="Run 3D detection on the point cloud",
            ),
            DeclareLaunchArgument(
                "publish_rate",
                default_value="1.0",
                description="Topic publishing rate (Hz)",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="open3d",
                description='GUI type of Visualizer: "open3d" (default) or "rviz"',
            ),
            # 启动节点
            camera_sim_node,
            # pointcloud_processor_node,
            detection_node,
            visualizer_node,
        ]
    )
