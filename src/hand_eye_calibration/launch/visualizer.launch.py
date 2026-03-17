"""相机图像可视化启动文件。"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    """生成启动描述。"""

    # 声明启动参数
    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='启用可视化功能'
    )

    save_dir_arg = DeclareLaunchArgument(
        'save_dir',
        default_value='saved_images',
        description='图像保存目录'
    )

    rgb_topic_arg = DeclareLaunchArgument(
        'rgb_topic',
        default_value='/camera/color/image_raw',
        description='RGB图像话题'
    )

    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/camera/aligned_depth_to_color/image_raw',
        description='深度图像话题'
    )

    return LaunchDescription([
        enable_visualization_arg,
        save_dir_arg,
        rgb_topic_arg,
        depth_topic_arg,

        Node(
            package='hand_eye_calibration',
            executable='visualizer_node',
            name='visualizer_node',
            output='screen',
            parameters=[{
                'enable_visualization': LaunchConfiguration('enable_visualization'),
                'save_dir': LaunchConfiguration('save_dir'),
                'rgb_topic': LaunchConfiguration('rgb_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
            }]
        )
    ])
