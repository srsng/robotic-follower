"""相机启动文件"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """生成相机启动描述"""
    package_name = 'camera_acquisition'

    # 获取包路径
    pkg_share = get_package_share_directory(package_name)

    # 默认配置文件路径
    default_config_file = os.path.join(pkg_share, 'config', 'camera_config.yaml')

    # 声明启动参数
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='相机配置文件路径'
    )
    declare_publish_rate = DeclareLaunchArgument(
        'publish_rate',
        default_value='30.0',
        description='相机数据发布频率 (Hz)'
    )

    launch_config_file = LaunchConfiguration('config_file')
    launch_publish_rate = LaunchConfiguration('publish_rate')

    return LaunchDescription([
        declare_config_file,
        declare_publish_rate,
        # 相机节点
        Node(
            package=package_name,
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[
                {'config_file': launch_config_file},
                {'publish_rate': launch_publish_rate}
            ],
        )
    ])
