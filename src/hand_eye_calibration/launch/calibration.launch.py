"""手眼标定启动文件。"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """生成启动描述。"""
    return LaunchDescription([
        Node(
            package='hand_eye_calibration',
            executable='calibration_node',
            name='hand_eye_calibration_node',
            output='screen'
        )
    ])
