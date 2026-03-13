# -*- coding: utf-8 -*-
"""
感知节点启动文件
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    """生成launch描述"""
    # 获取包共享目录
    pkg_share = Path(get_package_share_directory("cognition"))

    return LaunchDescription(
        [
            Node(
                package="cognition",
                executable="perception_node",
                name="perception_node",
                output="screen",
                parameters=[
                    # 是否启用3D检测
                    {"enable_detection": False},
                    # 是否启用密度计算
                    {"enable_density_calc": True},
                    # 模型配置路径（相对于包共享目录）
                    {
                        "model_config_path": str(
                            pkg_share / "config" / "model_config.yaml"
                        )
                    },
                    # 检查点路径（相对）
                    {"checkpoint_path": ""},
                    # 深度缩放因子（RealSense D435默认值）
                    {"depth_scale": 0.001},
                    # 深度范围
                    {"min_depth": 0.0},
                    {"max_depth": 3.0},
                ],
            ),
        ]
    )


def main(args=None):
    """主函数"""
    ld = generate_launch_description()

    from launch import LaunchService

    ls = LaunchService()
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == "__main__":
    main()
