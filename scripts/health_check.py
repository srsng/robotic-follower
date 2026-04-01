#!/usr/bin/env python3
"""系统健康检查脚本。"""

import rclpy
from rclpy.node import Node
import sys


class HealthChecker(Node):
    """系统健康检查器。"""

    def __init__(self):
        super().__init__('health_checker')
        self.results = []

    def check_topics(self):
        """检查必需话题。"""
        required_topics = [
            '/camera/color/image_raw',
            '/camera/depth/image_rect_raw',
            '/camera/color/camera_info',
            '/perception/detections',
            '/camera/camera/depth/color/points',
            '/joint_states',
        ]

        topic_list = self.get_topic_names_and_types()
        available_topics = [t[0] for t in topic_list]

        for topic in required_topics:
            if topic in available_topics:
                self.results.append(f"✓ 话题存在: {topic}")
            else:
                self.results.append(f"✗ 话题缺失: {topic}")

    def check_services(self):
        """检查必需服务。"""
        required_services = [
            '/hand_eye_calib/add_sample',
            '/hand_eye_calib/execute',
            '/hand_eye_calib/reset',
        ]

        service_list = self.get_service_names_and_types()
        available_services = [s[0] for s in service_list]

        for service in required_services:
            if service in available_services:
                self.results.append(f"✓ 服务可用: {service}")
            else:
                self.results.append(f"✗ 服务缺失: {service}")

    def print_results(self):
        """打印检查结果。"""
        print("\n" + "="*50)
        print("系统健康检查结果")
        print("="*50 + "\n")

        for result in self.results:
            print(result)

        failed = sum(1 for r in self.results if r.startswith('✗'))
        passed = len(self.results) - failed

        print(f"\n通过: {passed}/{len(self.results)}")
        print(f"失败: {failed}/{len(self.results)}\n")

        return failed == 0


def main():
    rclpy.init()
    checker = HealthChecker()

    try:
        checker.check_topics()
        checker.check_services()
        success = checker.print_results()
        sys.exit(0 if success else 1)
    finally:
        checker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
