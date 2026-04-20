#!/usr/bin/env python3


import rclpy
from rclpy.node import Node


class ANode(Node):
    def __init__(self):
        self.get_logger().fatal("ciallo")


def main(args=None):
    rclpy.init(args=args)
    node = ANode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到中断信号")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
