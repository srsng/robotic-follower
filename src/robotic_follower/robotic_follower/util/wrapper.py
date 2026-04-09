from rclpy.node import Node

from robotic_follower.util.handler import NodeHandler


class NodeWrapper(Node, NodeHandler):
    """ROS Node 封装类，同时具备 Node 能力和简化的日志接口。

    使用方式:
        class MyNode(NodeWrapper):
            def __init__(self):
                super().__init__("my_node")
                self._info("节点启动")  # 直接使用简化日志
    """

    def __init__(self, node_name: str, **kwargs):
        Node.__init__(self, node_name, **kwargs)
        NodeHandler.__init__(self, parent_node=self)
