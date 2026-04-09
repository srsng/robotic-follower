from typing import TypeVar

from rclpy.node import Node, Parameter

from robotic_follower.util.handler import NodeHandler


V = TypeVar("V", str, int, float, bool, list)


class NodeWrapper(Node, NodeHandler):
    """ROS Node 封装类，同时具备 Node 能力和简化的日志接口。

    使用方式:
        class MyNode(NodeWrapper):
            def __init__(self):
                super().__init__("my_node")
                self._info("节点启动")  # 直接使用简化日志
                # 声明类型并返回值，附带类型
                self.var = self.declare_and_get_parameter("paramter_name", some_default)
    """

    def __init__(self, node_name: str, **kwargs):
        Node.__init__(self, node_name, **kwargs)
        NodeHandler.__init__(self, parent_node=self)

    def get_parameter_val_typed(self, name: str, expected_type: type[V]) -> V:
        """获取参数值，并断言其类型为 expected_type。"""
        param = self.get_parameter(name)
        if param.type_ == Parameter.Type.NOT_SET:
            raise RuntimeError(f"Parameter '{name}' not declared or not set.")
        if not param.type_.check(param.value):
            raise TypeError(
                f"Parameter '{name}' has type {type(param.value).__name__}, "
                f"expected {expected_type.__name__}"
            )
        return param.value  # type: ignore

    def declare_and_get_parameter(
        self,
        name: str,
        default_value: V,
    ) -> V:
        """声明并获取参数，使用 default_value 推断类型

        不支持动态类型，不支持嵌套类型
        """
        self.declare_parameter(name, default_value)
        return self.get_parameter_val_typed(name, type(default_value))
