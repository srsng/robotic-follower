from typing import TypeVar, get_args, get_origin

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
        self._filter_warnings()

    def _check_type(self, value, expected) -> bool:
        """递归检查 value 是否符合 expected 类型。

        支持：
        - 基础类型：str, int, float, bool
        - list[inner]：验证所有元素类型一致
        - list[list[...]]：递归验证嵌套列表
        """
        if expected in (str, int, float, bool):
            return isinstance(value, expected)

        origin = get_origin(expected)
        if origin is list:
            if not isinstance(value, list):
                return False
            args = get_args(expected)
            if not args:
                return True  # list 无元素类型参数时不校验
            inner = args[0]
            return all(self._check_type(item, inner) for item in value)
        # 不支持的类型组合，默认放行
        self._warn(f"不能确定匹配的参数类型: {origin} {value}")
        return True

    def get_parameter_val_typed(self, name: str, expected_type: type[V]) -> V:
        """获取参数值，并断言其类型为 expected_type。

        expected_type 可以是基础类型（str/int/float/bool）或泛型类型（list[float]、
        list[list[float]] 等）。
        """
        param = self.get_parameter(name)
        if param.type_ == Parameter.Type.NOT_SET:
            raise RuntimeError(f"Parameter '{name}' not declared or not set.")
        if not param.type_.check(param.value):
            raise TypeError(
                f"Parameter '{name}' has type {type(param.value).__name__}, "
                f"expected {expected_type!r}"
            )
        if not self._check_type(param.value, expected_type):
            raise TypeError(
                f"Parameter '{name}' elements have type mismatch, "
                f"expected {expected_type!r}"
            )
        return param.value  # type: ignore

    def declare_and_get_parameter(
        self,
        name: str,
        default_value: V,
        expected_type: type[V] | None = None,
    ) -> V:
        """声明并获取参数。

        Args:
            name: 参数名
            default_value: 默认值，用于 declare 也用于类型推断
            expected_type: 期望的类型，用于类型断言, 默认使用 type(default_value),
                若是list[float]、list[list[float]] 等嵌套类型, 需要手动传入确定类型
        """
        self.declare_parameter(name, default_value)
        if expected_type is None:
            expected_type = type(default_value)
        return self.get_parameter_val_typed(name, expected_type)

    @staticmethod
    def _filter_warnings():
        """过滤底层库已知的无害警告"""
        import warnings

        messages = [
            "Unable to import Axes3D",
            "Unnecessary conv bias before batch/instance norm",
            "The torch.cuda.*DtypeTensor constructors are no longer recommended",
        ]
        for msg in messages:
            warnings.filterwarnings("ignore", message=msg, category=UserWarning)
