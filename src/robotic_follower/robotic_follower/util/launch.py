import os
import tempfile

import xacro
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def to_urdf(xacro_path, parameters=None):
    """将 xacro 文件转换为 URDF 文件.

    Example:
    ```
    some_xacro = os.path.join(
        get_package_share_directory("xxx_description"),
        "some_path",
        "xxx.xacro",
    )
    # 将 xacro 处理为 URDF（robot_state_publisher 只接受 URDF）
    some_urdf = to_urdf(some_xacro,  {"some_param": "some_value",})
    # 读取 URDF 文件内容
    with open(some_urdf) as f:
        robot_desc = f.read()
    ```
    """
    urdf_path = tempfile.mktemp(
        prefix=f"{os.path.basename(xacro_path)}_", suffix=".urdf"
    )
    doc = xacro.process_file(xacro_path, mappings=parameters)
    out = xacro.open_output(urdf_path)
    out.write(doc.toprettyxml(indent="  "))  # pyright: ignore[reportAttributeAccessIssue]
    return urdf_path


def set_configurable_parameters(local_params):
    """设置 launch 参数.

    Example:
    ```
    local_parameters = [{
            "name": "use_sim_time",
            "default": "false",
            "description": "...",
        },]
    params = set_configurable_parameters(local_parameters)
    use_sim_time = params["use_sim_time"]
    ```
    """
    return {param["name"]: LaunchConfiguration(param["name"]) for param in local_params}


def declare_configurable_parameters(local_params):
    """声明 launch 参数.

    Example:
    ```
    local_parameters = [{
            "name": "use_sim_time",
            "default": "false",
            "description": "...",
    },]
    return LaunchDescription([
        *declare_configurable_parameters(local_parameters),
    ])
    ```
    """

    return [
        DeclareLaunchArgument(
            param["name"],
            default_value=param["default"],
            description=param["description"],
        )
        for param in local_params
    ]
