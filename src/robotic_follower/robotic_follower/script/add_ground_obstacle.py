#!/usr/bin/env python3
"""添加地面障碍物到 MoveIt planning scene。

地面以世界原点为中心，边长5m的矩形，厚度0.01m。
"""

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


def create_box_obstacle(
    name: str,
    x: float,
    y: float,
    z: float,
    size_x: float,
    size_y: float,
    size_z: float,
    frame_id: str = "world",
) -> CollisionObject:
    """创建长方体障碍物。

    Args:
        name: 障碍物名称
        x, y, z: 障碍物中心位置
        size_x, size_y, size_z: 障碍物尺寸
        frame_id: 所属坐标系

    Returns:
        CollisionObject 消息
    """
    collision_object = CollisionObject()
    collision_object.header.frame_id = frame_id
    collision_object.id = name

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [size_x, size_y, size_z]

    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0

    collision_object.primitives.append(box)
    collision_object.primitive_poses.append(pose)
    collision_object.operation = CollisionObject.ADD

    return collision_object


def main() -> None:
    rclpy.init()
    node = Node("ground_obstacle_adder")
    scene_publisher = node.create_publisher(PlanningScene, "planning_scene", 10)

    import time

    time.sleep(1.0)

    planning_scene = PlanningScene()
    planning_scene.is_diff = True

    # 地面障碍物：中心在原点，边长5m，厚度0.01m，位于z=0平面
    ground = create_box_obstacle(
        name="ground",
        x=0.0,
        y=0.0,
        z=-0.005,
        size_x=5.0,
        size_y=5.0,
        size_z=0.01,
        frame_id="world",
    )

    planning_scene.world.collision_objects = [ground]
    scene_publisher.publish(planning_scene)

    print("✅ 地面障碍物已添加: 5m x 5m x 0.01m at (0, 0, -0.005)")

    time.sleep(2.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
