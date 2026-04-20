cmake_minimum_required(VERSION 3.8)
project(robotic_follower_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrackedObject3D.msg"
  "msg/TrackedObject3DArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
