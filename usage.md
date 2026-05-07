# 终端1：启动机械臂 MoveIt
ros2 launch dummy_moveit_config demo_real_arm.launch.py 
# 需要修改文件中的urdf路径为本项目的 dummy_with_d435i.urdf.xacro, 修改方式参考 src/robotic_follower/launch/demo_real_arm_launch.py

# 终端 2 - 启动机械臂控制器：
ros2 run dummy_controller dummy_arm_controller
# 或者 fake 发布关节状态
ros2 run robotic_follower fake_joint_states_publisher 

# 终端 3 - 运行规划器（可选）：
python3 src/ros2_dummy_arm_810/src/dummy_tools/moveit_rviz_planner.py


# 标定启动：
# 终端4：启动标定
ros2 launch robotic_follower eazy_handeye_calib.launch.py

# 运行时启动：
# 终端4：启动感知 
ros2 launch robotic_follower perception_real.launch.py
# 终端5：启动跟随控制
ros2 launch robotic_follower track_and_follow.launch.py 
