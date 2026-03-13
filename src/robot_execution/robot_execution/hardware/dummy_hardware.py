"""
Dummy机械臂的ros2_control硬件接口
实现 HardwareInterface 接口，提供与真实硬件的通信

注意：此模块需要 dummy_controller 包提供硬件驱动
需要在 setup.py 中添加依赖：dummy_controller
"""
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from std_msgs.msg import Float64

try:
    import dummy_controller.dummy_cli_tool.ref_tool as dummy_driver
    DUMMY_DRIVER_AVAILABLE = True
except ImportError:
    DUMMY_DRIVER_AVAILABLE = False


class DummyHardwareInterface:
    """Dummy机械臂硬件接口（ros2_control）"""

    # 关节名称（6个自由度）
    JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

    # 位置校正（度）- 对齐真实机械臂零位
    STATE_CORRECTION_DEG = np.array([0.0, -72.0, 90.0, 0.0, 0.0, 0.0])

    # 指令方向调整
    RAD_DIRECT_DIFF = np.array([1, 1, 1, 1, -1, -1])

    # 体积偏移（弧度）
    RAD_VOLUME_DIFF = np.array([0.0, -0.05, 1.57079, 0.0, 0.0, 0.0])

    # 电流限制（A）
    CURRENT_LIMITS = {
        'joint_1': 1.5,
        'joint_2': 3.0,
        'joint_3': 3.0,
        'joint_4': 2.0,
        'joint_5': 1.5,
        'joint_6': 1.2,
        'hand': 0.5
    }

    def __init__(self):
        if not DUMMY_DRIVER_AVAILABLE:
            raise ImportError(
                "dummy_controller 包未安装！请确保 dummy_controller 包可用"
            )

        # 状态和命令缓冲区
        self.position_commands = [0.0] * 6
        self.position_states = [0.0] * 6
        self.velocity_states = [0.0] * 6
        self.effort_states = [0.0] * 6

        # 硬件驱动
        self.driver = None
        self.is_connected = False
        self.is_enabled = False

        # ROS2节点（用于发布关节状态和服务）
        self.node = None
        self.joint_state_pub = None
        self.joint_state_timer = None

    # ========== ros2_control Lifecycle Callbacks ==========

    def on_init(self, hardware_info):
        """初始化回调"""
        # 创建ROS2节点（用于发布关节状态和服务）
        import rclpy
        if not rclpy.ok():
            rclpy.init()

        self.node = Node('dummy_hardware_interface')
        self.node.get_logger().info('初始化Dummy硬件接口')

        return 0  # Return 0 for success

    def on_configure(self, previous_state):
        """配置回调：连接硬件"""
        try:
            self.driver = dummy_driver.find_any()
            if self.driver is None:
                self.node.get_logger().error('未找到Dummy机械臂')
                return 1  # Return error code

            self.is_connected = True
            self.node.get_logger().info('Dummy机械臂连接成功')
            return 0  # Success

        except Exception as e:
            self.node.get_logger().error(f'硬件连接失败: {e}')
            return 1  # Error

    def on_activate(self, previous_state):
        """激活回调：使能机械臂"""
        if not self.is_connected:
            return 1  # Error

        try:
            # 创建关节状态发布器
            self.joint_state_pub = self.node.create_publisher(
                JointState, 'joint_states', 10
            )
            self.joint_state_timer = self.node.create_timer(
                0.1,  # 10Hz
                self._publish_joint_states
            )

            # 创建服务
            self._create_services()

            # 使能机械臂
            self.driver.robot.set_enable(1)
            self.driver.robot.set_command_mode(0)
            self._setup_current_limits()
            self._setup_gripper()

            self.is_enabled = True
            self.node.get_logger().info('机械臂已使能')
            return 0  # Success

        except Exception as e:
            self.node.get_logger().error(f'使能失败: {e}')
            return 1  # Error

    def on_deactivate(self, previous_state):
        """去激活回调"""
        if self.is_enabled:
            try:
                self.driver.robot.set_enable(0)
            except Exception:
                pass
            self.is_enabled = False

        # 清理定时器
        if self.joint_state_timer is not None:
            self.joint_state_timer.destroy()
            self.joint_state_timer = None

        return 0  # Success

    def on_cleanup(self, previous_state):
        """清理回调"""
        self.is_connected = False
        self.driver = None
        return 0  # Success

    def on_shutdown(self, previous_state):
        """关闭回调"""
        self.on_cleanup(previous_state)
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        return 0  # Success

    def on_error(self, previous_state):
        """错误回调"""
        return 0  #.Return 0 (allowed return value)

    # ========== ros2_control Hardware Interface ==========

    def export_state_interfaces(self):
        """导出状态接口供控制器读取"""
        interfaces = []
        for i, joint in enumerate(self.JOINT_NAMES):
            interfaces.append([f'{joint}/position', self.position_states, i])
            interfaces.append([f'{joint}/velocity', self.velocity_states, i])
        return interfaces

    def export_command_interfaces(self):
        """导出命令接口供控制器写入"""
        interfaces = []
        for i, joint in enumerate(self.JOINT_NAMES):
            interfaces.append([f'{joint}/position', self.position_commands, i])
        return interfaces

    def read(self, time, period):
        """读取硬件状态（周期性调用）"""
        if not self.is_connected or not self.is_enabled:
            return

        try:
            # 从硬件读取关节角度（度）
            angles_deg = np.array([
                self.driver.robot.joint_1.angle,
                self.driver.robot.joint_2.angle,
                self.driver.robot.joint_3.angle,
                self.driver.robot.joint_4.angle,
                self.driver.robot.joint_5.angle,
                self.driver.robot.joint_6.angle
            ])

            # 应用校正并转换为弧度
            corrected_deg = angles_deg + self.STATE_CORRECTION_DEG
            corrected_deg[5] = -corrected_deg[5]  # J6方向调整
            corrected_deg[4] = -corrected_deg[4]  # J5方向调整

            angles_rad = np.radians(corrected_deg)

            # 更新状态缓冲区
            for i in range(6):
                self.position_states[i] = angles_rad[i]
                self.velocity_states[i] = 0.0  # 可选：读取速度

        except Exception as e:
            if self.node:
                self.node.get_logger().error(f'读取状态失败: {e}')

    def write(self, time, period):
        """写入硬件命令（周期性调用）"""
        if not self.is_connected or not self.is_enabled:
            return

        try:
            # 获取位置命令
            target_rad = np.array(self.position_commands)

            # 应用指令变换
            corrected_rad = (target_rad + self.RAD_VOLUME_DIFF) * self.RAD_DIRECT_DIFF
            target_deg = np.degrees(corrected_rad)

            # 发送到硬件
            self.driver.robot.move_j(
                target_deg[0], target_deg[1], target_deg[2],
                target_deg[3], target_deg[4], target_deg[5]
            )

        except Exception as e:
            if self.node:
                self.node.get_logger().error(f'写入命令失败: {e}')

    # ========== Private Methods ==========

    def _publish_joint_states(self):
        """发布关节状态到ROS话题"""
        if self.joint_state_pub is None:
            return

        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = self.JOINT_NAMES
        msg.position = self.position_states
        msg.velocity = self.velocity_states
        msg.effort = [0.0] * 6
        self.joint_state_pub.publish(msg)

    def _create_services(self):
        """创建ROS服务"""
        if self.node is None:
            return

        # 使能服务
        self.enable_srv = self.node.create_service(
            SetBool, '/dummy_arm/enable', self._enable_callback
        )

        # 夹爪服务
        self.gripper_enable_srv = self.node.create_service(
            SetBool, '/dummy_arm/gripper_enable', self._gripper_enable_callback
        )
        self.gripper_open_srv = self.node.create_service(
            SetBool, '/dummy_arm/gripper_open', self._gripper_open_callback
        )
        self.gripper_close_srv = self.node.create_service(
            SetBool, '/dummy_arm/gripper_close', self._gripper_close_callback
        )

        # 夹爪角度订阅
        self.gripper_angle_sub = self.node.create_subscription(
            Float64, '/dummy_arm/gripper_angle',
            self._gripper_angle_callback, 10
        )

    def _setup_current_limits(self):
        """设置电流限制"""
        try:
            for joint_name, limit in self.CURRENT_LIMITS.items():
                if joint_name == 'hand':
                    if hasattr(self.driver.robot.hand, 'set_current_limit'):
                        self.driver.robot.hand.set_current_limit(limit)
                else:
                    joint = getattr(self.driver.robot, joint_name)
                    if hasattr(joint, 'set_current_limit'):
                        joint.set_current_limit(limit)
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f'设置电流限制失败: {e}')

    def _setup_gripper(self):
        """初始化夹爪"""
        try:
            if hasattr(self.driver.robot, 'hand'):
                self.driver.robot.hand.set_enable(True)
                self.driver.robot.hand.set_angle_with_speed_limit(-100.0)
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f'夹爪初始化失败: {e}')

    # ========== Service Callbacks ==========

    def _enable_callback(self, request, response):
        """使能/去使能服务回调"""
        try:
            if request.data:
                self.driver.robot.set_enable(1)
                self.driver.robot.set_command_mode(0)
                self._setup_current_limits()
                self.is_enabled = True
                response.success = True
                response.message = '机械臂已使能'
            else:
                self.driver.robot.set_enable(0)
                self.is_enabled = False
                response.success = True
                response.message = '机械臂已去使能'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _gripper_enable_callback(self, request, response):
        """夹爪使能回调"""
        try:
            if hasattr(self.driver.robot, 'hand'):
                self.driver.robot.hand.set_enable(request.data)
                response.success = True
                response.message = '夹爪已使能' if request.data else '夹爪已去使能'
            else:
                response.success = False
                response.message = '未检测到夹爪'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _gripper_open_callback(self, request, response):
        """打开夹爪"""
        try:
            if hasattr(self.driver.robot, 'hand'):
                self.driver.robot.hand.close()  # CLI中close是打开
                response.success = True
                response.message = '夹爪已打开'
            else:
                response.success = False
                response.message = '未检测到夹爪'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _gripper_close_callback(self, request, response):
        """闭合夹爪"""
        try:
            if hasattr(self.driver.robot, 'hand'):
                self.driver.robot.hand.open()  # CLI中open是闭合
                response.success = True
                response.message = '夹爪已闭合'
            else:
                response.success = False
                response.message = '未检测到夹爪'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _gripper_angle_callback(self, msg):
        """夹爪角度控制"""
        try:
            angle = max(-100, min(100, msg.data))
            if hasattr(self.driver.robot, 'hand'):
                self.driver.robot.hand.set_angle_with_speed_limit(angle)
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f'夹爪角度控制失败: {e}')
