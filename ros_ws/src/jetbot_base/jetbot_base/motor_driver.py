import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .motor_interface import MockMotorInterface, WaveshareMotorInterface
import time

class JetbotMotorDriver(Node):
    """
    ROS2 Node that consumes /cmd_vel and drives the JetBot motors.
    Implements the 'Sleeve' (Safety Governor) pattern.
    """
    def __init__(self):
        super().__init__('jetbot_motor_driver')

        # Parameters
        self.declare_parameter('wheel_base', 0.14) 
        self.declare_parameter('max_linear_vel', 0.2) 
        self.declare_parameter('max_angular_vel', 1.0) 
        self.declare_parameter('use_mock', True)
        self.declare_parameter('heartbeat_timeout', 0.5) 

        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.max_linear_vel = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.max_angular_vel = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        self.use_mock = self.get_parameter('use_mock').get_parameter_value().bool_value
        self.heartbeat_timeout = self.get_parameter('heartbeat_timeout').get_parameter_value().double_value

        # Hardware Interface Initialization
        if self.use_mock:
            self.get_logger().info("Using MOCK motor interface.")
            self.hw = MockMotorInterface()
        else:
            try:
                self.get_logger().info("Initializing Waveshare hardware interface...")
                self.hw = WaveshareMotorInterface()
            except (ImportError, Exception) as e:
                self.get_logger().error(f"Hardware Init Failed: {e}. Falling back to MOCK.")
                self.hw = MockMotorInterface()

        # State
        self.last_cmd_time = time.time()

        # ROS Interface
        # Subscribes to twist_mux's arbitrated output (see
        # jetbot_base/config/twist_mux.yaml), NOT a raw command source -
        # twist_mux is what decides safety/joystick/vla priority.
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel_mux',
            self.cmd_vel_callback,
            10
        )

        # Watchdog Timer
        self.timer = self.create_timer(0.1, self.watchdog_callback)

        self.get_logger().info("Jetbot Motor Driver node initialized and listening to /cmd_vel_mux")

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = time.time()
        
        # 1. Safety Governor: Clamp velocities
        linear_x = max(min(msg.linear.x, self.max_linear_vel), -self.max_linear_vel)
        angular_z = max(min(msg.angular.z, self.max_angular_vel), -self.max_angular_vel)
        
        # 2. Differential Drive Kinematics
        left_vel = linear_x + (angular_z * self.wheel_base / 2.0)
        right_vel = linear_x - (angular_z * self.wheel_base / 2.0)
        
        # 3. Normalize to [-1.0, 1.0]
        norm_left = max(min(left_vel / self.max_linear_vel, 1.0), -1.0)
        norm_right = max(min(right_vel / self.max_linear_vel, 1.0), -1.0)
        
        self.hw.set_speeds(norm_left, norm_right)

    def watchdog_callback(self):
        if (time.time() - self.last_cmd_time) > self.heartbeat_timeout:
            if self.last_cmd_time != 0: 
                self.get_logger().warn("Heartbeat lost! Stopping motors.")
                self.hw.stop()
                self.last_cmd_time = 0 

def main(args=None):
    rclpy.init(args=args)
    node = JetbotMotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.hw.stop()
        node.destroy_node()
        rclpy.shutdown()
