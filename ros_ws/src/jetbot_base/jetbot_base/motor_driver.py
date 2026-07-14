import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from .motor_interface import MockMotorInterface, WaveshareMotorInterface


class JetbotMotorDriver(Node):
    """
    ROS2 Node that consumes /cmd_vel_mux and drives the JetBot motors.

    Implements the 'Sleeve' (Safety Governor) pattern, and publishes
    open-loop /odom by integrating commanded velocity - the Waveshare
    JetBot's motors have no encoders, so this is dead-reckoning against
    what we *told* the motors to do, not what they actually did. It will
    drift steadily (wheel slip, PWM/motor nonlinearity, no feedback at
    all) and is published with large fixed covariance so any future
    sensor fusion (robot_localization, RTAB-Map) treats it as a weak
    prior, not ground truth.
    """

    def __init__(self):
        super().__init__('jetbot_motor_driver')

        # Parameters
        self.declare_parameter('wheel_base', 0.14)
        self.declare_parameter('max_linear_vel', 0.2)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('use_mock', True)
        self.declare_parameter('heartbeat_timeout', 0.5)
        self.declare_parameter('odom_rate_hz', 20.0)
        # Set false when robot_localization's EKF is fusing this /odom -
        # only one node should ever broadcast a given TF edge. See
        # jetbot_base's README for the odom->base_footprint TF ownership
        # rule this implements.
        self.declare_parameter('publish_tf', True)

        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.max_linear_vel = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.max_angular_vel = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        self.use_mock = self.get_parameter('use_mock').get_parameter_value().bool_value
        self.heartbeat_timeout = self.get_parameter('heartbeat_timeout').get_parameter_value().double_value
        odom_rate_hz = self.get_parameter('odom_rate_hz').get_parameter_value().double_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value

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
        self.commanded_linear_x = 0.0
        self.commanded_angular_z = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_odom_time = self.get_clock().now()

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

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Watchdog Timer
        self.timer = self.create_timer(0.1, self.watchdog_callback)
        self.odom_timer = self.create_timer(1.0 / odom_rate_hz, self.odom_callback)

        self.get_logger().info("Jetbot Motor Driver node initialized and listening to /cmd_vel_mux")

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = time.time()

        # 1. Safety Governor: Clamp velocities
        linear_x = max(min(msg.linear.x, self.max_linear_vel), -self.max_linear_vel)
        angular_z = max(min(msg.angular.z, self.max_angular_vel), -self.max_angular_vel)
        self.commanded_linear_x = linear_x
        self.commanded_angular_z = angular_z

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
                self.commanded_linear_x = 0.0
                self.commanded_angular_z = 0.0

    def odom_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = now

        # Unicycle-model dead reckoning from commanded (not measured) velocity.
        self.x += self.commanded_linear_x * math.cos(self.theta) * dt
        self.y += self.commanded_linear_x * math.sin(self.theta) * dt
        self.theta += self.commanded_angular_z * dt

        stamp = now.to_msg()
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        # Large, fixed covariance: this is open-loop and gets worse with
        # distance traveled, not a calibrated uncertainty estimate.
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.1
        odom.twist.twist.linear.x = self.commanded_linear_x
        odom.twist.twist.angular.z = self.commanded_angular_z
        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = 'odom'
            tf.child_frame_id = 'base_footprint'
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)


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
