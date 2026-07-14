import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from .motor_interface import MockMotorInterface, WaveshareMotorInterface


class JetbotMotorDriver(Node):
    """
    Drives the JetBot motors and arbitrates between three command sources
    based on joy_controller's mode - replaces twist_mux + mode_arbiter.

    joy_controller publishes 'joy_controller/mode' ('manual' / 'vla' /
    'traditional') and continuously relays the joystick as 'cmd_vel_joy'.
    This node follows whichever of {cmd_vel_joy, cmd_vel_final (VLA),
    cmd_vel_nav (traditional planner)} the current mode selects, and
    fails safe (stops) in two distinct ways:
      - if the SELECTED source's own data goes stale, same role
        twist_mux's per-topic timeouts used to play.
      - if joy_controller's mode signal itself goes stale, ALWAYS stop
        regardless of what mode was last selected. This is deliberate:
        losing the joystick means losing the human's override channel,
        so autonomous driving must not continue just because the VLA or
        planner data still looks fine.

    Also publishes open-loop /odom by integrating commanded velocity -
    the Waveshare JetBot's motors have no encoders, so this is
    dead-reckoning against what we *told* the motors to do, not what
    they actually did. It will drift steadily (wheel slip, PWM/motor
    nonlinearity, no feedback at all) and is published with large fixed
    covariance so any future sensor fusion (robot_localization,
    RTAB-Map) treats it as a weak prior, not ground truth.
    """

    def __init__(self):
        super().__init__('jetbot_motor_driver')

        # Parameters
        self.declare_parameter('wheel_base', 0.14)
        self.declare_parameter('max_linear_vel', 0.2)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('use_mock', True)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('odom_rate_hz', 20.0)
        self.declare_parameter('control_rate_hz', 20.0)
        # Set false when robot_localization's EKF is fusing this /odom -
        # only one node should ever broadcast a given TF edge. See
        # jetbot_base's README for the odom->base_footprint TF ownership
        # rule this implements.
        self.declare_parameter('publish_tf', True)

        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.max_linear_vel = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.max_angular_vel = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        self.use_mock = self.get_parameter('use_mock').get_parameter_value().bool_value
        self.command_timeout = self.get_parameter('command_timeout').get_parameter_value().double_value
        odom_rate_hz = self.get_parameter('odom_rate_hz').get_parameter_value().double_value
        control_rate_hz = self.get_parameter('control_rate_hz').get_parameter_value().double_value
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

        # Arbitration state: current mode plus the latest Twist + receive
        # time for each of the three candidate sources.
        self.mode = None
        self.last_mode_time = None
        self.sources = {
            'manual': {'twist': Twist(), 'time': None},
            'vla': {'twist': Twist(), 'time': None},
            'traditional': {'twist': Twist(), 'time': None},
        }
        self.commanded_linear_x = 0.0
        self.commanded_angular_z = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_odom_time = self.get_clock().now()

        self.create_subscription(String, 'joy_controller/mode', self._mode_callback, 10)
        self.create_subscription(Twist, 'cmd_vel_joy', self._make_source_callback('manual'), 10)
        self.create_subscription(Twist, 'cmd_vel_final', self._make_source_callback('vla'), 10)
        self.create_subscription(Twist, 'cmd_vel_nav', self._make_source_callback('traditional'), 10)

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.control_timer = self.create_timer(1.0 / control_rate_hz, self._control_loop)
        self.odom_timer = self.create_timer(1.0 / odom_rate_hz, self.odom_callback)

        self.get_logger().info(
            "Jetbot Motor Driver initialized: arbitrating cmd_vel_joy / "
            "cmd_vel_final / cmd_vel_nav via joy_controller/mode."
        )

    def _mode_callback(self, msg):
        self.mode = msg.data
        self.last_mode_time = time.time()

    def _make_source_callback(self, name):
        def callback(msg):
            self.sources[name]['twist'] = msg
            self.sources[name]['time'] = time.time()
        return callback

    def _control_loop(self):
        now = time.time()

        # Losing the joystick's mode signal means losing the human's
        # ability to instantly retake control - stop outright rather
        # than keep following whatever mode was last selected.
        if self.last_mode_time is None or (now - self.last_mode_time) > self.command_timeout:
            self._stop("No mode signal from joy_controller (joystick disconnected?).")
            return

        source = self.sources.get(self.mode)
        if source is None:
            self._stop(f"Unknown mode '{self.mode}' from joy_controller.")
            return

        if source['time'] is None or (now - source['time']) > self.command_timeout:
            self._stop(f"'{self.mode}' mode selected but its command source has gone stale.")
            return

        self._drive(source['twist'])

    def _stop(self, reason):
        if self.commanded_linear_x != 0.0 or self.commanded_angular_z != 0.0:
            self.get_logger().warn(f"Stopping: {reason}")
        self.commanded_linear_x = 0.0
        self.commanded_angular_z = 0.0
        self.hw.stop()

    def _drive(self, msg):
        # Safety clamp
        linear_x = max(min(msg.linear.x, self.max_linear_vel), -self.max_linear_vel)
        angular_z = max(min(msg.angular.z, self.max_angular_vel), -self.max_angular_vel)
        self.commanded_linear_x = linear_x
        self.commanded_angular_z = angular_z

        # Differential drive kinematics
        left_vel = linear_x + (angular_z * self.wheel_base / 2.0)
        right_vel = linear_x - (angular_z * self.wheel_base / 2.0)

        # Normalize to [-1.0, 1.0]
        norm_left = max(min(left_vel / self.max_linear_vel, 1.0), -1.0)
        norm_right = max(min(right_vel / self.max_linear_vel, 1.0), -1.0)

        self.hw.set_speeds(norm_left, norm_right)

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
