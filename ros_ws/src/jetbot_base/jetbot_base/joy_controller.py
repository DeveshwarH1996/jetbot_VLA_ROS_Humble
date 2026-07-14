from enum import Enum

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class Mode(str, Enum):
    VLA = 'vla'
    MANUAL = 'manual'
    TRADITIONAL = 'traditional'


MODE_CYCLE = [Mode.VLA, Mode.MANUAL, Mode.TRADITIONAL]


class JoyController(Node):
    """
    Single source of truth for "what should be driving the robot right
    now" - replaces twist_mux + mode_arbiter. Cycles through
    vla -> manual -> traditional -> vla on a configurable button press,
    and converts joystick axes to a Twist for manual mode.

    Publishes 'joy_controller/mode' and 'cmd_vel_joy' on every /joy
    message, not just on change. This matters: motor_driver's staleness
    checks (does the selected source's data still look alive?) depend on
    a steady stream of messages while the joystick is connected - the
    same role twist_mux's per-topic timeouts used to play. As long as
    the physical joystick is connected, joy_node's own autorepeat keeps
    /joy arriving continuously even with no input change, which is what
    keeps this continuous rather than needing a separate timer.

    Button/axis indices are hardware-specific and NOT verified against a
    physical controller in this environment - defaults below come from
    ros-humble-teleop_twist_joy's own reference xbox.config.yaml
    (axis_linear=1, axis_angular=0), which should be correct for a
    standard Xbox controller via the Linux joystick driver, but confirm
    with `ros2 topic echo /joy` and adjust config/joy_controller.yaml if
    your hardware maps differently.
    """

    def __init__(self):
        super().__init__('joy_controller')

        self.declare_parameter('mode_button', 7)     # Xbox: Start/Menu - deliberate, not a face button
        self.declare_parameter('linear_axis', 1)      # left stick vertical
        self.declare_parameter('angular_axis', 0)      # left stick horizontal
        self.declare_parameter('linear_scale', 0.2)    # matches motor_driver's max_linear_vel
        self.declare_parameter('angular_scale', 1.0)   # matches motor_driver's max_angular_vel
        self.declare_parameter('deadzone', 0.05)

        self.mode_button = self.get_parameter('mode_button').get_parameter_value().integer_value
        self.linear_axis = self.get_parameter('linear_axis').get_parameter_value().integer_value
        self.angular_axis = self.get_parameter('angular_axis').get_parameter_value().integer_value
        self.linear_scale = self.get_parameter('linear_scale').get_parameter_value().double_value
        self.angular_scale = self.get_parameter('angular_scale').get_parameter_value().double_value
        self.deadzone = self.get_parameter('deadzone').get_parameter_value().double_value

        self.mode_index = 0
        self.prev_button_state = 0

        self.mode_pub = self.create_publisher(String, 'joy_controller/mode', 10)
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel_joy', 10)
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.get_logger().info(
            f"Joy controller active. Starting mode: '{MODE_CYCLE[self.mode_index].value}'. "
            f"mode_button={self.mode_button} (verify against your hardware with 'ros2 topic echo /joy')."
        )

    def _apply_deadzone(self, value):
        return 0.0 if abs(value) < self.deadzone else value

    def joy_callback(self, msg):
        # Rising-edge detection: /joy publishes continuously (buttons[]
        # holds level, not a press event), so cycling on every message
        # where the button reads 1 would advance the mode many times per
        # second while held rather than once per physical press.
        if self.mode_button < len(msg.buttons):
            button_state = msg.buttons[self.mode_button]
            if button_state == 1 and self.prev_button_state == 0:
                self.mode_index = (self.mode_index + 1) % len(MODE_CYCLE)
                self.get_logger().info(f"Mode -> '{MODE_CYCLE[self.mode_index].value}'")
            self.prev_button_state = button_state

        twist = Twist()
        if self.linear_axis < len(msg.axes):
            twist.linear.x = self._apply_deadzone(msg.axes[self.linear_axis]) * self.linear_scale
        if self.angular_axis < len(msg.axes):
            twist.angular.z = self._apply_deadzone(msg.axes[self.angular_axis]) * self.angular_scale

        self.mode_pub.publish(String(data=MODE_CYCLE[self.mode_index].value))
        self.twist_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = JoyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
