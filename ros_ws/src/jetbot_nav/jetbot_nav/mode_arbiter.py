import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult


class ModeArbiter(Node):
    """
    Selects exactly one of two peer autonomous sources - the traditional
    Nav2 planner/controller, or the VLA - and republishes only that one.

    This exists because twist_mux itself can't do this selection: its
    "lock" mechanism blocks everything at or below a priority level (an
    e-stop/veto primitive), not a specific single channel, so it can't
    cleanly express "exactly one of these two peers is live" without
    also affecting unrelated priority tiers. This node sits upstream of
    twist_mux instead; twist_mux's own priority order (safety > joystick
    > autonomous) is unchanged, so the joystick still always overrides
    whichever autonomous source is currently selected here.

    Switch modes live: ros2 param set /mode_arbiter mode vla
    """

    VALID_MODES = ('traditional', 'vla')

    def __init__(self):
        super().__init__('mode_arbiter')

        self.declare_parameter('mode', 'traditional')
        mode = self.get_parameter('mode').get_parameter_value().string_value
        if mode not in self.VALID_MODES:
            self.get_logger().error(f"Invalid mode '{mode}', defaulting to 'traditional'.")
            mode = 'traditional'
        self.mode = mode
        self.add_on_set_parameters_callback(self._on_param_change)

        self.pub = self.create_publisher(Twist, 'cmd_vel_autonomous', 10)
        self.create_subscription(Twist, 'cmd_vel_nav', self._make_callback('traditional'), 10)
        self.create_subscription(Twist, 'cmd_vel_final', self._make_callback('vla'), 10)

        self.get_logger().info(f"Mode arbiter active. Starting mode: '{self.mode}'.")

    def _on_param_change(self, params):
        for p in params:
            if p.name == 'mode':
                if p.value not in self.VALID_MODES:
                    return SetParametersResult(
                        successful=False,
                        reason=f"mode must be one of {self.VALID_MODES}")
                self.mode = p.value
                self.get_logger().info(f"Switched to '{self.mode}' mode.")
        return SetParametersResult(successful=True)

    def _make_callback(self, source_mode):
        def callback(msg):
            if self.mode == source_mode:
                self.pub.publish(msg)
        return callback


def main(args=None):
    rclpy.init(args=args)
    node = ModeArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
