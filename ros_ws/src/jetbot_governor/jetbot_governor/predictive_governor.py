import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def min_front_arc_distance(scan: LaserScan, half_angle_rad: float) -> float:
    """
    Return the minimum obstacle range within +/- half_angle_rad of the
    scan's zero-angle (front of the robot), or inf if the arc is clear.
    Per REP-117: nan means "invalid measurement" (dropped), +inf means
    "no return within range_max" (treated as clear out to range_max).
    """
    best = math.inf
    for i, d in enumerate(scan.ranges):
        if math.isnan(d) or d < scan.range_min:
            continue  # invalid reading / sensor noise
        if math.isinf(d):
            d = scan.range_max  # nothing detected within sensor range
        angle = scan.angle_min + i * scan.angle_increment
        if -half_angle_rad <= angle <= half_angle_rad:
            best = min(best, d)
    return best


class PredictiveGovernor(Node):
    """
    Safety middleware (the WAM layer).
    Validates VLA proposals against real-time LiDAR data before they reach
    motor_driver (as the 'vla' mode input, selected by joy_controller - no
    mux/arbiter node). Fails safe (stop) whenever it cannot prove the move
    is safe.
    """
    def __init__(self):
        super().__init__('predictive_governor')

        self.declare_parameter('safety_threshold', 0.4)  # meters
        self.declare_parameter('front_arc_deg', 60.0)     # +/- degrees checked ahead
        self.declare_parameter('scan_timeout', 1.0)        # seconds

        self.threshold = self.get_parameter('safety_threshold').get_parameter_value().double_value
        front_arc_deg = self.get_parameter('front_arc_deg').get_parameter_value().double_value
        self.half_angle_rad = math.radians(front_arc_deg)
        self.scan_timeout = self.get_parameter('scan_timeout').get_parameter_value().double_value

        self.current_scan = None
        self.last_scan_time = None

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.vla_sub = self.create_subscription(Twist, '/cmd_vel_vla', self.vla_callback, 10)

        # Publishes the safety-checked command; motor_driver follows this
        # directly whenever joy_controller's mode is 'vla' - no mux node.
        self.pub = self.create_publisher(Twist, '/cmd_vel_final', 10)

        self.get_logger().info(
            f"Predictive Governor active. threshold={self.threshold}m "
            f"front_arc=+/-{front_arc_deg}deg"
        )

    def scan_callback(self, msg):
        self.current_scan = msg
        self.last_scan_time = self.get_clock().now()

    def _scan_is_stale(self):
        if self.current_scan is None or self.last_scan_time is None:
            return True
        age = (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9
        return age > self.scan_timeout

    def vla_callback(self, msg):
        # Fail safe: no LiDAR data yet, or it has gone stale (sensor/link
        # dropped) -> never allow forward motion.
        if self._scan_is_stale():
            self.get_logger().warn("No recent LiDAR data. Vetoing to STOP.")
            self.pub.publish(Twist())
            return

        dist_min = min_front_arc_distance(self.current_scan, self.half_angle_rad)

        if msg.linear.x > 0 and dist_min < self.threshold:
            self.get_logger().info(
                f"SAFETY VETO: obstacle at {dist_min:.2f}m in front arc. Overriding to STOP."
            )
            final_twist = Twist()
        else:
            final_twist = msg

        self.pub.publish(final_twist)


def main(args=None):
    rclpy.init(args=args)
    node = PredictiveGovernor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
