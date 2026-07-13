import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rcl_interfaces.msg import SetParametersResult


class MockLidarPublisher(Node):
    """
    Publishes a synthetic /scan for testing predictive_governor without a
    physical LiDAR. Every reading equals the 'front_distance' parameter,
    which can be changed live:

        ros2 param set /mock_lidar_publisher front_distance 0.2

    to simulate the robot approaching a wall (sim_test_suite.md Test Case 1).
    """
    def __init__(self):
        super().__init__('mock_lidar_publisher')

        self.declare_parameter('front_distance', 5.0)
        self.declare_parameter('num_points', 360)
        self.declare_parameter('rate_hz', 10.0)

        self.num_points = self.get_parameter('num_points').get_parameter_value().integer_value
        rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value

        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.add_on_set_parameters_callback(self._on_param_change)
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_scan)

        self.get_logger().info(
            f"Mock LiDAR publishing /scan at {self.get_parameter('front_distance').value}m clear."
        )

    def _on_param_change(self, params):
        for p in params:
            if p.name == 'front_distance':
                self.get_logger().info(f"front_distance -> {p.value}m")
        return SetParametersResult(successful=True)

    def publish_scan(self):
        distance = self.get_parameter('front_distance').get_parameter_value().double_value

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_link'
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = (2 * math.pi) / self.num_points
        msg.range_min = 0.05
        msg.range_max = 10.0
        msg.ranges = [distance] * self.num_points

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockLidarPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
