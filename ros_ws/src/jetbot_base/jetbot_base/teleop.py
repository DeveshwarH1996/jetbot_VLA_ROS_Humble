import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys

class JetbotTeleop(Node):
    """
    Basic keyboard teleop node for testing.
    """
    def __init__(self):
        super().__init__('jetbot_teleop')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info("Jetbot Teleop Node started. (SIMULATED INPUT)")
        self.get_logger().info("Sending a short test move forward... (Check /cmd_vel)")
        
        # Start a timer to send a test signal
        self.timer = self.create_timer(2.0, self.send_test_cmd)

    def send_test_cmd(self):
        msg = Twist()
        msg.linear.x = 0.1 # 0.1 m/s
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info("Published: Linear X=0.1")

def main(args=None):
    rclpy.init(args=args)
    node = JetbotTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
