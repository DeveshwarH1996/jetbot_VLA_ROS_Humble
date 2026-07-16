import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import requests
import cv2
from cv_bridge import CvBridge


class VLAClientBridge(Node):
    """
    Bridge node that captures camera frames and sends them to a remote VLA
    server. Maps discrete VLA action tokens to continuous ROS2 Twist
    messages. Output is a raw, unvalidated proposal - predictive_governor
    is responsible for safety-checking it before it reaches motor_driver
    (as the 'vla' mode input, selected by joy_controller - no mux node).
    """
    def __init__(self):
        super().__init__('vla_client_bridge')

        self.declare_parameter('server_url', 'http://vla-server:8000/predict')
        self.declare_parameter('inference_rate_hz', 1.0)
        self.declare_parameter('task', 'Navigate to target and stop')
        self.declare_parameter('request_timeout', 2.0)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('command_max_age', 3.0)

        self.server_url = self.get_parameter('server_url').get_parameter_value().string_value
        rate_hz = self.get_parameter('inference_rate_hz').get_parameter_value().double_value
        self.inference_interval = 1.0 / rate_hz
        self.task = self.get_parameter('task').get_parameter_value().string_value
        self.request_timeout = self.get_parameter(
            'request_timeout').get_parameter_value().double_value
        publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.publish_interval = 1.0 / publish_rate_hz
        self.command_max_age = self.get_parameter(
            'command_max_age').get_parameter_value().double_value

        self.bridge = CvBridge()
        self.last_frame = None
        self.last_twist = Twist()
        self.last_action_time = None

        self.pub = self.create_publisher(Twist, '/cmd_vel_vla', 10)
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # VLA inference is slow (server round-trip), so it runs on its own
        # timer. Downstream consumers (predictive_governor -> motor_driver,
        # which stops if its selected source or joy_controller's mode goes
        # stale) all expect a steady stream of commands though, so a
        # separate faster timer republishes the last decision - decoupling
        # "how often the VLA re-plans" from "how often we publish a
        # command". If no fresh decision has arrived within command_max_age,
        # we publish zero instead of a stale command.
        self.inference_timer = self.create_timer(self.inference_interval, self.inference_loop)
        self.publish_timer = self.create_timer(self.publish_interval, self.publish_loop)

        self.get_logger().info(f"VLA Client Bridge initialized. Server: {self.server_url}")

    def image_callback(self, msg):
        self.last_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def inference_loop(self):
        if self.last_frame is None:
            return

        try:
            _, img_encoded = cv2.imencode('.jpg', self.last_frame)
            img_bytes = img_encoded.tobytes()

            response = requests.post(
                self.server_url,
                files={'image': ('frame.jpg', img_bytes, 'image/jpeg')},
                data={'task': self.task},
                timeout=self.request_timeout,
            )

            if response.status_code == 200:
                data = response.json()
                action_tokens = data.get('actions', [])
                self.map_tokens_to_twist(action_tokens)
            else:
                self.get_logger().warn(f"Server error: {response.status_code}")

        except requests.exceptions.RequestException as e:
            # Server down or slow: leave last_twist as-is. publish_loop's
            # command_max_age check is what stops the robot if this
            # persists, not a missing message on /cmd_vel_vla.
            self.get_logger().error(f"Inference loop failure: {e}")

    def map_tokens_to_twist(self, tokens):
        """
        Map the VLA's discrete action tokens to a velocity command.
        Example mapping: 'move_fwd' -> linear.x = 0.2, 'turn_l' -> angular.z = 0.5
        """
        if not tokens:
            return

        twist = Twist()
        token = tokens[0]
        if token == 'move_fwd':
            twist.linear.x = 0.2
        elif token == 'move_back':
            twist.linear.x = -0.1
        elif token == 'turn_l':
            twist.angular.z = 0.5
        elif token == 'turn_r':
            twist.angular.z = -0.5
        elif token == 'stop':
            pass  # zero twist

        self.last_twist = twist
        self.last_action_time = self.get_clock().now()

    def publish_loop(self):
        if self.last_action_time is None:
            return  # no VLA decision yet

        age = (self.get_clock().now() - self.last_action_time).nanoseconds / 1e9
        self.pub.publish(self.last_twist if age <= self.command_max_age else Twist())


def main(args=None):
    rclpy.init(args=args)
    node = VLAClientBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
