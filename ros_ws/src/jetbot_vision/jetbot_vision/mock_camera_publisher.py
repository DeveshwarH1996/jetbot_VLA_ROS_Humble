import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class MockCameraPublisher(Node):
    """
    Publishes synthetic frames on /camera/image_raw so the rest of the
    pipeline (vla_client_bridge, yolo_detector) can be exercised without a
    physical camera attached, e.g. on a dev machine or in CI.
    """
    def __init__(self):
        super().__init__('mock_camera_publisher')

        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 10.0)

        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        fps = self.get_parameter('fps').get_parameter_value().double_value

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)

        self.get_logger().info(f"Mock camera publishing {self.width}x{self.height} @ {fps}fps")

    def timer_callback(self):
        frame = np.random.randint(0, 255, (self.height, self.width, 3), dtype=np.uint8)
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera_link'
        self.publisher.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
