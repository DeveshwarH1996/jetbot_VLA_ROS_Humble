import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class JetbotCameraNode(Node):
    """
    Publishes the raw camera stream from /dev/video0 to ROS2.
    Uses OpenCV for capture and CvBridge for ROS2 message conversion.
    """
    def __init__(self):
        super().__init__('jetbot_camera_node')
        
        # Parameters
        self.declare_parameter('video_device', '/dev/video0')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)

        self.device = self.get_parameter('video_device').get_parameter_value().string_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value

        # Initialize Camera
        self.cap = cv2.VideoCapture(self.device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open video device {self.device}")
            raise RuntimeError(f"Camera {self.device} failed to open")

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        
        # Timer for publishing
        self.timer = self.create_timer(1.0 / self.fps, self.timer_callback)
        
        self.get_logger().info(f"Camera node started: {self.device} ({self.width}x{self.height} @ {self.fps}fps)")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Convert OpenCV image to ROS Image message
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_link'
            
            self.publisher.publish(img_msg)
        else:
            self.get_logger().warn("Failed to capture frame from camera")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = JetbotCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
