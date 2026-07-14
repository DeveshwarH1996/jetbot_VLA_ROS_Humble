import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

try:
    from camera_info_manager import CameraInfoManager
except ImportError:
    CameraInfoManager = None


class JetbotCameraNode(Node):
    """
    Publish the raw camera stream from /dev/video0 to ROS2.

    Uses OpenCV for capture and CvBridge for ROS2 message conversion.
    """

    def __init__(self):
        super().__init__('jetbot_camera_node')

        # Parameters
        self.declare_parameter('video_device', '/dev/video0')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        # camera_calibration-format YAML, e.g. file:///path/to/jetbot_camera.yaml.
        # Without this, no /camera/camera_info is published - most vision
        # pipelines (visual odometry/SLAM in particular) need real intrinsics
        # and will silently misbehave on made-up ones, so we don't fabricate
        # defaults here.
        self.declare_parameter('camera_info_url', '')

        self.device = self.get_parameter('video_device').get_parameter_value().string_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        camera_info_url = self.get_parameter('camera_info_url').get_parameter_value().string_value

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
        self.info_publisher = self.create_publisher(CameraInfo, 'camera/camera_info', 10)

        self.cim = None
        if CameraInfoManager is None:
            self.get_logger().error(
                "camera_info_manager not installed (sudo apt-get install "
                "ros-humble-camera-info-manager-py). /camera/camera_info will "
                "NOT be published - visual odometry/SLAM needs it."
            )
        else:
            self.cim = CameraInfoManager(self, cname='jetbot_camera', url=camera_info_url,
                                          namespace='camera')
            self.cim.loadCameraInfo()
            if not self.cim.isCalibrated():
                self.get_logger().warn(
                    "No camera calibration loaded (camera_info_url not set or "
                    "invalid) - publishing UNCALIBRATED CameraInfo. Run "
                    "ros2 run camera_calibration cameracalibrator to calibrate; "
                    "geometric vision (visual odometry/SLAM) accuracy will "
                    "suffer without it."
                )

        # Timer for publishing
        self.timer = self.create_timer(1.0 / self.fps, self.timer_callback)

        self.get_logger().info(f"Camera node started: {self.device} ({self.width}x{self.height} @ {self.fps}fps)")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            stamp = self.get_clock().now().to_msg()

            # Convert OpenCV image to ROS Image message
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp = stamp
            img_msg.header.frame_id = 'camera_link'
            self.publisher.publish(img_msg)

            if self.cim is not None:
                info_msg = self.cim.getCameraInfo()
                info_msg.header.stamp = stamp
                info_msg.header.frame_id = 'camera_link'
                self.info_publisher.publish(info_msg)
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
