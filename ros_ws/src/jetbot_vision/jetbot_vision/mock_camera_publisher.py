import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class MockCameraPublisher(Node):
    """
    Publish synthetic frames on /camera/image_raw so the rest of the
    pipeline (vla_client_bridge, yolo_detector, visual odometry/SLAM) can be
    exercised without a physical camera attached, e.g. on a dev machine.

    Frames are a static checkerboard, not random noise: feature-based
    pipelines (ORB/GFTT, visual odometry) need trackable structure to
    exercise their code paths at all, even though a static image can only
    prove the pipeline runs cleanly, not that it tracks real motion - there
    is no real camera moving through space in mock mode.
    """

    def __init__(self):
        super().__init__('mock_camera_publisher')

        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 10.0)

        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        fps = self.get_parameter('fps').get_parameter_value().double_value

        self.frame = self._make_checkerboard(self.width, self.height)

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.info_publisher = self.create_publisher(CameraInfo, 'camera/camera_info', 10)
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)

        self.get_logger().warn(
            "Publishing SYNTHETIC (fake, unmeasured) camera intrinsics on "
            "camera/camera_info for pipeline-wiring tests only - never use "
            "these for real geometric vision."
        )
        self.get_logger().info(f"Mock camera publishing {self.width}x{self.height} @ {fps}fps")

    @staticmethod
    def _make_checkerboard(width, height, square=40):
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        for y in range(0, height, square):
            for x in range(0, width, square):
                if (x // square + y // square) % 2 == 0:
                    frame[y:y + square, x:x + square] = (220, 220, 220)
        noise = np.random.randint(0, 15, frame.shape, dtype=np.uint8)
        return cv2.add(frame, noise)

    def _make_camera_info(self, stamp):
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = 'camera_optical_frame'
        info.width = self.width
        info.height = self.height
        # Plausible but entirely made-up pinhole intrinsics (~60deg HFOV),
        # only good enough to keep a downstream pipeline from dividing by
        # zero - not a substitute for a real camera_calibration run.
        fx = fy = float(self.width)
        cx, cy = self.width / 2.0, self.height / 2.0
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def timer_callback(self):
        stamp = self.get_clock().now().to_msg()

        img_msg = self.bridge.cv2_to_imgmsg(self.frame, encoding='bgr8')
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = 'camera_optical_frame'
        self.publisher.publish(img_msg)

        self.info_publisher.publish(self._make_camera_info(stamp))


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
