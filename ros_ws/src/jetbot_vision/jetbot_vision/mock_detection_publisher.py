import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class MockDetectionPublisher(Node):
    """
    Publishes a synthetic detection on 'detections' so the downstream
    ground-plane-projection -> Nav2 costmap chain can be exercised without
    ultralytics/a real camera. Bounding box defaults roughly simulate a
    person-sized object a couple meters ahead in a 640x480 frame.
    """
    def __init__(self):
        super().__init__('mock_detection_publisher')

        self.declare_parameter('class_id', 'person')
        self.declare_parameter('bbox_center_x', 340.0)
        self.declare_parameter('bbox_center_y', 380.0)
        self.declare_parameter('bbox_width', 80.0)
        self.declare_parameter('bbox_height', 180.0)
        self.declare_parameter('score', 0.9)
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('publish', True)

        self.publisher = self.create_publisher(Detection2DArray, 'detections', 10)
        rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / rate_hz, self.timer_callback)

        self.get_logger().info(
            "Mock detection publisher active. Set 'publish:=false' via "
            "ros2 param set to simulate a clear frame."
        )

    def timer_callback(self):
        if not self.get_parameter('publish').get_parameter_value().bool_value:
            return

        msg = Detection2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_optical_frame'

        detection = Detection2D()
        detection.header = msg.header
        detection.bbox.center.position.x = self.get_parameter(
            'bbox_center_x').get_parameter_value().double_value
        detection.bbox.center.position.y = self.get_parameter(
            'bbox_center_y').get_parameter_value().double_value
        detection.bbox.size_x = self.get_parameter('bbox_width').get_parameter_value().double_value
        detection.bbox.size_y = self.get_parameter('bbox_height').get_parameter_value().double_value

        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = self.get_parameter('class_id').get_parameter_value().string_value
        hypothesis.hypothesis.score = self.get_parameter('score').get_parameter_value().double_value
        detection.results.append(hypothesis)

        msg.detections.append(detection)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockDetectionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
