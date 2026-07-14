import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class JetbotYoloDetector(Node):
    """
    Object detection using YOLOv8/TensorRT.

    Publishes raw detections only - it does NOT drive the robot. A
    downstream planner (jetbot_nav's ground_plane_projector -> Nav2
    costmap -> controller) is what turns detections into safe motion.
    An earlier version of this node computed a proportional-steering
    Twist directly from bounding box position and published straight to
    cmd_vel, bypassing any obstacle/path safety reasoning entirely -
    that direct-to-motor path has been removed.
    """
    def __init__(self):
        super().__init__('jetbot_yolo_detector')

        self.declare_parameter('model_path', 'yolov8n.engine')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('target_class', '')  # '' = publish all detected classes

        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('conf_threshold').get_parameter_value().double_value
        self.target_class = self.get_parameter('target_class').get_parameter_value().string_value

        if YOLO is None:
            self.get_logger().error("Ultralytics not installed. Please run setup_vision_env.sh")
            self.model = None
        elif not os.path.exists(self.model_path):
            self.get_logger().error(f"Model file {self.model_path} not found. Please export it first.")
            self.model = None
        else:
            self.get_logger().info(f"Loading YOLO model: {self.model_path}")
            self.model = YOLO(self.model_path, task='detect')

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Detection2DArray, 'detections', 10)
        self.subscription = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10
        )

        self.get_logger().info("YOLO Detector node initialized.")

    def image_callback(self, msg):
        if self.model is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model.predict(
            source=frame,
            conf=self.conf_threshold,
            device=0,
            verbose=False,
            half=True,  # FP16 for TensorRT speedup
        )

        detections_msg = Detection2DArray()
        detections_msg.header = msg.header

        for box in results[0].boxes:
            cls = int(box.cls[0])
            label = self.model.names[cls]
            if self.target_class and label != self.target_class:
                continue

            xyxy = box.xyxy[0].cpu().numpy()
            detection = Detection2D()
            detection.header = msg.header
            detection.bbox.center.position.x = float((xyxy[0] + xyxy[2]) / 2)
            detection.bbox.center.position.y = float((xyxy[1] + xyxy[3]) / 2)
            detection.bbox.size_x = float(xyxy[2] - xyxy[0])
            detection.bbox.size_y = float(xyxy[3] - xyxy[1])

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = label
            hypothesis.hypothesis.score = float(box.conf[0])
            detection.results.append(hypothesis)

            detections_msg.detections.append(detection)

        if detections_msg.detections:
            self.publisher.publish(detections_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JetbotYoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
