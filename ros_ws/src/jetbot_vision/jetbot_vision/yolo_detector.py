import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import os
import time

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

class JetbotYoloDetector(Node):
    """
    Performs object detection using YOLOv8 TensorRT.
    Outputs detected objects and can optionally send simple steering commands.
    """
    def __init__(self):
        super().__init__('jetbot_yolo_detector')

        # Parameters
        self.declare_parameter('model_path', 'yolov8n.engine')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('target_class', 'person') # e.g. 'person', 'bottle'
        self.declare_parameter('use_tensorrt', True)

        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('conf_threshold').get_parameter_value().double_value
        self.target_class = self.get_parameter('target_class').get_parameter_value().string_value

        # Load Model
        if YOLO is None:
            self.get_logger().error("Ultralytics not installed. Please run setup_vision_env.sh")
            self.model = None
        else:
            if not os.path.exists(self.model_path):
                self.get_logger().error(f"Model file {self.model_path} not found. Please export it first.")
                self.model = None
            else:
                self.get_logger().info(f"Loading YOLO model: {self.model_path}")
                self.model = YOLO(self.model_path, task='detect')

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscriber to camera stream
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )
        
        self.get_logger().info("YOLO Detector node initialized.")

    def image_callback(self, msg):
        if self.model is None:
            return

        # Convert ROS Image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Inference
        results = self.model.predict(
            source=frame, 
            conf=self.conf_threshold, 
            device=0, 
            verbose=False,
            half=True # Use FP16 for TensorRT speedup
        )

        # Process detections
        detections = results[0].boxes
        if len(detections) > 0:
            # Find target object
            target_box = None
            for box in detections:
                cls = int(box.cls[0])
                label = self.model.names[cls]
                if label == self.target_class:
                    target_box = box
                    break
            
            if target_box is not None:
                self.handle_detection(target_box, frame)

    def handle_detection(self, box, frame):
        # Calculate center of the box
        xyxy = box.xyxy[0].cpu().numpy()
        center_x = (xyxy[0] + xyxy[2]) / 2
        
        # map center_x to angular velocity (simple P-controller)
        # Frame width is usually 640
        img_width = frame.shape[1]
        error = (center_x - (img_width / 2)) / (img_width / 2) # range [-1, 1]
        
        cmd = Twist()
        cmd.linear.x = 0.1 # Constant slow move forward
        cmd.angular.z = -error * 0.5 # Proportional turn to center object
        
        self.publisher.publish(cmd)
        self.get_logger().info(f"Target {self.target_class} found! Steering: {cmd.angular.z:.2f}")

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
