import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from vision_msgs.msg import Detection2DArray
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_vector3
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Header


class GroundPlaneProjector(Node):
    """
    Turns 2D object detections into 3D obstacle points for Nav2's costmap,
    by assuming each detection touches the floor and intersecting a ray
    through its bounding box's bottom-center pixel with the ground plane.

    LIMITATION (inherent to a single RGB camera, not a bug): this only
    locates obstacles that touch the floor in view - it cannot see
    overhangs, and is meaningfully less accurate than real depth/LiDAR.
    It's a bridge until LiDAR is physically on the robot, not a
    replacement for it.
    """

    GROUND_FRAME = 'base_footprint'

    def __init__(self):
        super().__init__('ground_plane_projector')

        self.declare_parameter('camera_optical_frame', 'camera_optical_frame')
        self.optical_frame = self.get_parameter(
            'camera_optical_frame').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.camera_info = None
        self.info_sub = self.create_subscription(
            CameraInfo, 'camera/camera_info', self.info_callback, 10)
        self.detections_sub = self.create_subscription(
            Detection2DArray, 'detections', self.detections_callback, 10)
        self.points_pub = self.create_publisher(PointCloud2, 'obstacles/points', 10)

        self.get_logger().info("Ground plane projector active.")

    def info_callback(self, msg):
        self.camera_info = msg

    def detections_callback(self, msg):
        if self.camera_info is None:
            self.get_logger().warn(
                "No camera_info yet - can't project detections.", throttle_duration_sec=5.0)
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.GROUND_FRAME, self.optical_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.2))
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}", throttle_duration_sec=5.0)
            return

        # Copy scalars, NOT a reference to transform.transform.translation:
        # do_transform_vector3() below mutates the Transform it's given
        # (it zeroes the translation in place, since a vector transform is
        # rotation-only by definition) - keeping a plain reference here
        # silently went to (0, 0, 0) after the first do_transform_vector3()
        # call, found only by adding debug logging and comparing against a
        # standalone replication of the same math.
        origin_x = transform.transform.translation.x
        origin_y = transform.transform.translation.y
        origin_z = transform.transform.translation.z
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        points = []
        for detection in msg.detections:
            u = detection.bbox.center.position.x
            # Bottom edge of the box: the assumed ground-contact pixel.
            v = detection.bbox.center.position.y + detection.bbox.size_y / 2.0

            ray = Vector3Stamped()
            ray.header.frame_id = self.optical_frame
            ray.vector.x = (u - cx) / fx
            ray.vector.y = (v - cy) / fy
            ray.vector.z = 1.0
            ray_in_ground_frame = do_transform_vector3(ray, transform)
            direction = ray_in_ground_frame.vector

            if direction.z >= -1e-6:
                # Ray points level or upward - never reaches the ground
                # plane (e.g. something mounted above the camera's view).
                continue

            t = -origin_z / direction.z
            x = origin_x + t * direction.x
            y = origin_y + t * direction.y
            points.append((x, y, 0.0))

        if not points:
            return

        header = Header(stamp=msg.header.stamp, frame_id=self.GROUND_FRAME)
        cloud = point_cloud2.create_cloud_xyz32(header, points)
        self.points_pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = GroundPlaneProjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
