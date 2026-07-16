"""
Camera-based SLAM for JetBot via RTAB-Map.

IMPORTANT SCOPE NOTE: a single monocular camera cannot give RTAB-Map real
metric depth. Per RTAB-Map's own docs, a mono camera can only support
appearance-based loop-closure detection - metric graph optimization needs
either RGB-D, stereo, or an external odometry source for scale. We use
jetbot_base's /odom (currently open-loop dead-reckoning, since the
Waveshare kit has no wheel encoders - see motor_driver.py) as that external
metric source, and let RTAB-Map's RGB pipeline handle loop-closure
recognition and drift correction on top of it. `visual_odometry:=false`
below is what selects this mode (external odom instead of RTAB-Map's own
mono VO, which would drift badly on its own).

This gives visual localization / loop closure, NOT an obstacle-aware
occupancy grid for Nav2 - that still needs the planned LiDAR or a depth
camera.

PREREQUISITES this launch file does NOT start itself:
  - jetbot_description's robot_state_publisher (with the URDF loaded) must
    already be running, so the static base_footprint->chassis->camera_link
    TF chain exists. Without it, rtabmap fails per-frame with "camera_link
    passed to lookupTransform ... does not exist".
  - jetbot_base's bringup.launch.py must be running: motor_driver publishes
    /odom, and robot_localization's ekf_node (not motor_driver directly -
    motor_driver's publish_tf is false in bringup.launch.py) broadcasts the
    odom->base_footprint TF that rtabmap actually looks up.
  - A camera node (real camera_node or jetbot_vision's mock_camera_publisher)
    must be publishing camera/image_raw + camera/camera_info.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    rtabmap_share = get_package_share_directory('rtabmap_launch')

    return LaunchDescription([
        DeclareLaunchArgument('database_path', default_value='~/.ros/jetbot_slam.db'),
        DeclareLaunchArgument('localization', default_value='false',
                               description='true = localize against an existing map instead of building a new one'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rtabmap_share, 'launch', 'rtabmap.launch.py')
            ),
            launch_arguments={
                'database_path': LaunchConfiguration('database_path'),
                'localization': LaunchConfiguration('localization'),

                'frame_id': 'base_footprint',
                # Setting odom_frame_id makes rtabmap read odometry from TF
                # (odom->base_footprint, broadcast by motor_driver) instead
                # of the /odom topic directly - odom_topic is then unused.
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',

                # External odom supplies metric scale; RTAB-Map does RGB
                # loop-closure only, not its own (unreliable-without-depth)
                # monocular visual odometry.
                'visual_odometry': 'false',

                'rgb_topic': '/camera/image_raw',
                'camera_info_topic': '/camera/camera_info',
                'depth': 'false',        # no depth sensor - this is the real arg name, NOT subscribe_depth
                # subscribe_rgb defaults to whatever 'depth' is (see rtabmap_launch source),
                # so it must be set explicitly here or nothing subscribes to the camera at all.
                'subscribe_rgb': 'true',
                'subscribe_rgbd': 'false',
                'subscribe_scan': 'false',  # no LiDAR yet

                'approx_sync': 'true',
                'qos': '2',

                # Ground robot constrained to the XY plane + yaw, and
                # RGBD/Enabled=false selects RTAB-Map's "loop closure on
                # images-only" mode (its own suggestion when no
                # depth/stereo/rgbd is subscribed) instead of full RGB-D SLAM.
                'rtabmap_args': '--Reg/Force3DoF true --Vis/MinInliers 15 --RGBD/Enabled false',

                'rviz': 'false',
                'rtabmap_viz': 'false',  # separate GUI from rviz; also off for headless/automated runs
            }.items(),
        ),
    ])
