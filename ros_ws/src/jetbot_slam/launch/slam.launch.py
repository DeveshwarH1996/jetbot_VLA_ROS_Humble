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
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',

                # External odom supplies metric scale; RTAB-Map does RGB
                # loop-closure only, not its own (unreliable-without-depth)
                # monocular visual odometry.
                'visual_odometry': 'false',
                'odom_topic': '/odom',

                'rgb_topic': '/camera/image_raw',
                'camera_info_topic': '/camera/camera_info',
                'subscribe_depth': 'false',
                'subscribe_rgbd': 'false',
                'subscribe_scan': 'false',  # no LiDAR yet

                'approx_sync': 'true',
                'qos': '2',

                # Ground robot constrained to the XY plane + yaw: this
                # matters a lot for a wheeled robot's pose graph quality.
                'rtabmap_args': '--Reg/Force3DoF true --Vis/MinInliers 15',

                'rviz': 'false',
            }.items(),
        ),
    ])
