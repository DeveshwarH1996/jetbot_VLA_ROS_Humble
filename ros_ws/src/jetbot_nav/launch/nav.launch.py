"""
Traditional (Nav2-based) navigation for JetBot.

Starts: a detections source (mock_detection_publisher in mock mode, or
the real yolo_detector), ground_plane_projector (camera detections ->
costmap obstacle points), the standard Nav2 stack via nav2_bringup's
navigation_launch.py (controller/planner/behavior servers, bt_navigator,
velocity_smoother, lifecycle_manager - NOT map_server/amcl, see
config/nav2_params.yaml for why), and mode_arbiter (selects this
pipeline's output vs the VLA's as twist_mux's autonomous input).

nav2_bringup's navigation_launch.py already remaps its internal 'cmd_vel'
to 'cmd_vel_nav' by default - verified against the installed launch file
source rather than assumed, after getting bitten by exactly this kind of
wrong-argument-name mistake in jetbot_slam's launch file.

PREREQUISITES this launch file does NOT start itself:
  - jetbot_description's robot_state_publisher (static camera TF)
  - jetbot_base's bringup.launch.py (motor_driver, robot_localization,
    twist_mux - this file's mode_arbiter output only reaches the motors
    if twist_mux is also running)
  - A camera node publishing camera/image_raw + camera/camera_info
    (mock_camera_publisher or camera_node - not started here, since the
    detections source below needs it and jetbot_base/bringup.launch.py
    already owns camera lifecycle)

Usage (mock mode, no hardware/ultralytics required):
  ros2 launch jetbot_nav nav.launch.py mock_mode:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mock_mode = LaunchConfiguration('mock_mode')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    nav2_params = os.path.join(
        get_package_share_directory('jetbot_nav'), 'config', 'nav2_params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('mock_mode', default_value='true',
                               description='Use a synthetic detection instead of real YOLO/ultralytics'),

        Node(
            package='jetbot_vision',
            executable='mock_detection_publisher',
            name='mock_detection_publisher',
            condition=IfCondition(mock_mode),
            output='screen',
        ),
        Node(
            package='jetbot_vision',
            executable='yolo_detector',
            name='jetbot_yolo_detector',
            condition=UnlessCondition(mock_mode),
            output='screen',
        ),

        Node(
            package='jetbot_nav',
            executable='ground_plane_projector',
            name='ground_plane_projector',
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={'params_file': nav2_params}.items(),
        ),

        Node(
            package='jetbot_nav',
            executable='mode_arbiter',
            name='mode_arbiter',
            output='screen',
        ),
    ])
