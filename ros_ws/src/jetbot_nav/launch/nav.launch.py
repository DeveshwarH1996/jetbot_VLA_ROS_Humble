"""
Traditional (Nav2-based) navigation for JetBot.

Starts: a detections source (mock_detection_publisher in mock mode, or
the real yolo_detector), ground_plane_projector (camera detections ->
costmap obstacle points), and the standard Nav2 stack via nav2_bringup's
navigation_launch.py (controller/planner/behavior servers, bt_navigator,
velocity_smoother, lifecycle_manager - NOT map_server/amcl, see
config/nav2_params.yaml for why).

Nav2's output (cmd_vel_nav) is one of three sources jetbot_base's
motor_driver arbitrates between directly, selected by joy_controller's
mode - there is no separate arbiter node in this package anymore.

nav2_bringup's navigation_launch.py already remaps its internal 'cmd_vel'
to 'cmd_vel_nav' by default - verified against the installed launch file
source rather than assumed, after getting bitten by exactly this kind of
wrong-argument-name mistake in jetbot_slam's launch file.

Robot footprint and velocity limits are NOT hand-copied into
nav2_params.yaml - they're merged in from jetbot_base's
config/robot_params.yaml (the single shared source of truth also used by
motor_driver and joy_controller) at launch time, via _merge_nav2_params
below. Nav2's own launch API only accepts one params_file, so this reads
both YAMLs, overwrites the relevant nested keys, and writes a combined
file to hand to navigation_launch.py instead.

PREREQUISITES this launch file does NOT start itself:
  - jetbot_description's robot_state_publisher (static camera TF)
  - jetbot_base's bringup.launch.py (motor_driver, robot_localization,
    joy_controller - cmd_vel_nav only reaches the motors if motor_driver
    is running and its mode is set to 'traditional')
  - A camera node publishing camera/image_raw + camera/camera_info
    (mock_camera_publisher or camera_node - not started here, since the
    detections source below needs it and jetbot_base/bringup.launch.py
    already owns camera lifecycle)

Usage (mock mode, no hardware/ultralytics required):
  ros2 launch jetbot_nav nav.launch.py mock_mode:=true
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _merge_nav2_params(context):
    """
    Overwrite nav2_params.yaml's footprint/velocity placeholders with the
    real values from jetbot_base's robot_params.yaml, and hand Nav2 the
    merged result instead of the checked-in file directly.
    """
    robot_params_path = os.path.join(
        get_package_share_directory('jetbot_base'), 'config', 'robot_params.yaml')
    nav2_params_path = os.path.join(
        get_package_share_directory('jetbot_nav'), 'config', 'nav2_params.yaml')

    with open(robot_params_path) as f:
        robot_params = yaml.safe_load(f)['/**']['ros__parameters']
    with open(nav2_params_path) as f:
        nav2_params = yaml.safe_load(f)

    footprint = robot_params['footprint']
    max_linear_vel = robot_params['max_linear_vel']
    max_angular_vel = robot_params['max_angular_vel']

    nav2_params['local_costmap']['local_costmap']['ros__parameters']['footprint'] = footprint
    nav2_params['global_costmap']['global_costmap']['ros__parameters']['footprint'] = footprint

    # NOTE: nav2_regulated_pure_pursuit_controller has no general
    # "max_angular_vel" cap parameter in Humble - confirmed via
    # `ros2 param list /controller_server` against the actual running
    # node, not assumed. An earlier version of this config set one
    # anyway; it was silently ignored (unknown params in a YAML don't
    # error, they just never get declared/used). The real angular-rate
    # ceiling is enforced downstream by velocity_smoother's max_velocity
    # (set below) - rotate_to_heading_angular_vel only bounds the
    # explicit "rotate to face the path" behavior, a narrower thing.
    follow_path = nav2_params['controller_server']['ros__parameters']['FollowPath']
    follow_path['rotate_to_heading_angular_vel'] = max_angular_vel
    # Cruise speed deliberately below the hard max - leaves headroom for
    # the controller's own velocity regulation, not a duplicate of it.
    follow_path['desired_linear_vel'] = round(max_linear_vel * 0.75, 4)

    smoother_params = nav2_params['velocity_smoother']['ros__parameters']
    smoother_params['max_velocity'] = [max_linear_vel, 0.0, max_angular_vel]
    smoother_params['min_velocity'] = [-max_linear_vel, 0.0, -max_angular_vel]

    merged_path = os.path.join(tempfile.gettempdir(), 'jetbot_nav2_params_merged.yaml')
    with open(merged_path, 'w') as f:
        yaml.safe_dump(nav2_params, f)

    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={'params_file': merged_path}.items(),
        ),
    ]


def generate_launch_description():
    mock_mode = LaunchConfiguration('mock_mode')

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

        OpaqueFunction(function=_merge_nav2_params),
    ])
