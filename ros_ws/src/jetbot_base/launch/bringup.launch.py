"""
Master launch file for the JetBot's core control loop.

Node graph:
  joy_node (ros-humble-joy)      -> /joy
  joy_controller                 -> joy_controller/mode, cmd_vel_joy
  mock_camera_publisher / camera_node -> /camera/image_raw
  vla_client_bridge     -> /cmd_vel_vla
  mock_lidar_publisher / lidar driver -> /scan
  predictive_governor   subscribes /cmd_vel_vla + /scan -> publishes /cmd_vel_final
  motor_driver           -> /odom (open-loop) -> ekf_filter_node -> /odometry/filtered
                            + odom->base_footprint TF (motor_driver's own TF broadcast
                            is disabled via publish_tf:=false; the EKF owns it instead)
  motor_driver           subscribes joy_controller/mode, cmd_vel_joy, cmd_vel_final,
                          cmd_vel_nav - follows whichever one the mode selects, no
                          separate mux node (see motor_driver.py's own docstring for
                          why joystick loss always means stop, regardless of mode)

NOTE: this file does NOT start jetbot_nav's nav.launch.py (Nav2 + the
traditional planner). cmd_vel_final and cmd_vel_nav only reach the motors
if their respective producer is running AND joy_controller's mode
selects them - without jetbot_nav's launch file running alongside this
one, 'traditional' mode has no cmd_vel_nav publisher and motor_driver
fails safe (stops) rather than silently doing nothing. See jetbot_nav's
README.

joy_node needs a real joystick device (/dev/input/jsX) - it will log
errors without one attached. To exercise the mode-cycling/arbitration
logic without physical hardware, publish synthetic sensor_msgs/Joy
messages directly instead of running joy_node (see jetbot_base's README).

Usage (mock mode, no hardware required):
  ros2 launch jetbot_base bringup.launch.py mock_mode:=true \
      server_url:=http://localhost:8000/predict
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mock_mode = LaunchConfiguration('mock_mode')
    server_url = LaunchConfiguration('server_url')
    safety_threshold = LaunchConfiguration('safety_threshold')

    ekf_config = os.path.join(
        get_package_share_directory('jetbot_base'), 'config', 'ekf.yaml'
    )
    joy_controller_config = os.path.join(
        get_package_share_directory('jetbot_base'), 'config', 'joy_controller.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('mock_mode', default_value='true',
                               description='Use mock motors/camera/LiDAR instead of real hardware'),
        DeclareLaunchArgument('server_url', default_value='http://localhost:8000/predict',
                               description='VLA server prediction endpoint'),
        DeclareLaunchArgument('safety_threshold', default_value='0.4',
                               description='Governor veto distance in meters'),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),

        Node(
            package='jetbot_base',
            executable='joy_controller',
            name='joy_controller',
            parameters=[joy_controller_config],
            output='screen',
        ),

        Node(
            package='jetbot_base',
            executable='motor_driver',
            name='jetbot_motor_driver',
            parameters=[{'use_mock': mock_mode, 'publish_tf': False}],
            output='screen',
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_config],
            output='screen',
        ),

        Node(
            package='jetbot_governor',
            executable='predictive_governor',
            name='predictive_governor',
            parameters=[{'safety_threshold': safety_threshold}],
            output='screen',
        ),

        Node(
            package='jetbot_vla_bridge',
            executable='vla_client_bridge',
            name='vla_client_bridge',
            parameters=[{'server_url': server_url}],
            output='screen',
        ),

        # Mock mode: synthetic camera + LiDAR, no hardware required.
        Node(
            package='jetbot_vision',
            executable='mock_camera_publisher',
            name='mock_camera_publisher',
            condition=IfCondition(mock_mode),
            output='screen',
        ),
        Node(
            package='jetbot_governor',
            executable='mock_lidar_publisher',
            name='mock_lidar_publisher',
            condition=IfCondition(mock_mode),
            output='screen',
        ),

        # Real hardware: actual CSI camera.
        # NOTE: no real LiDAR driver node exists in this repo yet
        # (action_plan.md Phase 1.4) - mock_mode:=false will run without
        # /scan until that driver package is added, so predictive_governor
        # will veto every forward command (fails safe, see scan_timeout).
        Node(
            package='jetbot_vision',
            executable='camera_node',
            name='jetbot_camera_node',
            condition=UnlessCondition(mock_mode),
            output='screen',
        ),
    ])
