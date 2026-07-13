"""
Master launch file for the JetBot VLA pipeline.

Node graph (see /home/srinivas/jetbot_project/launch_guide.md section 5):
  mock_camera_publisher / camera_node -> /camera/image_raw
  vla_client_bridge     -> /cmd_vel_vla
  mock_lidar_publisher / lidar driver -> /scan
  predictive_governor   subscribes /cmd_vel_vla + /scan -> publishes /cmd_vel_final
  teleop_twist_keyboard (run manually)                  -> /cmd_vel_joy
  twist_mux             subscribes /cmd_vel_final, /cmd_vel_joy, /cmd_vel_safety
                         -> publishes cmd_vel_out (remapped to /cmd_vel_mux)
  motor_driver           subscribes /cmd_vel_mux

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

    twist_mux_config = os.path.join(
        get_package_share_directory('jetbot_base'), 'config', 'twist_mux.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('mock_mode', default_value='true',
                               description='Use mock motors/camera/LiDAR instead of real hardware'),
        DeclareLaunchArgument('server_url', default_value='http://localhost:8000/predict',
                               description='VLA server prediction endpoint'),
        DeclareLaunchArgument('safety_threshold', default_value='0.4',
                               description='Governor veto distance in meters'),

        Node(
            package='jetbot_base',
            executable='motor_driver',
            name='jetbot_motor_driver',
            parameters=[{'use_mock': mock_mode}],
            output='screen',
        ),

        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            parameters=[twist_mux_config],
            remappings=[('cmd_vel_out', 'cmd_vel_mux')],
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
