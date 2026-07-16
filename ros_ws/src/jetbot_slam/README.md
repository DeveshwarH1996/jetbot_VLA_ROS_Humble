# jetbot_slam

Camera-based SLAM via RTAB-Map (`ros-humble-rtabmap-ros`). No custom nodes — this package is a launch file + config wrapping the upstream `rtabmap_launch` package correctly for a JetBot with a single monocular camera and no depth/LiDAR sensor.

## The scope limitation, up front

**A single RGB camera cannot give RTAB-Map real metric depth.** Per RTAB-Map's own documentation, a monocular camera can only support appearance-based loop-closure detection — actual metric graph optimization needs RGB-D, stereo, or an external source of scale. This package supplies that external scale from `jetbot_base`'s `/odom` (see that package's README for the accuracy caveat — it's open-loop dead-reckoning, no wheel encoders). `visual_odometry:=false` in the launch file is what selects this mode, instead of RTAB-Map's own (unreliable without depth) monocular visual odometry.

**What you get**: visual localization and loop-closure correction on top of the (drifty) wheel odometry.
**What you don't get**: an obstacle-aware occupancy grid for Nav2 — that still needs the planned LiDAR or a depth camera. `RGBD/Enabled:=false` in `rtabmap_args` explicitly selects RTAB-Map's "loop closure on images-only" mode rather than attempting full RGB-D SLAM.

## Launch

```bash
ros2 launch jetbot_slam slam.launch.py
```

**Prerequisites this launch file does *not* start itself** (a real gap discovered while testing this — RTAB-Map fails per-frame without them, with an unhelpful TF error):
1. `jetbot_description`'s `robot_state_publisher` (with the URDF loaded) — provides the static `base_footprint → chassis → camera_link` TF chain.
2. `jetbot_base`'s `bringup.launch.py` — provides `/odom` (from `motor_driver`) and the `odom → base_footprint` TF. That TF edge is published by `robot_localization`'s `ekf_node`, not `motor_driver` directly — `motor_driver`'s own `publish_tf` param is set `false` in `bringup.launch.py` so there's exactly one writer for the edge (see `jetbot_base`'s README). RTAB-Map only cares that the edge exists via TF, not who publishes it.
3. A camera node (`jetbot_vision`'s `camera_node` or `mock_camera_publisher`) — provides `camera/image_raw` + `camera/camera_info`.

**Arguments**: `database_path` (default `~/.ros/jetbot_slam.db`), `localization` (`false` = build a new map, `true` = localize against the existing database).

## Argument names that look plausible but are wrong

Two mistakes from an earlier pass, kept here as a warning since they don't error loudly — they just silently produce a rtabmap node waiting forever on a topic that will never exist:
- The depth toggle is `depth`, **not** `subscribe_depth`.
- `subscribe_rgb` defaults to whatever `depth` is set to — setting `depth:=false` without also setting `subscribe_rgb:=true` means nothing subscribes to the camera at all.

Verified end-to-end (mock camera + `bringup.launch.py` + `robot_state_publisher` + rtabmap, all running together): RTAB-Map processes frames continuously with no errors, and the full TF chain `map → odom → base_footprint → chassis → camera_link` resolves. Re-checked after the `robot_localization` EKF consolidation (which moved `odom → base_footprint` off `motor_driver` and onto `ekf_filter_node`, see `jetbot_base`'s README) specifically to confirm RTAB-Map doesn't care which node publishes that edge, only that it exists on TF — confirmed via `ros2 node info /jetbot_motor_driver` (no live `/tf` traffic, `publish_tf` param is `false`) vs `/ekf_filter_node` (actively broadcasting) while the chain above still resolved correctly.
