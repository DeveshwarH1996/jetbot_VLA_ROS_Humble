# jetbot_nav

The "traditional pipeline": camera-fed Nav2 navigation, plus the arbiter that lets the operator choose between it and the VLA as the autonomous source. This exists because an earlier version of `jetbot_vision`'s `yolo_detector` computed steering directly from bounding-box position and published straight to the motors — no obstacle reasoning, no path planning, nothing standing between "I saw a thing" and "the wheels move." This package is what that got replaced with.

## Nodes

### `ground_plane_projector`

Turns 2D detections into 3D obstacle points for Nav2's costmap, by assuming each detection touches the floor and intersecting a ray through its bounding box's bottom-center pixel with the ground plane (standard pinhole-camera + ground-plane-assumption technique, sometimes called inverse perspective mapping).

**Subscribes**: `camera/camera_info` (`sensor_msgs/CameraInfo`), `detections` (`vision_msgs/Detection2DArray`, from `yolo_detector` or `mock_detection_publisher`).
**Publishes**: `obstacles/points` (`sensor_msgs/PointCloud2`, `base_footprint` frame) — Nav2's costmap obstacle layer consumes this directly, no custom costmap plugin needed.

**Why this needs `camera_optical_frame`, not `camera_link`**: `camera_link` follows REP103 (X-forward, Y-left, Z-up); pixel math and `CameraInfo`'s K matrix assume the standard optical convention (X-right, Y-down, Z-forward-into-scene). `jetbot_description`'s URDF has a fixed `camera_optical_joint` doing that conversion, and this node looks the transform up via TF rather than hardcoding the camera's mount angle in code — if the camera ever gets remounted, only the URDF needs to change.

**⚠️ Real limitation, not a bug**: this only locates obstacles that touch the floor within view. It cannot see overhangs, and it's meaningfully less accurate than real depth/LiDAR — treat it as a bridge until LiDAR is physically on the robot, not a permanent substitute. Per-project decision: build with the camera as the primary obstacle source for now; LiDAR integration is separate future work.

**Bug found during verification, worth knowing if you touch this code**: `tf2_geometry_msgs.do_transform_vector3()` mutates the `TransformStamped` it's given — it zeroes out the translation in place, since a vector transform is rotation-only by definition. The first version of this node read `origin = transform.transform.translation` *before* calling `do_transform_vector3()`, but that's a Python reference, not a copy — `origin` silently became `(0, 0, 0)` retroactively the moment `do_transform_vector3()` ran, and every projected point landed exactly on the camera's own base position. It produced a valid `PointCloud2` with no errors or warnings; the only way this surfaced was checking the actual coordinate values, not just that a message was published. Fixed by copying the three scalars out immediately after the TF lookup, before any vector-transform call touches the object.

### `mode_arbiter`

Selects exactly one of two peer autonomous sources and republishes only that one, so the operator can choose "drive via the traditional planner" or "drive via the VLA" as an explicit either/or, not an implicit blend.

**Subscribes**: `cmd_vel_nav` (Nav2's controller output), `cmd_vel_final` (the VLA's safety-checked output, from `jetbot_governor`).
**Publishes**: `cmd_vel_autonomous` — this is `twist_mux`'s single autonomous-tier input (see `jetbot_base/config/twist_mux.yaml`).
**Parameter**: `mode` (`'traditional'` or `'vla'`, default `'traditional'`). Switch live: `ros2 param set /mode_arbiter mode vla`.

**Why this node exists instead of using `twist_mux`'s own locks**: `twist_mux`'s "lock" mechanism blocks everything *at or below* a priority level when engaged — it's an e-stop/veto primitive, not a per-channel mute. It can't cleanly express "exactly one of these two peer channels is live" without also affecting priority tiers you didn't mean to touch (e.g. a lock strong enough to silence the VLA would also silence the traditional planner, since both sit below the joystick). `mode_arbiter` does the selection *before* `twist_mux` sees either signal, so `twist_mux`'s own priority order (safety > joystick > autonomous) is untouched and the joystick still always overrides whichever autonomous mode is currently selected.

## Nav2 configuration (`config/nav2_params.yaml`)

Standard Nav2 stack (`controller_server`, `planner_server`, `behavior_server`, `bt_navigator`, `velocity_smoother`, `lifecycle_manager`) via `nav2_bringup`'s `navigation_launch.py` — **no `map_server`/`amcl`**. `jetbot_slam`'s RTAB-Map does visual localization/loop-closure only (monocular, no depth — see that package's README), not an occupancy grid, so there's no static map to localize against yet. Both costmaps (local and global) run in rolling-window mode directly off the live camera-derived obstacle cloud instead.

Controller: `nav2_regulated_pure_pursuit_controller` — simpler to tune than DWB/MPPI, a reasonable default for a small differential-drive robot with no manipulator. Footprint and velocity limits are matched to `jetbot_description`'s URDF and `jetbot_base`'s `motor_driver` parameters — **keep these in sync if either changes**, nothing currently enforces that automatically.

Verified against the actual installed `nav2_bringup` launch source before wiring it up (its `navigation_launch.py` already remaps `cmd_vel` → `cmd_vel_nav` internally by default — this project hit exactly this kind of wrong-launch-argument mistake once already with `rtabmap_launch`, so this one was checked against the real source first rather than assumed).

**Two more bugs found by actually launching this, not just reading Nav2 docs**: (1) `nav2_navfn_planner::NavfnPlanner` and `nav2_behaviors::Spin`/`BackUp`/`Wait` (C++-namespace plugin names) don't work — pluginlib only has these two packages registered under their `package/ClassName` form (`nav2_navfn_planner/NavfnPlanner`, `nav2_behaviors/Spin`, etc.), even though several *other* Nav2 plugins (the controller, costmap layers) accept both forms. `planner_server`/`behavior_server` fail to configure with a `FATAL` error naming the exact valid declared types — that's how this got caught and fixed, not by knowing the convention in advance. (2) `nav2_bringup`'s `navigation_launch.py` unconditionally starts a `smoother_server` node alongside the others, which this config's `lifecycle_manager.node_names` list initially omitted — it stayed configured but never activated until added.

## Launch

```bash
ros2 launch jetbot_nav nav.launch.py mock_mode:=true
```

**Prerequisites this launch file does not start itself**: `jetbot_description`'s `robot_state_publisher`, `jetbot_base`'s `bringup.launch.py` (motor driver, EKF, `twist_mux` — without `twist_mux` running, `mode_arbiter`'s output has nowhere to go), and a camera node publishing `camera/image_raw` + `camera/camera_info`.

`mock_mode:=true` (default) starts `mock_detection_publisher` instead of the real `yolo_detector` — no `ultralytics`/exported model/real camera needed to test the costmap/planner wiring.

## Verified

All four processes (`bringup.launch.py`, `robot_state_publisher`, `nav.launch.py`, plus the mock VLA server) running together, with a synthetic detection: the mock bounding box correctly projects to a real ground point (`~0.18m` ahead, matching an independent hand-calculation of the same ray/plane intersection), Nav2's local costmap marks it (81 occupied cells at value 100), `Managed nodes are active` for the full lifecycle-managed Nav2 stack, `mode_arbiter` correctly forwards the VLA's output only when switched to `'vla'` mode (confirmed via live-changing `angular.z` values, not just a single static sample), and the joystick still preempts whichever autonomous mode is selected through the new `twist_mux` `autonomous` tier.
