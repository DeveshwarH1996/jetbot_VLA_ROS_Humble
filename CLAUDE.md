# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A Waveshare JetBot (Jetson Nano 4GB) driven by natural-language instructions: the Jetson runs reactive/safety
control in ROS2, and a separate GPU machine (not part of this repo) runs the high-level "brain" (a VLA/navigation
model), reached over HTTP. Status: the ROS2 graph is fully built and verified end-to-end against mocks. The
Jetson itself has not been flashed, and no real hardware (LiDAR, calibrated camera) is connected yet — see the
top-level README's "Known gaps" section, which is kept current and should be checked/updated as gaps close.

## Standing instructions: planning & documentation

- **This file is overarching-only.** It covers whole-system architecture: what's connected, how nodes/packages
  communicate, and system-wide conventions that span more than one package. It does **not** contain package-internal
  detail — topics, parameters, node internals, package-specific bug postmortems, or package-specific plans. That
  material lives in each package's own `README.md` (and plan markdowns, see below). If you're about to add
  package-internal detail here, put it in that package's README instead.
- **Plan before building.** Before starting any non-trivial change (new feature, architecture change, a bug whose
  fix isn't obvious/local), use the superpowers skill system as appropriate — brainstorming for design work,
  systematic-debugging for bugs, etc. — to think it through before writing code.
- **Store the plan in the package it affects**, as a markdown file inside that package's directory (e.g.
  `ros_ws/src/jetbot_nav/plans/<topic>.md`), not at the repo root and not inside this file. Work spanning multiple
  packages gets a plan in the most-affected package, cross-referenced from the others' READMEs.
- **Keep this file promptly in sync.** Any plan that changes the overarching picture — new/removed node, new/changed
  topic between packages, a new shared convention, a changed data flow — must be reflected here as soon as that
  change lands, not batched up for later. Additions, subtractions, and modifications to a plan all count, not just
  new plans. Purely internal-to-one-package changes belong in that package's README only.

## Build, test, run

```bash
# Environment gotcha: use system /usr/bin/python3 (3.10), not a conda/pyenv python that
# may be earlier in PATH — rclpy's compiled extension is built against the system
# interpreter and fails with ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
# under a mismatched Python.

cd ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# Full test suite
colcon test --packages-select jetbot_governor jetbot_base jetbot_vision jetbot_vla_bridge jetbot_nav

# Single package
colcon test --packages-select jetbot_governor
colcon test-result --verbose
```

`jetbot_governor`'s `test_predictive_governor.py` is the one package with real behavioral unit tests
(front-arc distance logic, edge cases). The rest of the suite is ROS2's standard
`ament_flake8`/`ament_pep257`/`ament_copyright` lint boilerplate — there's a known pre-existing docstring-style
lint backlog across most files, not functional bugs, not yet cleaned up.

`simulation/sim_test_suite.md` documents 3 scenario-level tests (Wall/Override/Latency) manually verified against
a live launch — not yet automated as `launch_testing`.

### Running the full mock pipeline (no hardware required)

```bash
# Terminal 1 — mock VLA server (plain Python, not a ROS2 node)
cd simulation && python3 mock_vla_server.py

# Terminal 2 — main pipeline
source /opt/ros/humble/setup.bash && source ros_ws/install/setup.bash
ros2 launch jetbot_base bringup.launch.py mock_mode:=true server_url:=http://localhost:8000/predict

# Optional — RViz
ros2 launch jetbot_description display.launch.py

# Optional — SLAM (needs robot_state_publisher + bringup already running)
ros2 launch jetbot_slam slam.launch.py

# Optional — traditional Nav2 pipeline (needs robot_state_publisher + bringup already running)
ros2 launch jetbot_nav nav.launch.py mock_mode:=true
```

Without a physical joystick, drive by publishing synthetic `sensor_msgs/Joy` messages (button 7 = mode cycle,
axis 1 = linear, axis 0 = angular — see `jetbot_base`'s README before trusting these against real hardware):

```bash
ros2 topic pub /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.8, 0,0,0,0,0,0], buttons: [0,0,0,0,0,0,0,1,0,0,0]}" -r 20
```

Extra apt dependencies beyond `ros-humble-desktop` (see README for the full install block):
`ros-humble-joy`, `ros-humble-rtabmap-ros`, `ros-humble-camera-info-manager-py`, `ros-humble-nav2-bringup`,
`ros-humble-vision-msgs`, `ros-humble-robot-localization`. Mock VLA server needs
`pip install fastapi uvicorn python-multipart`.

## Architecture

### No mux node — `motor_driver` is the arbiter

There is no `twist_mux` (removed) and no separate arbiter node. `jetbot_base/motor_driver` subscribes directly to
all three candidate velocity sources — `cmd_vel_joy` (manual), `cmd_vel_final` (VLA, via governor),
`cmd_vel_nav` (traditional/Nav2) — and follows whichever one `joy_controller/mode` (`std_msgs/String`, one of
`vla`/`manual`/`traditional`) currently selects. `jetbot_base/joy_controller` reads `/joy` and cycles the mode on
a button press (edge-detected, not level-triggered); it publishes both `joy_controller/mode` and `cmd_vel_joy` on
*every* `/joy` message, not just on change — this is what lets `motor_driver` detect "joystick still connected"
from message recency alone.

**Two distinct, deliberately different fail-safes in `motor_driver`:**
1. Selected source goes stale (no message within `command_timeout`) → stop.
2. `joy_controller/mode` itself goes stale (joystick unplugged/crashed) → **stop unconditionally**, regardless
   of which mode was active. Losing the joystick means losing the human override channel, so autonomous driving
   must not continue on the theory that the VLA/planner output still looks fine.

When touching arbitration logic, read `jetbot_base`'s README in full first — this behavior was arrived at
deliberately after an explicit design discussion, not by default.

### Physical robot constants live in one place

`jetbot_base/config/robot_params.yaml` (`wheel_base`, `max_linear_vel`, `max_angular_vel`, `footprint`) is the
single shared source of truth, loaded via ROS2's `/**:` wildcard so any node with a matching declared parameter
name picks it up. `bringup.launch.py` loads it for both `motor_driver` and `joy_controller`; `jetbot_nav`'s
`nav.launch.py` merges it into Nav2's params at launch time via `_merge_nav2_params` (an `OpaqueFunction`) since
Nav2's launch API only accepts one `params_file` — it reads both YAMLs, overwrites the nested footprint/velocity
keys, and writes a combined temp file that actually gets launched. **This file is not auto-derived from
`jetbot_description`'s URDF** — these values have drifted before (see that package's README), so if you change
robot dimensions, update both by hand.

### Vision → planning split (not vision → motors)

`yolo_detector` (or `mock_detection_publisher`) publishes `vision_msgs/Detection2DArray` on `detections` — it
does **not** compute steering or touch `cmd_vel` directly (an earlier version did; this was explicitly identified
as wrong and replaced). `jetbot_nav/ground_plane_projector` turns detections into 3D obstacle points by
ray/ground-plane intersection (assumes each detection touches the floor) and publishes `sensor_msgs/PointCloud2`
on `obstacles/points`, which Nav2's costmap layers consume directly — no custom costmap plugin. This needs the TF
lookup for `camera_optical_frame` (see below) and live `CameraInfo` intrinsics (`fx`/`fy`/`cx`/`cy` from `k`, not
hardcoded) — it checks for an uncalibrated all-zero K matrix and refuses to project rather than dividing by zero.

### `camera_optical_frame` vs `camera_link`

`camera_link` follows REP103 (X-forward/Y-left/Z-up, like the rest of the robot). Pixel math and `CameraInfo`'s K
matrix assume the standard optical convention (X-right/Y-down/Z-forward-into-scene). `jetbot_description`'s URDF
has a fixed `camera_optical_joint` doing that conversion; camera nodes stamp images with `camera_optical_frame`,
and `ground_plane_projector` looks the transform up via TF rather than hardcoding the mount angle — if the camera
is remounted, only the URDF needs to change.

### No occupancy-grid map / no AMCL

`jetbot_slam` runs RTAB-Map in **monocular** mode — visual localization/loop-closure only, no depth camera, so no
occupancy grid. Nav2 therefore runs costmap-only (`jetbot_nav/config/nav2_params.yaml` has no
`map_server`/`amcl`), with both local and global costmaps in rolling-window mode fed directly by
`ground_plane_projector`'s live obstacle cloud instead of a static map.

### Single-writer-per-TF-edge

`motor_driver` can publish `odom → base_footprint` TF itself (`publish_tf` param), but in `bringup.launch.py`
this is set `false` because `robot_localization`'s `ekf_node` fuses `motor_driver`'s `/odom` into
`/odometry/filtered` and owns that TF edge instead. `jetbot_slam` continues to own `map → odom` unchanged and
reads `odom → base_footprint` via TF regardless of which node is currently publishing it. If you add a node that
touches TF, check who currently owns the edge first — this project has exactly one writer per edge, deliberately.

### `/odom` is open-loop, not measured

The Waveshare kit's TT motors have no encoders. `motor_driver`'s `/odom` is integrated from *commanded* velocity,
not measured wheel motion — it drifts steadily and is published with large fixed covariance on purpose. Treat it
as a weak prior; `robot_localization` fuses it but doesn't make it accurate.

### Package map

| Package | Role |
|---|---|
| `jetbot_base` | `joy_controller`, `motor_driver` (arbitration + odom), EKF config, master `bringup.launch.py` |
| `jetbot_vision` | `camera_node`/`mock_camera_publisher`, `yolo_detector`/`mock_detection_publisher` |
| `jetbot_governor` | `predictive_governor` (LiDAR-distance safety veto on VLA output — "the WAM layer"), `mock_lidar_publisher` |
| `jetbot_vla_bridge` | `vla_client_bridge` — HTTP client to the external VLA/navigation server |
| `jetbot_nav` | `ground_plane_projector`, Nav2 config/launch (traditional pipeline) |
| `jetbot_slam` | RTAB-Map monocular SLAM launch/config |
| `jetbot_description` | URDF, RViz config, `display.launch.py` |

Each package has its own README with node-level detail (topics, parameters, known gaps, bug postmortems from
actual verification — these are worth reading before touching that package, they document real bugs found by
running the code, not hypothetical concerns). The top-level `README.md` has the full node communication diagram
and the authoritative "Known gaps" list — keep both current when architecture changes.

## Working conventions specific to this project

- **Verify against the running system, not just the code.** Multiple real bugs in this repo were only caught by
  checking live `ros2 param get`/`ros2 topic echo` output or actual published values against what the YAML/code
  seemed to say — Nav2 silently drops unknown params, `do_transform_vector3()` mutates its input in place, etc.
  Reading the code and confirming it "looks right" has repeatedly not been enough on this project.
- Kyogin/any specific personal machine name should never be hardcoded into configs or docs — the GPU server is
  referred to generically (`server_url`, "GPU server") so the repo works for anyone's setup.
- Camera-as-primary-obstacle-source is a deliberate, current decision (not a placeholder) — real LiDAR
  integration is explicitly deferred, separate future work, not an oversight to "fix."
