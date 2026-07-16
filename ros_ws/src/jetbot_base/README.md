# jetbot_base

Motor control, odometry, and joystick input for the Waveshare JetBot AI Kit.

## Nodes

### `joy_controller`

Single source of truth for "what should be driving the robot right now" — replaces `twist_mux` + `jetbot_nav`'s old `mode_arbiter`. Subscribes to `/joy` (`sensor_msgs/Joy`, from the standard `joy` package reading the physical controller) and cycles through three modes on a configurable button press: `vla` → `manual` → `traditional` → back to `vla`. In `manual` mode it converts stick axes into a driving `Twist`.

**Subscribes**: `joy` (`sensor_msgs/Joy`).
**Publishes**: `joy_controller/mode` (`std_msgs/String`, one of `vla`/`manual`/`traditional`), `cmd_vel_joy` (`geometry_msgs/Twist`).

Both are published on **every** `/joy` message, not just on change — this matters. `joy_node` keeps `/joy` arriving continuously (autorepeat) as long as the physical controller is connected, and `motor_driver`'s staleness checks depend on that steady stream to tell "controller still connected" from "controller lost" — the same role `twist_mux`'s per-topic timeouts used to play.

**Parameters** — hardware-specific ones from `config/joy_controller.yaml`, physical-robot ones from the shared `config/robot_params.yaml` (see below)
| Name | Default | Source | Meaning |
|---|---|---|---|
| `mode_button` | `7` | `joy_controller.yaml` | Button index that cycles modes (Xbox: Start/Menu — deliberate, not a face button) |
| `linear_axis` | `1` | `joy_controller.yaml` | Axis index for forward/back (Xbox: left stick vertical) |
| `angular_axis` | `0` | `joy_controller.yaml` | Axis index for turning (Xbox: left stick horizontal) |
| `max_linear_vel` | `0.2` | `robot_params.yaml` | Scale factor for the linear axis — same parameter name and value as `motor_driver`'s, not a separately-tracked copy |
| `max_angular_vel` | `1.0` | `robot_params.yaml` | Scale factor for the angular axis, likewise shared with `motor_driver` |
| `deadzone` | `0.05` | `joy_controller.yaml` | Axis values smaller than this are treated as zero |

**⚠️ Button/axis indices are not verified against physical hardware in this dev environment.** Defaults come from `ros-humble-teleop_twist_joy`'s own reference `xbox.config.yaml` (`axis_linear=1`, `axis_angular=0`), which should be right for a standard Xbox controller via the Linux joystick driver — but run `ros2 topic echo /joy`, move the stick and press the button you intend, and confirm the indices actually match before trusting them.

Mode cycling is edge-detected (only advances on a 0→1 transition of `mode_button`), not level-triggered — `/joy` reports button state continuously, so without edge detection, holding the button down for even 100ms at a typical 20-50Hz publish rate would cycle through several modes in one press.

### `motor_driver`

Drives the physical (or mock) motors, and is now the arbiter too — no separate mux node. Follows whichever of `cmd_vel_joy` / `cmd_vel_final` (VLA, from `jetbot_governor`) / `cmd_vel_nav` (traditional, from `jetbot_nav`'s Nav2 stack) `joy_controller`'s current mode selects.

**Subscribes**
| Topic | Type | Notes |
|---|---|---|
| `joy_controller/mode` | `std_msgs/String` | Selects which of the three sources below is currently followed. |
| `cmd_vel_joy` | `geometry_msgs/Twist` | Used when mode is `manual`. |
| `cmd_vel_final` | `geometry_msgs/Twist` | Used when mode is `vla`. `jetbot_governor`'s safety-checked VLA output. |
| `cmd_vel_nav` | `geometry_msgs/Twist` | Used when mode is `traditional`. `jetbot_nav`'s Nav2 controller output. |

**Fail-safe behavior — two distinct cases, deliberately different from each other:**
1. **The selected source goes stale** (no message within `command_timeout`) → stop. Same role `twist_mux`'s per-topic timeouts used to play.
2. **`joy_controller/mode` itself goes stale** (joystick unplugged, `joy_controller` crashed) → **stop unconditionally, regardless of which mode was last active.** This is a deliberate design choice, not just a mechanical port of the old behavior: losing the joystick means losing the human's ability to instantly retake control, so autonomous driving must not continue on the theory that "the VLA/planner data still looks fine." Confirmed by test: killing the `/joy` feed while in `traditional` mode with `cmd_vel_nav` actively publishing still stops the robot, purely because the mode signal itself went quiet.

**Publishes**
| Topic | Type | Notes |
|---|---|---|
| `odom` | `nav_msgs/Odometry` | Open-loop dead-reckoning (see caveat below), 20Hz default. |
| `odom` → `base_footprint` TF | — | Only if `publish_tf:=true`. In `bringup.launch.py` this is set `false` — `robot_localization`'s EKF owns that TF edge instead (see below); only one node should ever broadcast a given transform. Note: `motor_driver` creates its `TransformBroadcaster` unconditionally, so `ros2 node info /jetbot_motor_driver` still lists `/tf` as a publisher endpoint even with `publish_tf:=false` — check the live `publish_tf` param (`ros2 param get`), not the publisher list, to see who's actually sending. |

**Parameters** — `wheel_base`/`max_linear_vel`/`max_angular_vel` come from the shared `config/robot_params.yaml`; the rest are `motor_driver`-specific
| Name | Default | Source | Meaning |
|---|---|---|---|
| `wheel_base` | `0.102` | `robot_params.yaml` | meters, used for differential-drive kinematics |
| `max_linear_vel` | `0.2` | `robot_params.yaml` | m/s, also the clamp ceiling on incoming commands |
| `max_angular_vel` | `1.0` | `robot_params.yaml` | rad/s, clamp ceiling |
| `use_mock` | `true` | — | `false` tries `WaveshareMotorInterface` (real hardware), falling back to mock on any failure |
| `command_timeout` | `0.5` | — | seconds; applies to both fail-safe cases above |
| `odom_rate_hz` | `20.0` | — | odometry publish rate |
| `control_rate_hz` | `20.0` | — | arbitration/motor-command rate |
| `publish_tf` | `true` | — | set `false` when `robot_localization`'s EKF is fusing this `/odom` and should own `odom→base_footprint` instead |

**⚠️ Odometry accuracy caveat**: the Waveshare kit's TT motors have no encoders. `/odom` is computed by integrating *commanded* velocity, not measured wheel motion — it will drift steadily (no correction for wheel slip, PWM nonlinearity, or anything else) and is published with large, fixed covariance for exactly that reason. Treat it as a weak prior, not ground truth.

## `config/robot_params.yaml` — the shared source of truth

`wheel_base`, `max_linear_vel`, `max_angular_vel`, and `footprint` used to be separately hand-copied into `motor_driver`'s defaults, `joy_controller`'s scale factors, and `jetbot_nav`'s Nav2 config — and they'd drifted: this file's `wheel_base` used to be `0.14` while the URDF's actual wheel spacing works out to `0.102`, a mismatch nobody had caught. Now there's one file, loaded via ROS2's `/**:` wildcard syntax so any node that declares a matching parameter name picks it up automatically — `bringup.launch.py` loads it for both `motor_driver` and `joy_controller`, and `jetbot_nav`'s `nav.launch.py` merges it into Nav2's config at launch time (see that package's README).

**`wheel_base` and `footprint` should match `jetbot_description`'s URDF** — kept manually in sync, not auto-derived (parsing the URDF at launch time to compute these would need real URDF-introspection tooling, more than this project currently has). If you change the robot's physical dimensions, update both the URDF and this file together.

## `config/ekf.yaml` — `robot_localization`

`bringup.launch.py` runs `robot_localization`'s `ekf_node` to fuse `motor_driver`'s continuous-but-drifty `/odom` into `/odometry/filtered`, and it — not `motor_driver` — owns the `odom → base_footprint` TF (see `publish_tf` above). `jetbot_slam`'s RTAB-Map is unaffected by this and continues to own `map → odom` TF unchanged; it reads `odom → base_footprint` via TF regardless of which node is currently publishing it. Today there's only one input source (wheel odom); this is the place to add an IMU or real encoders later without any downstream node needing to change.

## `motor_interface.py`

Hardware abstraction layer, not a node — imported by `motor_driver`.
- `MockMotorInterface`: in-memory, for development without hardware.
- `WaveshareMotorInterface`: wraps `Adafruit_MotorHAT` (I2C). Raises `ImportError` if the library isn't installed, which `motor_driver` catches and falls back to mock.

## `launch/bringup.launch.py`

Master launch file for the core control loop (`joy_node`, `joy_controller`, `motor_driver`, EKF, governor, VLA bridge, camera). Does **not** include `jetbot_nav`'s `nav.launch.py` — without that running alongside, selecting `traditional` mode has no `cmd_vel_nav` publisher, and `motor_driver` fails safe (stops) rather than silently doing nothing. See the top-level repo README for usage.

`joy_node` needs a real joystick device (`/dev/input/jsX`) and will log errors without one attached. To exercise the mode-cycling/arbitration logic without physical hardware, publish synthetic `sensor_msgs/Joy` messages directly instead of running `joy_node`, e.g.:

```bash
ros2 topic pub /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.8, 0,0,0,0,0,0], buttons: [0,0,0,0,0,0,0,1,0,0,0]}" -r 20
```

## Known gaps

- No wheel encoders means no real odometry; see the caveat above.
- Button/axis indices in `config/joy_controller.yaml` are not verified against a physical Xbox controller (none available in this dev environment) — confirm with `ros2 topic echo /joy` before trusting them.
- `predictive_governor`'s veto is still a fixed LiDAR-distance threshold, not the "intelligent decision between traditional and VLA" the project's longer-term plan describes — see `brain_research_report.md` at the repo root.
