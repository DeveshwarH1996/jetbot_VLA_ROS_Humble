# jetbot_base

Motor control, odometry, and teleop for the Waveshare JetBot AI Kit.

## Nodes

### `motor_driver`

Consumes the arbitrated velocity command and drives the physical (or mock) motors. Implements a heartbeat watchdog (stops the motors if commands go quiet) and publishes open-loop odometry.

**Subscribes**
| Topic | Type | Notes |
|---|---|---|
| `cmd_vel_mux` | `geometry_msgs/Twist` | `twist_mux`'s arbitrated output — **not** a raw command source. Never publish directly here from a new node; add it as a `twist_mux` input instead (see `config/twist_mux.yaml`). |

**Publishes**
| Topic | Type | Notes |
|---|---|---|
| `odom` | `nav_msgs/Odometry` | Open-loop dead-reckoning (see caveat below), 20Hz default. |
| `odom` → `base_footprint` TF | — | Only if `publish_tf:=true`. In `bringup.launch.py` this is set `false` — `robot_localization`'s EKF owns that TF edge instead (see below); only one node should ever broadcast a given transform. |

**Parameters**
| Name | Default | Meaning |
|---|---|---|
| `wheel_base` | `0.14` | meters, used for differential-drive kinematics |
| `max_linear_vel` | `0.2` | m/s, also the clamp ceiling on incoming commands |
| `max_angular_vel` | `1.0` | rad/s, clamp ceiling |
| `use_mock` | `true` | `false` tries `WaveshareMotorInterface` (real hardware), falling back to mock on any failure |
| `heartbeat_timeout` | `0.5` | seconds; motors stop if no command arrives within this window |
| `odom_rate_hz` | `20.0` | odometry publish rate |
| `publish_tf` | `true` | set `false` when `robot_localization`'s EKF is fusing this `/odom` and should own `odom→base_footprint` instead |

**⚠️ Odometry accuracy caveat**: the Waveshare kit's TT motors have no encoders. `/odom` is computed by integrating *commanded* velocity, not measured wheel motion — it will drift steadily (no correction for wheel slip, PWM nonlinearity, or anything else) and is published with large, fixed covariance for exactly that reason. Treat it as a weak prior, not ground truth.

## `config/ekf.yaml` — `robot_localization`

`bringup.launch.py` runs `robot_localization`'s `ekf_node` to fuse `motor_driver`'s continuous-but-drifty `/odom` into `/odometry/filtered`, and it — not `motor_driver` — owns the `odom → base_footprint` TF (see `publish_tf` above). `jetbot_slam`'s RTAB-Map is unaffected by this and continues to own `map → odom` TF unchanged; it reads `odom → base_footprint` via TF regardless of which node is currently publishing it. Today there's only one input source (wheel odom); this is the place to add an IMU or real encoders later without any downstream node needing to change.

### `teleop`

**Not real teleop.** This is a fixed timer that publishes one hardcoded forward-motion command every 2 seconds, for exercising `/cmd_vel` in isolation. It also publishes to plain `cmd_vel`, which nothing downstream currently subscribes to (see the module-level wiring note below) — running it does not move the robot through the current bringup graph.

For actual manual driving, use the stock `teleop_twist_keyboard` or `teleop_twist_joy` packages (already installed as ROS2 dependencies) remapped to `cmd_vel_joy`, which *is* a real `twist_mux` input:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_joy
```

## `motor_interface.py`

Hardware abstraction layer, not a node — imported by `motor_driver`.
- `MockMotorInterface`: in-memory, for development without hardware.
- `WaveshareMotorInterface`: wraps `Adafruit_MotorHAT` (I2C). Raises `ImportError` if the library isn't installed, which `motor_driver` catches and falls back to mock.

## `config/twist_mux.yaml`

Priority arbitration for `twist_mux` (external package, `ros-humble-twist-mux`). **Note the priority convention**: twist_mux uses "higher number wins" — the opposite of the "lower number = higher priority" phrasing in the project's early planning docs (`launch_guide.md`). Current priorities: `safety` (90, reserved — nothing publishes to `cmd_vel_safety` yet) > `joystick` (50) > `autonomous` (10, `cmd_vel_autonomous`). Output topic is remapped to `cmd_vel_mux` in `launch/bringup.launch.py`.

`twist_mux` only ever sees **one** autonomous source, never two — `jetbot_nav`'s `mode_arbiter` picks exactly one of {traditional Nav2 planner, VLA} and republishes only that one to `cmd_vel_autonomous`. This is deliberate: `twist_mux`'s own "lock" mechanism blocks everything at or below a priority level (an e-stop primitive), not a specific channel, so it can't cleanly express "exactly one of these two peers is active" — see `jetbot_nav`'s README for the full reasoning. The joystick still always overrides whichever autonomous mode is selected, unchanged from before.

## `launch/bringup.launch.py`

Master launch file for the core control loop (motor driver, EKF, twist_mux, governor, VLA bridge, camera). Does **not** include `jetbot_nav`'s `nav.launch.py` — without that running alongside, `cmd_vel_autonomous` has no publisher and the robot only responds to the joystick. See the top-level repo README for usage.

## Known gaps

- No wheel encoders means no real odometry; see the caveat above.
- `teleop` (this package) is still not real teleop — a fixed timer publishing one hardcoded command to plain `cmd_vel`, which isn't a `twist_mux` input. Use `teleop_twist_keyboard`/`teleop_twist_joy` instead (see above).
