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
| `odom` → `base_footprint` TF | — | Broadcast alongside the odometry message. |

**Parameters**
| Name | Default | Meaning |
|---|---|---|
| `wheel_base` | `0.14` | meters, used for differential-drive kinematics |
| `max_linear_vel` | `0.2` | m/s, also the clamp ceiling on incoming commands |
| `max_angular_vel` | `1.0` | rad/s, clamp ceiling |
| `use_mock` | `true` | `false` tries `WaveshareMotorInterface` (real hardware), falling back to mock on any failure |
| `heartbeat_timeout` | `0.5` | seconds; motors stop if no command arrives within this window |
| `odom_rate_hz` | `20.0` | odometry publish rate |

**⚠️ Odometry accuracy caveat**: the Waveshare kit's TT motors have no encoders. `/odom` is computed by integrating *commanded* velocity, not measured wheel motion — it will drift steadily (no correction for wheel slip, PWM nonlinearity, or anything else) and is published with large, fixed covariance for exactly that reason. Treat it as a weak prior for `jetbot_slam`'s loop closure, not ground truth. If a real Nav2 costmap needs to trust dead-reckoning between corrections, this is the first thing to upgrade (wheel encoders or an IMU).

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

Priority arbitration for `twist_mux` (external package, `ros-humble-twist-mux`). **Note the priority convention**: twist_mux uses "higher number wins" — the opposite of the "lower number = higher priority" phrasing in the project's early planning docs (`launch_guide.md`). Current priorities: `safety` (90, reserved — nothing publishes to `cmd_vel_safety` yet) > `joystick` (50) > `vla` (10, fed by `jetbot_governor`'s `cmd_vel_final`). Output topic is remapped to `cmd_vel_mux` in `launch/bringup.launch.py`.

## `launch/bringup.launch.py`

Master launch file for the full pipeline (motor driver, twist_mux, governor, VLA bridge, camera). See the top-level repo README for usage.

## Known gaps

- `yolo_detector` (in `jetbot_vision`) also publishes to plain `cmd_vel`, same as `teleop` — neither is wired into `twist_mux`. If you want either to actually drive the robot, add a `twist_mux` input for it and remap.
- No wheel encoders means no real odometry; see the caveat above.
