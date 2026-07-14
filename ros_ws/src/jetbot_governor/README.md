# jetbot_governor

The safety layer between the VLA bridge and the rest of the control stack — vetoes proposed forward motion when LiDAR says it's unsafe. This is the node the project's planning docs call "the WAM layer" / "Safety Bubble", though today it's a hardcoded distance threshold, not a learned model (see `brain_research_report.md` at the repo root for research on whether/how to change that).

## Nodes

### `predictive_governor`

**Subscribes**
| Topic | Type | Notes |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Only the front arc (`front_arc_deg`) is checked, not the full 360° — an obstacle behind the robot doesn't block forward motion. |
| `/cmd_vel_vla` | `geometry_msgs/Twist` | Raw, unvalidated proposal from `jetbot_vla_bridge`. |

**Publishes**
| Topic | Type | Notes |
|---|---|---|
| `/cmd_vel_final` | `geometry_msgs/Twist` | Safety-checked result. Feeds `jetbot_nav`'s `mode_arbiter` as the "VLA" option (the operator/joystick selects whether this or the traditional Nav2 planner actually reaches `twist_mux` — see `jetbot_nav`'s README) — this node does **not** talk to the motors directly. |

**Parameters**
| Name | Default | Meaning |
|---|---|---|
| `safety_threshold` | `0.4` (m) | Veto forward motion if anything in the front arc is closer than this |
| `front_arc_deg` | `60.0` | ± degrees ahead that count as "front" |
| `scan_timeout` | `1.0` (s) | If no `/scan` message arrives within this window, fail safe (veto everything) rather than assume clear |

**Fail-safe behavior**: no LiDAR data yet, stale LiDAR data, or an all-invalid scan all veto to a zero `Twist` rather than let the robot move blindly. LaserScan `inf` readings are treated as "clear to `range_max`" per REP-117; `nan` readings are skipped as sensor noise.

### `mock_lidar_publisher`

Publishes a synthetic `/scan` where every point is `front_distance` meters away, for testing the governor without real LiDAR hardware. Change the distance live to simulate an approaching wall:

```bash
ros2 param set /mock_lidar_publisher front_distance 0.2
```

**Parameters**: `front_distance` (`5.0`), `num_points` (`360`), `rate_hz` (`10.0`).

## Testing

`test/test_predictive_governor.py` unit-tests the front-arc distance logic directly (no rclpy node spin-up needed) — covers the wall-ahead case, the "obstacle behind is ignored" case, and empty/`nan`/`inf` scan edge cases. Run with `colcon test --packages-select jetbot_governor`.
