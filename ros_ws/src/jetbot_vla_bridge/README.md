# jetbot_vla_bridge

The "System 2" client: sends camera frames + a task string to a remote VLA server and turns its response into velocity proposals. Its output is unvalidated — `jetbot_governor` safety-checks it before it reaches `twist_mux`.

## Nodes

### `vla_client_bridge`

**Subscribes**
| Topic | Type |
|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` |

**Publishes**
| Topic | Type | Notes |
|---|---|---|
| `/cmd_vel_vla` | `geometry_msgs/Twist` | Raw proposal — not safety-checked, not arbitrated. |

**Parameters**
| Name | Default | Meaning |
|---|---|---|
| `server_url` | `http://vla-server:8000/predict` | VLA server endpoint. See `simulation/mock_vla_server.py` for the expected request/response shape. |
| `inference_rate_hz` | `1.0` | How often a real inference request is sent — VLA servers are slow, so this is deliberately decoupled from the publish rate below. |
| `task` | `'Navigate to target and stop'` | Text sent with every request. |
| `request_timeout` | `2.0` (s) | HTTP request timeout — protects against a hung server blocking the node. |
| `publish_rate_hz` | `10.0` | How often the *last known* action is republished to `/cmd_vel_vla`, independent of the (slow) inference cadence. |
| `command_max_age` | `3.0` (s) | If the last successful inference is older than this, publish zero instead of a stale command. |

### Why two rates?

Early testing found that publishing only on each inference response (≈1Hz by default) starved `motor_driver`'s 0.5s heartbeat watchdog — the robot would stutter to a stop between every VLA cycle even under perfect network conditions. `publish_rate_hz` decouples "how often the VLA re-plans" from "how often we emit a command"; `command_max_age` is what actually enforces "stop if the VLA has gone quiet," not the publish timer itself.

### Action token mapping

`map_tokens_to_twist` currently understands: `move_fwd`, `move_back`, `turn_l`, `turn_r`, `stop`. This is a placeholder scheme matched to `simulation/mock_vla_server.py`'s random token generator — swapping in a real VLA (see `brain_research_report.md`) will very likely need continuous waypoint/velocity outputs instead of discrete tokens, not just a bigger token vocabulary.
