# jetbot_vision

Camera capture and on-device vision for the JetBot.

## Nodes

### `camera_node`

Captures from a real V4L2 camera (CSI or USB) via OpenCV and publishes it as a ROS2 `Image`, plus `CameraInfo` if a calibration is available.

**Publishes**
| Topic | Type | Notes |
|---|---|---|
| `camera/image_raw` | `sensor_msgs/Image` | `bgr8` encoding, `frame_id: camera_link` |
| `camera/camera_info` | `sensor_msgs/CameraInfo` | Only if `camera_info_manager` is installed **and** `camera_info_url` points to a valid calibration; otherwise this topic is silently not published (a clear error is logged instead — geometric vision like visual odometry/SLAM needs real intrinsics, so we don't fabricate defaults). |

**Parameters**
| Name | Default | Meaning |
|---|---|---|
| `video_device` | `/dev/video0` | |
| `width` / `height` / `fps` | `640` / `480` / `30` | |
| `camera_info_url` | `''` | e.g. `file:///path/to/jetbot_camera.yaml`, standard `camera_calibration`-format YAML. Run `ros2 run camera_calibration cameracalibrator` to generate one. |

Raises at startup if the device can't be opened; if the device opens but frames stop arriving later (e.g. a flaky USB connection), it logs a warning per dropped frame rather than crashing.

### `mock_camera_publisher`

Synthetic camera for testing without hardware — publishes a static checkerboard pattern (not random noise: feature-based pipelines like visual odometry need trackable structure to exercise their code at all) plus a **fabricated** `CameraInfo` good enough to keep a downstream pipeline from dividing by zero, explicitly not a substitute for real calibration. A static image can only prove a vision pipeline runs cleanly, not that it tracks real motion — there is no camera actually moving through space in mock mode.

**Parameters**: `width` (640), `height` (480), `fps` (10.0).

### `yolo_detector`

TensorRT-accelerated YOLOv8 object detector with a simple proportional-steering behavior (turns toward a detected target class).

**⚠️ Not currently wired into the rest of the graph.** It publishes steering commands to plain `cmd_vel`, which nothing subscribes to in the current `twist_mux` setup (see `jetbot_base`'s README) — running it does not move the robot. It also needs `ultralytics` installed and a real exported TensorRT engine file (`model_path`, default `yolov8n.engine`) — neither ships in this repo; run `jetbot_vision/setup_vision_env.sh` on the Jetson to set both up. If the model or library is missing, it logs an error and stays inert rather than crashing.

**Parameters**: `model_path` (`yolov8n.engine`), `conf_threshold` (`0.5`), `target_class` (`'person'`), `use_tensorrt` (`true`).

## `setup_vision_env.sh`

Jetson-only setup script (checks for `/etc/nv_tegra_release`) that installs `ultralytics` and checks for TensorRT, then prints the command to export a YOLOv8 model to a TensorRT engine.
