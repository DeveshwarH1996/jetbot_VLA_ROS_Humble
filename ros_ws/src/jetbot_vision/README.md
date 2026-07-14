# jetbot_vision

Camera capture and on-device vision for the JetBot.

## Nodes

### `camera_node`

Captures from a real V4L2 camera (CSI or USB) via OpenCV and publishes it as a ROS2 `Image`, plus `CameraInfo` if a calibration is available.

**Publishes**
| Topic | Type | Notes |
|---|---|---|
| `camera/image_raw` | `sensor_msgs/Image` | `bgr8` encoding, `frame_id: camera_optical_frame` (the standard optical-convention frame added in `jetbot_description`'s URDF — not `camera_link`, which follows REP103 axes instead; see `jetbot_nav`'s README for why the distinction matters) |
| `camera/camera_info` | `sensor_msgs/CameraInfo` | Only if `camera_info_manager` is installed **and** `camera_info_url` points to a valid calibration; otherwise this topic is silently not published (a clear error is logged instead — geometric vision like visual odometry/SLAM needs real intrinsics, so we don't fabricate defaults). |

**Parameters**
| Name | Default | Meaning |
|---|---|---|
| `video_device` | `/dev/video0` | |
| `width` / `height` / `fps` | `640` / `480` / `30` | |
| `camera_info_url` | `''` | e.g. `file:///path/to/jetbot_camera.yaml`, standard `camera_calibration`-format YAML. Run `ros2 run camera_calibration cameracalibrator` to generate one. |

Raises at startup if the device can't be opened; if the device opens but frames stop arriving later (e.g. a flaky USB connection), it logs a warning per dropped frame rather than crashing.

## Calibrating a real camera

`camera_info_url` is how per-robot camera intrinsics get in — `jetbot_nav`'s `ground_plane_projector` reads the real `fx`/`fy`/`cx`/`cy` from whatever `CameraInfo` this publishes, not a hardcoded guess. But nobody has calibrated the actual JetBot camera yet, so this defaults to unset, and `camera_node` publishes an **uncalibrated** `CameraInfo` (all-zero intrinsics) with a warning in that case. `ground_plane_projector` detects this specifically (`fx`/`fy` ≤ 0) and refuses to project rather than crashing or silently computing garbage positions.

To fix it for real:
1. Install the calibration tool: `sudo apt-get install ros-humble-camera-calibration`.
2. Print a checkerboard target, then run (adjust `--size`/`--square` for your actual board):
   ```bash
   ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.024 \
       --ros-args -r image:=/camera/image_raw
   ```
3. Move the board through the frame until all four bars in the calibration GUI turn green, click **CALIBRATE**, then **SAVE** — it writes a YAML in the standard format to `/tmp`.
4. Point `camera_node` at it: `-p camera_info_url:=file:///path/to/your_calibration.yaml`.

`config/camera_calibration.example.yaml` in this package shows the expected file format with clearly-labeled placeholder numbers — **do not use it as a real calibration**, a plausible-looking wrong intrinsic is more dangerous than the uncalibrated case, since it would silently produce wrong obstacle positions instead of failing loudly.

### `mock_camera_publisher`

Synthetic camera for testing without hardware — publishes a static checkerboard pattern (not random noise: feature-based pipelines like visual odometry need trackable structure to exercise their code at all) plus a **fabricated** `CameraInfo` good enough to keep a downstream pipeline from dividing by zero, explicitly not a substitute for real calibration. A static image can only prove a vision pipeline runs cleanly, not that it tracks real motion — there is no camera actually moving through space in mock mode.

**Parameters**: `width` (640), `height` (480), `fps` (10.0).

### `yolo_detector`

TensorRT-accelerated YOLOv8 object detector. **Publishes detections only — it does not drive the robot.** An earlier version computed proportional steering directly from bounding-box position and published straight to `cmd_vel`, bypassing any obstacle/path safety reasoning entirely; that direct-to-motor path has been removed. Detections now feed `jetbot_nav`'s `ground_plane_projector` → Nav2 costmap → planner/controller, so a real trajectory plan sits between "an object was seen" and "the wheels move." See `jetbot_nav`'s README for that pipeline.

**Publishes**: `detections` (`vision_msgs/Detection2DArray`).

**Parameters**: `model_path` (`yolov8n.engine`), `conf_threshold` (`0.5`), `target_class` (`''` — empty publishes all detected classes; set to e.g. `'person'` to filter to one).

Needs `ultralytics` installed and a real exported TensorRT engine file — neither ships in this repo; run `jetbot_vision/setup_vision_env.sh` on the Jetson to set both up. If the model or library is missing, it logs an error and stays inert rather than crashing. For testing without either, use `mock_detection_publisher` below.

### `mock_detection_publisher`

Publishes a synthetic `vision_msgs/Detection2DArray` (roughly a person-sized box, a couple meters ahead) so `jetbot_nav`'s ground-plane-projection → Nav2 costmap chain can be exercised without `ultralytics` or a real camera. Set `publish:=false` live to simulate a clear frame.

**Parameters**: `class_id` (`'person'`), `bbox_center_x`/`bbox_center_y` (`340.0`/`380.0`), `bbox_width`/`bbox_height` (`80.0`/`180.0`), `score` (`0.9`), `rate_hz` (`5.0`), `publish` (`true`).

## `setup_vision_env.sh`

Jetson-only setup script (checks for `/etc/nv_tegra_release`) that installs `ultralytics` and checks for TensorRT, then prints the command to export a YOLOv8 model to a TensorRT engine.
