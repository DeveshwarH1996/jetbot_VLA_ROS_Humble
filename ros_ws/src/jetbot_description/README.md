# jetbot_description

URDF and RViz visualization for the JetBot. No custom nodes — `ament_cmake` package providing `urdf/`, `launch/`, and `rviz/` files for `robot_state_publisher`/`joint_state_publisher`/`rviz2`.

## `urdf/jetbot.urdf`

Simple-geometry model (boxes/cylinders, no meshes): `base_footprint` → `chassis` → `left_wheel` / `right_wheel` / `camera_link` → `camera_optical_frame`. Dimensions are illustrative placeholders (10cm chassis box, 3cm wheel radius), not measured from a real kit — if you have real dimensions, update this before trusting any Nav2 footprint config derived from it (`jetbot_nav`'s `nav2_params.yaml` footprint is currently matched to these placeholders).

`camera_optical_frame` is a fixed child of `camera_link` applying the standard REP103→optical-convention rotation (`camera_link` is X-forward/Y-left/Z-up like the rest of the robot; image/pixel math needs X-right/Y-down/Z-forward-into-scene). `jetbot_vision`'s camera nodes stamp images with this frame, and `jetbot_nav`'s `ground_plane_projector` looks the transform up via TF rather than hardcoding the camera's mount angle — if the camera is ever remounted, only this URDF needs to change.

## `launch/display.launch.py`

```bash
ros2 launch jetbot_description display.launch.py
```

Starts `robot_state_publisher` (loads the URDF), `joint_state_publisher` (fake joint states — no real encoders to read), and `rviz2` with `rviz/jetbot.rviz`.

Note: earlier versions of this launch file started the first two nodes but never actually launched RViz or configured any displays — it published TF/joint-state data but didn't "display" anything. Fixed and verified against a real render (RobotModel + TF displays, `Global Status: Ok`).

## `rviz/jetbot.rviz`

Fixed Frame `base_footprint`, `RobotModel` + `TF` displays enabled. TF axis marker scale is set to `0.05` — the RViz default is sized for typical robots and completely dwarfs this JetBot's ~10cm chassis at default scale.
