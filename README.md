# HydrakonV2
Second version of the code deployed on the ADS-DV for FS-AI 2026

## Workspaces

This repo (`HydrakonV2`) holds `hydrakon_bringup`, `hydrakon_can`, `hydrakon_perception`, and `hydrakon_planner`.

The ZED SDK wrapper lives in a **separate workspace**, `~/zed_ws`, not in this repo:
- `zed-ros2-wrapper` — the camera driver (`zed_wrapper`, `zed_components`, `zed_msgs`, ...)
- `zed-ros2-examples` — RViz visualization for object detection (`zed_display_rviz2`, `rviz_plugin_zed_od`); only these two subpackages are built, the rest (`isaac_ros`, `tools`, `tutorials`, `examples`, `zed_display_foxglove`) are `COLCON_IGNORE`d

`~/.bashrc` already sources ROS Humble and `zed_ws` for every terminal. Each terminal working with this repo additionally needs:

```bash
source ~/HydrakonV2/install/local_setup.bash
```

## Building

```bash
cd ~/HydrakonV2
colcon build --symlink-install
```

## Launching the stack

**Camera + object detection** (ZED X, cone detection, marker publishing):
```bash
ros2 launch hydrakon_bringup hydrakon_bringup.py
```
This includes `zedx_bringup.py` (camera + custom object detection) and starts `cone_marker_publisher`. On the very first run after a fresh model/engine cache, the ZED SDK auto-converts the ONNX model to a TensorRT engine — this is a one-time step and can take **10-20 minutes** at 1280x1280 on Jetson (watch for `Optimizing model: best Progress: X%` in the log). It's cached after that; subsequent launches start immediately.

**CAN interface** (separate terminal):
```bash
ros2 launch hydrakon_can hydrakon_can_launch.py
```
Currently configured for real hardware (`simulate_can: 0`, `can_interface: can0`) — confirm the interface is up first: `ip link show can0`.

**RViz with native ZED object detection display** (separate terminal, camera already running elsewhere):
```bash
ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=zedx start_zed_node:=False
```

**Corridor controller — 2D pixel-space or 3D metric-position** (separate terminal, camera + object detection already running):
```bash
ros2 launch hydrakon_bringup corridor_controller_launch.py controller_mode:=2d   # or controller_mode:=3d
```
- `controller_mode` (default `2d`): `2d` runs `corridor_controller_2d` (pixel bounding-box steering), `3d` runs `corridor_controller_3d` (metric-position steering via `atan2`). They're mutually exclusive — both publish to `/hydrakon_can/command`, don't run both at once.
- `active_ami_states` (default `TRACKDRIVE,AUTOCROSS`): comma-separated AMI mission names the controller is gated on. It only publishes drive commands while `/hydrakon_can/state_str` reports one of these missions; otherwise it stays silent. Override for an isolated test session, e.g.:
  ```bash
  ros2 launch hydrakon_bringup corridor_controller_launch.py controller_mode:=3d active_ami_states:=SKIDPAD
  ```

There is no single top-level launch file combining bringup + CAN + planner yet — run them in separate terminals.

**Full stack, four terminals:**
```bash
# 1
ros2 launch hydrakon_bringup hydrakon_bringup.py
# 2
ros2 launch hydrakon_can hydrakon_can_launch.py
# 3
ros2 launch hydrakon_bringup corridor_controller_launch.py controller_mode:=2d
# 4 (optional, visualization)
ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=zedx start_zed_node:=False
```
Each terminal needs the sourcing from the top of this file (`ROS Humble` + `zed_ws` are already automatic via `.bashrc`; still run `source ~/HydrakonV2/install/local_setup.bash` in each).

## Perception: cone detection

- Model: custom-trained YOLOv26m, 1280x1280 input, from the `atlasracing-perception` training repo (`~/atlasracing-perception`), trained on the `FS_Cone` dataset.
- 5 classes (fixed order, matches `FS_Cone/yolo_dataset/data.yaml`): `unknown_cone`, `yellow_cone`, `blue_cone`, `orange_cone`, `large_orange_cone`.
- Model + config live inside `hydrakon_perception`:
  - `hydrakon_perception/models/best.onnx` — exported ONNX model
  - `config/cone_detection.yaml` — per-class ZED object detection config (labels, thresholds, tracking hints)
- Detection runs natively inside the ZED SDK (`OBJECT_DETECTION_MODEL::CUSTOM_YOLOLIKE_BOX_OBJECTS`, set via `hydrakon_bringup/launch/zedx_bringup.py`), not a separate inference node. The SDK publishes tracked 2D/3D detections on `/zed/zed_node/obj_det/objects` (`zed_msgs/msg/ObjectsStamped`).
- `cone_marker_publisher` (in `hydrakon_perception`) subscribes to that topic and republishes color-coded `visualization_msgs/msg/MarkerArray` on `cone_markers`, so detections are visible in plain RViz without the ZED-specific display plugin: yellow/blue/orange cylinders, larger dark-orange for `large_orange_cone`, gray for `unknown_cone`.
- Launch arg `cone_is_static` (default `true`) controls whether the SDK treats cones as stationary. `true` gives smoother, more stable tracking for a real static track, but heavily damps position updates — noticeable as latency if hand-testing by moving a cone around. Use `cone_is_static:=false` for that kind of testing:
  ```bash
  ros2 launch hydrakon_bringup hydrakon_bringup.py cone_is_static:=false
  ```

## Planning: corridor controllers (Trackdrive/Autocross)

- Package `hydrakon_planner` — ported from last year's `CombinedController`, fixed for the current detection pipeline (was reading `vision_msgs/Detection2DArray` at a stale 1280x720 resolution with a class-ID mapping that no longer matches `cone_detection.yaml`). Both nodes now read `zed_msgs/msg/ObjectsStamped` directly and match cones by `label` string.
- Two interchangeable implementations, same topics in/out:
  - `corridor_controller_2d` — finds the closest yellow/blue cone by pixel bounding box (native 1920x1200 ZED X resolution), steers toward the midpoint in pixel space.
  - `corridor_controller_3d` — finds the closest yellow/blue cone by real forward distance (`obj.position`), steers toward the metric midpoint via `atan2`. No pixel-space assumptions.
- Both subscribe to `/hydrakon_can/state_str` and `/zed/zed_node/obj_det/objects`, publish `ackermann_msgs/msg/AckermannDriveStamped` on `/hydrakon_can/command`, and only do so while gated on an active AMI mission (`active_ami_states` parameter, default `TRACKDRIVE,AUTOCROSS`) — required, since `hydrakon_can`'s EBS watchdog trips if `/hydrakon_can/command` goes silent for >0.5s while driving, so both controllers publish on every detection callback even with zero cones (decayed steering, zero acceleration).
- **Nothing in here is tuned.** `steering_gain`, separation thresholds, and curvature offsets are carried over from last year's constants with a first-order resolution rescale (1280→1920px) — real values need on-track testing. See code comments in `hydrakon_planner/corridor_controller_2d.py` / `_3d.py` for the exact rationale on each constant.
- Not implemented: lap counting, mission completion/stop logic, closed-loop speed control (acceleration is one of 4 fixed tiers, not a target-speed loop), fault handling on sustained perception loss (currently just decays to zero rather than requesting EBS), and Skidpad/Acceleration mission controllers.

## Not yet implemented

- Lap counting, mission completion, and closed-loop speed control in the planner (see above).
- Skidpad and Acceleration event controllers — only Trackdrive/Autocross are covered.
- A single top-level launch combining `hydrakon_bringup` + `hydrakon_can` + `hydrakon_planner`.
- Automated tests for the planner's steering/accel geometry (currently only ament boilerplate — copyright/flake8/pep257 — no logic tests).
