# HydrakonV2
Second version of the code deployed on the ADS-DV for FS-AI 2026

## Workspaces

This repo (`HydrakonV2`) holds `hydrakon_bringup`, `hydrakon_can`, and `hydrakon_perception`.

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

There is no single top-level launch file combining bringup + CAN yet — run them in separate terminals.

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

## Not yet implemented

- Planning: no package exists yet.
- A single top-level launch combining `hydrakon_bringup` + `hydrakon_can`.
