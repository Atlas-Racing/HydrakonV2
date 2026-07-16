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

## Greenwave Monitor: rich TUI (r2s_gw)

`greenwave_monitor`'s ncurses dashboard works out of the box, but the richer Textual-based TUI, `r2s_gw` (vendored into `src/r2s_gw` from [NVIDIA-ISAAC-ROS/r2s_gw](https://github.com/NVIDIA-ISAAC-ROS/r2s_gw)), needs `numpy>=2.0.1` — incompatible with the `numpy 1.21.5` this workspace's ROS packages (`cv_bridge`, etc.) are built against. Installing it with plain `pip install --user` **breaks `cv_bridge` system-wide** (confirmed: `numpy.ndarray size changed` / `_ARRAY_API not found` errors) since user-site packages shadow the apt-installed numpy for every Python process, not just the TUI.

`src/r2s_gw` therefore carries a `COLCON_IGNORE` marker — a plain `colcon build` skips it — and it's built separately into its own virtualenv so its dependencies never leak into the rest of the workspace:

```bash
cd ~/HydrakonV2
python3 -m venv --system-site-packages .venvs/r2s_gw
.venvs/r2s_gw/bin/pip install --ignore-installed pygments -r src/r2s_gw/requirements.txt

# Build r2s_gw specifically with the venv active, so its console-script
# entry point's shebang points at the venv's python (plain colcon build
# would otherwise regenerate it pointing at system python).
rm -f src/r2s_gw/COLCON_IGNORE
source .venvs/r2s_gw/bin/activate
python3 -m colcon build --symlink-install --packages-select r2s_gw
deactivate
touch src/r2s_gw/COLCON_IGNORE
```

`--system-site-packages` lets the venv still see `rclpy` and other ROS Python bindings from `/opt/ros/humble`; only `r2s_gw`'s own deps (`numpy`, `textual`, `rich`, ...) are isolated to the venv. Once built, `source install/local_setup.bash` as usual, then:
```bash
ros2 run r2s_gw r2s_gw
```
Rebuilding `r2s_gw` after a source change needs the same venv-activated `python3 -m colcon build --packages-select r2s_gw` invocation, not a plain `colcon build`.

## Network: remote topic access via the Deeper Connect Mini

The Jetson's `eno1` port feeds the Deeper Connect Mini and shares the Jetson's WiFi internet connection to it (NetworkManager `ipv4.method shared` on the `eno1` connection profile). The Jetson is `10.42.0.1` on that link; the Deeper Connect Mini DHCPs a WAN address from it and broadcasts its own WiFi network with internet passthrough. The Deeper Connect Mini's admin page is reachable from the Jetson at its current DHCP lease (check `ip neigh show dev eno1`).

Any laptop joining the Deeper Connect Mini's WiFi to see the Jetson's ROS 2 topics must be on the same ROS domain:
```bash
export ROS_DOMAIN_ID=0
```

## Launching the stack

**Camera + object detection** (ZED X, cone detection, marker publishing):
```bash
ros2 launch hydrakon_bringup hydrakon_bringup.py
```
This includes `zedx_bringup.py` (camera + custom object detection), starts `cone_marker_publisher`, and starts `greenwave_monitor` (vendored from [NVIDIA-ISAAC-ROS/greenwave_monitor](https://github.com/NVIDIA-ISAAC-ROS/greenwave_monitor) into `src/greenwave_monitor`) for topic rate/latency diagnostics — a faster, always-on alternative to `ros2 topic hz`.

`greenwave_monitor` itself only tracks an explicit topic list (empty by default), so `hydrakon_bringup` also starts `greenwave_auto_discovery` (`src/hydrakon_bringup/hydrakon_bringup/greenwave_auto_discovery.py`), which polls the ROS graph every 2s and registers every topic it finds (except `/rosout`, `/parameter_events`, `/diagnostics`) via `greenwave_monitor`'s `manage_topic` service. **By default every topic is monitored**, including ones like `/zed/zed_node/obj_det/objects` that only appear once object detection finishes starting up. To restrict monitoring to specific topics instead, turn auto-discovery off and pass your own list:
```bash
ros2 launch hydrakon_bringup hydrakon_bringup.py gw_monitor_all_topics:=false gw_monitored_topics:='["/zed/zed_node/obj_det/objects", "/cone_markers"]'
```
View live rates/latency/status with `ros2 run greenwave_monitor ncurses_dashboard` in another terminal, or with the richer Textual-based TUI, `ros2 run r2s_gw r2s_gw` (see "Greenwave Monitor: rich TUI (r2s_gw)" below for one-time setup).

On the very first run after a fresh model/engine cache, the ZED SDK auto-converts the ONNX model to a TensorRT engine — this is a one-time step and can take **10-20 minutes** at 1280x1280 on Jetson (watch for `Optimizing model: best Progress: X%` in the log). It's cached after that; subsequent launches start immediately.

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

**Skidpad controller** (separate terminal, camera + object detection already running):
```bash
ros2 launch hydrakon_bringup skidpad_controller_launch.py
```
- Dedicated phase-state-machine controller (WARMUP → ENTRY → RIGHT → LEFT → EXIT → STOPPING → DONE), not the generic corridor follower — see below.
- `active_ami_states` (default `SKIDPAD`): same gating pattern as the corridor controller.

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
You can add `skidpad_controller_launch.py` as an additional terminal alongside terminal 3 rather than swapping it in — both self-gate on `/hydrakon_can/state_str` and only one ever actually publishes at a time, since their default `active_ami_states` (`TRACKDRIVE,AUTOCROSS` vs `SKIDPAD`) don't overlap. (This is unlike `corridor_controller_2d` vs `_3d` above, which share the same default gate and *would* genuinely fight if both were launched.)
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
- Not implemented: lap counting, mission completion/stop logic, closed-loop speed control (acceleration is one of 4 fixed tiers, not a target-speed loop), fault handling on sustained perception loss (currently just decays to zero rather than requesting EBS).

## Planning: skidpad controller

- `hydrakon_planner/skidpad_controller.py`, entry point `skidpad_controller` — ported from last year's CarMaker-only `skidpad_node.py` onto the same real-vehicle interfaces the corridor controllers use: `zed_msgs/msg/ObjectsStamped` for cones, `ackermann_msgs/msg/AckermannDriveStamped` on `/hydrakon_can/command` for output, `/hydrakon_can/state_str` + `active_ami_states` (default `SKIDPAD`) for AMI gating, and `/hydrakon_can/imu` + `/hydrakon_can/wheel_speed` in place of the old `/carmaker/*` topics.
- Unlike the corridor controllers, this is a full phase state machine (`WARMUP → ENTRY → RIGHT → LEFT → EXIT → STOPPING → DONE`) driven by a fixed 20 Hz timer (not the object-detection callback), with closed-loop speed PID, an angle-aware opposite-circle cone mask, an inner-cone safety carve-out, and potential-field obstacle avoidance. See the module/class docstrings in `skidpad_controller.py` for the full v1/v2 failure-mode history behind those design choices.
- `drive.acceleration` here is a single signed PID output (clipped to `[-max_brake, max_gas]`), fed straight through to `hydrakon_can`'s accel/brake interpretation — not the corridor controllers' 4 fixed accel tiers.
- `STOPPING` (`stop_delay_sec`, default `1.0`s) keeps the car driving straight for that long after clearing the exit gate before `DONE` actually brakes — same shape as `hydrakon_can.cpp`'s `handleStaticInspectionA` (timed stage after the last driving action). On entering `DONE`, it publishes `std_msgs/msg/Bool(true)` on `/hydrakon_can/is_mission_completed` every tick, the same topic `hydrakon_can.cpp`'s `flagCallback` already consumes to set `mission_complete_` (reported to the VCU as `MISSION_FINISHED`).
- Also not tuned on real hardware yet (inherited from the CarMaker-era constants) — same caveat as the corridor controllers above.

## Not yet implemented

- Lap counting, mission completion, and closed-loop speed control in the corridor (Trackdrive/Autocross) planner (see above) — the skidpad controller does have closed-loop speed control and its own phase/completion logic.
- Acceleration event controller.
- A single top-level launch combining `hydrakon_bringup` + `hydrakon_can` + `hydrakon_planner`.
- Automated tests for the planner's steering/accel geometry (currently only ament boilerplate — copyright/flake8/pep257 — no logic tests).
