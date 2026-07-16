"""2D pixel-space cone-corridor controller (Trackdrive / Autocross).

Ported from last year's CombinedController. Fixed to read zed_msgs/ObjectsStamped
(bounding_box_2d) at the ZED X's native HD1200 (1920x1200) grab resolution instead
of vision_msgs/Detection2DArray at the old, stale 1280x720 assumption. Matches
cones by label string, not numeric class id (cone_detection.yaml's class order
does not match the old YELLOW=0/BLUE=1/... ids).
"""
import math
import re

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String
from zed_msgs.msg import ObjectsStamped

AMI_RE = re.compile(r'AMI:(\w+)')

# NOTE: last year's code hardcoded a single '== SKIDPAD' gate because AMI was
# manually faked to SKIDPAD during test sessions (regardless of actual mission)
# to avoid touching code. This boundary-following algorithm was actually written
# for Trackdrive/Autocross - gate on those AMI values for real. Override via the
# 'active_ami_states' parameter if a different mission set is needed.
DEFAULT_ACTIVE_AMI_STATES = ['TRACKDRIVE', 'AUTOCROSS']

TRACKING_STATE_OK = 1
TRACKING_STATE_SEARCHING = 2

# Native ZED X grab resolution (zedx.yaml: grab_resolution: 'HD1200'). bounding_box_2d
# pixel coords live in this space, NOT the downscaled published image resolution.
IMAGE_WIDTH_PX = 1920.0
IMAGE_CENTER_X_PX = IMAGE_WIDTH_PX / 2.0

# hydrakon_can.cpp truncates AI2VCU_STEER_ANGLE_REQUEST_deg to MAX_STEERING_ANGLE_DEG_
# (21.0, see hydrakon_can.hpp) regardless of what we send, so match that limit here
# rather than clipping to some other value at this layer.
MAX_STEERING_RAD = math.radians(21.0)


def _bbox_center_x(obj) -> float:
    # corners: [0]=top-left, [1]=top-right, [2]=bottom-right, [3]=bottom-left
    # (always axis-aligned per zed_msgs/BoundingBox2Di.msg) - diagonal midpoint
    # of opposite corners == centroid.
    corners = obj.bounding_box_2d.corners
    return (corners[0].kp[0] + corners[2].kp[0]) / 2.0


def _bbox_bottom_y(obj) -> float:
    corners = obj.bounding_box_2d.corners
    return max(corners[2].kp[1], corners[3].kp[1])


def _is_tracked(obj) -> bool:
    return obj.tracking_state in (TRACKING_STATE_OK, TRACKING_STATE_SEARCHING)


class CorridorController2D(Node):

    def __init__(self):
        super().__init__('corridor_controller_2d')

        self.declare_parameter('active_ami_states', DEFAULT_ACTIVE_AMI_STATES)
        # Old constant (0.003) was tuned for a 1280px-wide image; the same physical
        # lateral offset spans proportionally more pixels at 1920px, so this is
        # rescaled by 1280/1920 as a first-order correction only - the old value
        # came from a different camera/lens entirely and needed on-vehicle retuning.
        # On-vehicle testing showed steering barely turning for normal corridor
        # offsets, so bumped 3x from the rescaled-only value; keep tuning via the
        # 'steering_gain' parameter (ros2 param set) rather than editing this default.
        self.declare_parameter('steering_gain', 0.006 * (1280.0 / 1920.0) * 3.0)
        self.declare_parameter('midpoint_smoothing', 0.7)
        self.declare_parameter('steering_decay', 0.9)
        self.declare_parameter('separation_threshold_px', 50.0 * (1920.0 / 1280.0))
        self.declare_parameter('single_color_offset_px', 180.0 * (1920.0 / 1280.0))
        self.declare_parameter('accel_nominal', 0.6)
        self.declare_parameter('accel_ambiguous', 0.55)
        self.declare_parameter('accel_single_color', 0.3)

        self._active_ami_states = set(self.get_parameter('active_ami_states').value)
        self._steering_gain = self.get_parameter('steering_gain').value
        self._smoothing = self.get_parameter('midpoint_smoothing').value
        self._decay = self.get_parameter('steering_decay').value
        self._sep_threshold_px = self.get_parameter('separation_threshold_px').value
        self._single_offset_px = self.get_parameter('single_color_offset_px').value
        self._accel_nominal = self.get_parameter('accel_nominal').value
        self._accel_ambiguous = self.get_parameter('accel_ambiguous').value
        self._accel_single = self.get_parameter('accel_single_color').value

        self._ami_state = None
        self._last_steering = 0.0
        self._smoothed_x = IMAGE_CENTER_X_PX

        self._state_sub = self.create_subscription(
            String, '/hydrakon_can/state_str', self._on_state_str, 1)
        self._objects_sub = self.create_subscription(
            ObjectsStamped, '/zed/zed_node/obj_det/objects', self._on_objects, 10)
        self._cmd_pub = self.create_publisher(
            AckermannDriveStamped, '/hydrakon_can/command', 1)

        self.get_logger().info(
            f'corridor_controller_2d active for AMI states: {sorted(self._active_ami_states)}')

    def _on_state_str(self, msg: String) -> None:
        match = AMI_RE.search(msg.data)
        if match:
            self._ami_state = match.group(1)

    def _on_objects(self, msg: ObjectsStamped) -> None:
        if self._ami_state not in self._active_ami_states:
            return  # not gated for this mission; don't publish - avoid fighting other controllers

        steering, accel = self._compute_command(msg.objects)
        self._publish(msg.header, steering, accel)

    def _compute_command(self, objects):
        yellow = [o for o in objects if o.label == 'yellow_cone' and _is_tracked(o)]
        blue = [o for o in objects if o.label == 'blue_cone' and _is_tracked(o)]

        y = max(yellow, key=_bbox_bottom_y, default=None)
        b = max(blue, key=_bbox_bottom_y, default=None)

        if y is not None and b is not None:
            y_x, b_x = _bbox_center_x(y), _bbox_center_x(b)
            midpoint_x = (y_x + b_x) / 2.0
            separation = b_x - y_x
            accel = (self._accel_nominal if separation > self._sep_threshold_px
                     else self._accel_ambiguous)
        elif y is not None or b is not None:
            single = y if y is not None else b
            offset = self._single_offset_px if single is b else -self._single_offset_px
            midpoint_x = _bbox_center_x(single) + offset
            accel = self._accel_single
        else:
            self._last_steering *= self._decay
            return self._last_steering, 0.0

        self._smoothed_x = (self._smoothing * self._smoothed_x
                             + (1.0 - self._smoothing) * midpoint_x)
        steering = (self._smoothed_x - IMAGE_CENTER_X_PX) * -self._steering_gain
        steering = max(-MAX_STEERING_RAD, min(MAX_STEERING_RAD, steering))
        self._last_steering = steering
        return steering, accel

    def _publish(self, header, steering: float, accel: float) -> None:
        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = header.frame_id
        out.drive.steering_angle = float(steering)
        out.drive.acceleration = float(accel)
        self._cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CorridorController2D()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
