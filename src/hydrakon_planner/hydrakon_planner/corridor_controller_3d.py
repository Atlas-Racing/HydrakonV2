"""3D metric-position cone-corridor controller (Trackdrive / Autocross).

Uses zed_msgs/ObjectsStamped's metric obj.position (x=forward, y=left-positive,
camera frame) instead of pixel bounding boxes. Closest cone is chosen by smallest
forward distance (position.x), not by largest bbox y-pixel as in the 2D version -
we have real range now, no need for the pixel heuristic.
"""
import math
import re

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String
from zed_msgs.msg import ObjectsStamped

AMI_RE = re.compile(r'AMI:(\w+)')

# NOTE: see corridor_controller_2d.py - last year's code hardcoded '== SKIDPAD'
# only because AMI was faked during test sessions. This algorithm targets
# Trackdrive/Autocross; override 'active_ami_states' if a different mission set
# is needed.
DEFAULT_ACTIVE_AMI_STATES = ['TRACKDRIVE', 'AUTOCROSS']

TRACKING_STATE_OK = 1
TRACKING_STATE_SEARCHING = 2

MAX_STEERING_RAD = 0.4


def _is_tracked(obj) -> bool:
    return obj.tracking_state in (TRACKING_STATE_OK, TRACKING_STATE_SEARCHING)


class CorridorController3D(Node):

    def __init__(self):
        super().__init__('corridor_controller_3d')

        self.declare_parameter('active_ami_states', DEFAULT_ACTIVE_AMI_STATES)
        self.declare_parameter('midpoint_smoothing', 0.7)
        self.declare_parameter('steering_decay', 0.9)
        # Metric equivalent of the 2D version's pixel separation threshold; unlike
        # the pixel gain, there's no clean unit-conversion from the old 50px value
        # (depends on focal length/FOV), so this is a fresh starting guess - tune
        # on-vehicle.
        self.declare_parameter('separation_threshold_m', 0.5)
        # Metric analogue of the old 180px single-color curvature-following offset.
        # ~half a typical ~3m skidpad/track lane width, aiming for lane center from
        # a single boundary cone. Starting guess - tune on-vehicle.
        self.declare_parameter('single_color_offset_m', 1.5)
        self.declare_parameter('accel_nominal', 0.9)
        self.declare_parameter('accel_ambiguous', 0.6)
        self.declare_parameter('accel_single_color', 0.72)

        self._active_ami_states = set(self.get_parameter('active_ami_states').value)
        self._smoothing = self.get_parameter('midpoint_smoothing').value
        self._decay = self.get_parameter('steering_decay').value
        self._sep_threshold_m = self.get_parameter('separation_threshold_m').value
        self._single_offset_m = self.get_parameter('single_color_offset_m').value
        self._accel_nominal = self.get_parameter('accel_nominal').value
        self._accel_ambiguous = self.get_parameter('accel_ambiguous').value
        self._accel_single = self.get_parameter('accel_single_color').value

        self._ami_state = None
        self._last_steering = 0.0
        self._smoothed_x = 1.0  # forward distance, avoid atan2(0,0) before first detection
        self._smoothed_y = 0.0

        self._state_sub = self.create_subscription(
            String, '/hydrakon_can/state_str', self._on_state_str, 1)
        self._objects_sub = self.create_subscription(
            ObjectsStamped, '/zed/zed_node/obj_det/objects', self._on_objects, 10)
        self._cmd_pub = self.create_publisher(
            AckermannDriveStamped, '/hydrakon_can/command', 1)

        self.get_logger().info(
            f'corridor_controller_3d active for AMI states: {sorted(self._active_ami_states)}')

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

        y = min(yellow, key=lambda o: o.position[0], default=None)
        b = min(blue, key=lambda o: o.position[0], default=None)

        if y is not None and b is not None:
            mid_x = (y.position[0] + b.position[0]) / 2.0
            mid_y = (y.position[1] + b.position[1]) / 2.0
            lateral_sep = abs(y.position[1] - b.position[1])
            # y.position[1] > b.position[1] mirrors the old pixel code's "yellow
            # left of blue" convention (image-x-small == physical-left == larger
            # metric y here). Carried forward as-is from last year's logic -
            # verify against actual FS boundary-color convention before trusting
            # this gate for anything safety-critical.
            accel = (self._accel_nominal
                     if lateral_sep > self._sep_threshold_m and y.position[1] > b.position[1]
                     else self._accel_ambiguous)
        elif y is not None or b is not None:
            single = y if y is not None else b
            lateral_offset = self._single_offset_m if single is y else -self._single_offset_m
            mid_x = single.position[0]
            mid_y = single.position[1] + lateral_offset
            accel = self._accel_single
        else:
            self._last_steering *= self._decay
            return self._last_steering, 0.0

        self._smoothed_x = self._smoothing * self._smoothed_x + (1.0 - self._smoothing) * mid_x
        self._smoothed_y = self._smoothing * self._smoothed_y + (1.0 - self._smoothing) * mid_y

        # atan2(y, x): y=left-positive -> positive angle = steer left, matches
        # AckermannDrive's "positive yaw is to the left" convention directly, no
        # sign flip needed (unlike the 2D version's -steering_gain).
        steering = math.atan2(self._smoothed_y, self._smoothed_x)
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
    node = CorridorController3D()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
