"""Skidpad mission controller.

Ported from last year's CarMaker-era `skidpad_node.py` (raw `/hydrakon/cones`
MarkerArray input, `vehiclecontrol_msgs/VehicleControl` output) onto the same
real-vehicle interfaces `corridor_controller_2d`/`corridor_controller_3d`
already use for Trackdrive/Autocross:

  - Cones come from `zed_msgs/ObjectsStamped` on `/zed/zed_node/obj_det/objects`
    (already-classified, already-tracked detections -- label string + metric
    `position`), not a hand-rolled color-threshold MarkerArray parse.
  - AMI gating follows the same `AMI:<NAME>` string on `/hydrakon_can/state_str`
    + `active_ami_states` parameter pattern, defaulting to `['SKIDPAD']`.
  - Output is `ackermann_msgs/AckermannDriveStamped` on `/hydrakon_can/command`
    (`drive.steering_angle` rad, `drive.acceleration` a signed accel term --
    see `hydrakon_can.cpp::commandCallback` -- not a separate gas/brake pair).
  - IMU/wheel speed come from `/hydrakon_can/imu` (`sensor_msgs/Imu`) and
    `/hydrakon_can/wheel_speed` (`hydrakon_can/WheelSpeed`, RPM per wheel)
    instead of the old `/carmaker/imu` + `/carmaker/wheel_speeds`.

The skidpad-specific driving logic itself (phase state machine, angle-aware
opposite-circle cone mask, inner-cone safety carve-out, potential-field
avoidance, debounced early-exit) is carried over unchanged from the legacy
node -- see its docstring history for why each piece is shaped the way it is.
Only the I/O layer changed.
"""
import math
import re

import numpy as np
import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from hydrakon_can.msg import WheelSpeed
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String
from zed_msgs.msg import ObjectsStamped

AMI_RE = re.compile(r'AMI:(\w+)')
DEFAULT_ACTIVE_AMI_STATES = ['SKIDPAD']

TRACKING_STATE_OK = 1
TRACKING_STATE_SEARCHING = 2

# hydrakon_can.cpp truncates AI2VCU_STEER_ANGLE_REQUEST_deg to
# MAX_STEERING_ANGLE_DEG_ (21.0) regardless of what we send -- match that
# limit here rather than clipping to some other value at this layer.
MAX_STEERING_RAD = math.radians(21.0)

# WheelSpeed.msg reports RPM per wheel (see hydrakon_can.hpp / .cpp's own
# twist conversion: wheel_speed_rpm * pi * wheel_radius / 30 == m/s).
RPM_TO_RADS = math.pi / 30.0


def _yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _is_tracked(obj) -> bool:
    return obj.tracking_state in (TRACKING_STATE_OK, TRACKING_STATE_SEARCHING)


class PID:
    def __init__(self, kp, ki, kd, lo, hi):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.lo, self.hi = lo, hi
        self.prev_e = 0.0
        self.integ = 0.0

    def update(self, e, dt):
        if dt <= 0.0:
            return 0.0
        self.integ += e * dt
        d = (e - self.prev_e) / dt
        self.prev_e = e
        return float(np.clip(self.kp * e + self.ki * self.integ + self.kd * d, self.lo, self.hi))


WARMUP, ENTRY, RIGHT, LEFT, EXIT, STOPPING, DONE = 0, 1, 2, 3, 4, 5, 6
_NAME = ['WARMUP', 'ENTRY', 'RIGHT', 'LEFT', 'EXIT', 'STOPPING', 'DONE']


class SkidpadController(Node):
    """Phase-state-machine skidpad controller, real-vehicle I/O.

    Lineage / rationale (so the next person doesn't re-break this):

    - v1 (y-threshold cone mask) drove the two circles cleanly but crashed at
      the crossover: a hard `abs(y) > 2.5` mask in the *current vehicle frame*
      is fine mid-circle, but right at the S-curve the opposite circle's
      cones rotate into that same |y| band and get paired into our own
      centerline, actively steering us toward them.

    - v2 (dual-ring filter against dead-reckoned circle centers) fixed the
      crossover but then crashed into its OWN inner cones mid-circle. Root
      cause: the dead reckoning integrates raw, uncorrected IMU yaw/speed for
      up to a full ~730 deg phase. Any small bias drifts the estimated own-
      circle center, so by mid-lap the ring test starts rejecting real inner
      cones as "not mine". The "sign check" then zeroes out whatever's left
      of the centerline term whenever it disagrees with the open-loop arc
      direction -- which is exactly when the filter has already gone bad --
      so the car is left with only the open-loop arc and a (now blind-sided)
      APF, and clips the inner cones.

    Fix strategy used here:
      1. Drop dead reckoning / global center estimation entirely -- it's a
         drift-prone solution to a problem that doesn't need global position.
      2. Replace the hard y-threshold mask with an angle-aware mask: tight
         (aggressive) opposite-circle rejection for the bulk of the lap,
         widening only near the crossover -- combined with a hard cone-range
         cap so a wider window still can't pull in the far-side circle.
      3. Add an explicit "inner cone" safety carve-out: a close, inner-side
         cone is NEVER masked out, regardless of lap angle. This is the
         direct fix for v2's failure mode -- the cones most likely to be
         hit are the ones we must never drop.
      4. Keep v1's phase_margin steer clamp (limits -- but doesn't zero --
         centerline influence toward the wrong side) instead of v2's
         all-or-nothing sign check, so a slightly-noisy centerline near the
         crossover still contributes a partial, useful correction.
      5. Absorb v2's genuinely independent improvement: the debounced,
         lap-fraction-gated early-exit trigger, which removes a real
         single-frame noise/race condition in v1's transition logic and has
         nothing to do with either crash.
      6. Use v1's stronger blue/yellow APF defaults (radius/gain) as the
         safety-net field, since v2 had weakened these at the same time its
         primary filter became unreliable -- compounding the inner-cone risk.
    """

    def __init__(self):
        super().__init__('skidpad_controller')

        # AMI gating -- same pattern as corridor_controller_2d/3d.
        self.declare_parameter('active_ami_states', DEFAULT_ACTIVE_AMI_STATES)

        # geometry / sequence
        self.declare_parameter('entry_distance', 12.0)    # turn-in before the arc (empirically tuned)
        self.declare_parameter('exit_distance', 20.0)

        # Core Parameters
        self.declare_parameter('target_radius', 8.3)      # FS skidpad driving-line radius (road file)
        # Outward bias added to target_radius for the OPEN-LOOP arc only.
        # The official skidpad inner cone ring sits at ~7.9m, only 0.4m
        # inside target_radius -- razor thin once steer-rate lag during the
        # entry/crossover transients is accounted for (the car can't snap to
        # circle_steer instantly, so the actually-driven radius is briefly
        # tighter than target_radius, i.e. closer to the inner ring, not
        # farther). This bias pushes the driven arc a bit further from the
        # inner cones without touching the published/visualized target_radius.
        self.declare_parameter('inner_margin_bias', 0.5)
        self.declare_parameter('lap_angle_deg', 726.0)    # exactly 2 laps -> back at the crossover heading
        self.declare_parameter('cruise_speed', 2.0)
        self.declare_parameter('speed_kp', 0.4)
        self.declare_parameter('speed_ki', 0.02)
        self.declare_parameter('speed_kd', 0.0)
        self.declare_parameter('max_gas', 0.7)
        self.declare_parameter('max_brake', 0.5)
        self.declare_parameter('accel_rate', 1.0)
        self.declare_parameter('steer_rate', 5.0)
        self.declare_parameter('wheel_radius', 0.2508)
        self.declare_parameter('cone_weight', 0.5)

        self.declare_parameter('lookahead', 3.0)
        self.declare_parameter('track_half_width', 1.5)   # lane half-width (cones at path +-1.5 m)
        self.declare_parameter('pair_max_dist', 5.0)
        self.declare_parameter('wheelbase', 1.6)
        self.declare_parameter('steering_gain', 1.0)
        self.declare_parameter('max_steer', 0.5)
        self.declare_parameter('phase_margin', 0.0)       # 0 = never steer toward the opposite circle mid-lap

        # --- Angle-aware opposite-circle cone mask (replaces v1's flat
        # y-threshold AND v2's dead-reckoned dual-ring test) ---
        # Tight mid-lap, intentionally widened only in the crossover window,
        # with a hard range cap so widening can never pull in far-side cones.
        self.declare_parameter('mask_y_tight', 2.0)       # mid-lap opposite-side rejection band
        self.declare_parameter('mask_y_wide', 3.2)        # crossover-window rejection band
        self.declare_parameter('crossover_window_deg', 40.0)  # +/- this many deg around start/end of phase
        self.declare_parameter('mask_max_range', 6.0)     # hard cap: never pair a cone farther than this
        # Inner-cone safety carve-out: a close cone on the side the circle is
        # curving toward is NEVER masked, at any lap angle. This is the direct
        # countermeasure for v2's inner-cone crash.
        self.declare_parameter('inner_cone_range', 4.0)
        # Dedicated inner-side repulsion field. The generic APF below only
        # fires for cones within 0 < x < 3.0m -- i.e. it's a last-resort save
        # once a cone is nearly on top of you, not a standoff mechanism. This
        # field activates earlier (longer range) and ONLY for cones on the
        # inner side of the current turn, so it builds in extra clearance
        # from the inner ring well before the generic APF would ever trigger.
        self.declare_parameter('inner_repulse_range', 3.0)
        self.declare_parameter('inner_repulse_gain', 0.9)

        # Obstacle Avoidance Force Field Parameters (blue/yellow boundary cones)
        # (v1's values -- stronger than v2's, used here as the safety net)
        self.declare_parameter('avoidance_radius', 2.0)
        self.declare_parameter('avoidance_gain', 0.7)
        # Stronger, wider field for the central ORANGE gate so we always give the
        # opposite circle's orange cones a berth when threading the crossover.
        self.declare_parameter('orange_avoid_radius', 3.0)
        self.declare_parameter('orange_avoid_gain', 2.0)

        # Camera Warmup Hold Parameter
        self.declare_parameter('warmup_delay_sec', 5.0)

        # How long to keep driving straight after clearing the exit gate
        # before actually braking to a stop -- mirrors hydrakon_can.cpp's
        # Static Inspection A pattern (a timed stage after the last driving
        # action, then a one-shot mission-complete flag), rather than braking
        # the instant a gate cone gets close.
        self.declare_parameter('stop_delay_sec', 1.0)

        # Debounce for the early phase-transition trigger (from v2): require
        # the orange gate to be seen for several consecutive cone callbacks,
        # AND require a minimum fraction of the lap to already be complete,
        # before allowing an early exit. Removes a single-frame noise/race
        # condition that v1 was exposed to.
        self.declare_parameter('early_exit_debounce', 4)
        self.declare_parameter('early_exit_min_lap_frac', 0.75)

        g = lambda n: self.get_parameter(n).value
        self._active_ami_states = set(g('active_ami_states'))
        self.entry_distance = g('entry_distance')
        self.exit_distance = g('exit_distance')
        self.target_radius = g('target_radius')
        self.inner_margin_bias = g('inner_margin_bias')
        self.wheelbase = g('wheelbase')
        self.circle_steer = math.atan(self.wheelbase / (self.target_radius + self.inner_margin_bias))
        self.lap_angle = math.radians(g('lap_angle_deg'))
        self.cruise_speed = g('cruise_speed')
        self.max_gas = g('max_gas')
        self.max_brake = g('max_brake')
        self.accel_rate = g('accel_rate')
        self.steer_rate = g('steer_rate')
        self.wheel_radius = g('wheel_radius')
        self.cone_weight = g('cone_weight')
        self.lookahead = g('lookahead')
        self.half_width = g('track_half_width')
        self.pair_max_dist = g('pair_max_dist')
        self.steering_gain = g('steering_gain')
        self.max_steer = g('max_steer')
        self.phase_margin = g('phase_margin')

        self.mask_y_tight = g('mask_y_tight')
        self.mask_y_wide = g('mask_y_wide')
        self.crossover_window = math.radians(g('crossover_window_deg'))
        self.mask_max_range = g('mask_max_range')
        self.inner_cone_range = g('inner_cone_range')
        self.inner_repulse_range = g('inner_repulse_range')
        self.inner_repulse_gain = g('inner_repulse_gain')

        self.avoidance_radius = g('avoidance_radius')
        self.avoidance_gain = g('avoidance_gain')
        self.orange_avoid_radius = g('orange_avoid_radius')
        self.orange_avoid_gain = g('orange_avoid_gain')
        self.warmup_delay_sec = g('warmup_delay_sec')
        self.stop_delay_sec = g('stop_delay_sec')

        self.early_exit_debounce = int(g('early_exit_debounce'))
        self.early_exit_min_lap_frac = g('early_exit_min_lap_frac')

        self._pid = PID(g('speed_kp'), g('speed_ki'), g('speed_kd'), -self.max_brake, self.max_gas)

        self._ami_state = None
        self._state_sub = self.create_subscription(
            String, '/hydrakon_can/state_str', self._on_state_str, 1)
        self._imu_sub = self.create_subscription(
            Imu, '/hydrakon_can/imu', self._on_imu, 10)
        self._wheel_sub = self.create_subscription(
            WheelSpeed, '/hydrakon_can/wheel_speed', self._on_wheel_speed, 10)
        self._objects_sub = self.create_subscription(
            ObjectsStamped, '/zed/zed_node/obj_det/objects', self._on_objects, 10)
        self._cmd_pub = self.create_publisher(
            AckermannDriveStamped, '/hydrakon_can/command', 1)
        # Mirrors hydrakon_can.cpp's own inspection-handler pattern (see
        # inspection_handlers.cpp::handleStaticInspectionA, final stage):
        # signal completion by publishing std_msgs/Bool(true) on
        # /hydrakon_can/is_mission_completed, which hydrakon_can.cpp's
        # flagCallback consumes to set mission_complete_ (in turn reported
        # to the VCU as MISSION_FINISHED).
        self._mission_complete_pub = self.create_publisher(
            Bool, '/hydrakon_can/is_mission_completed', 1)

        # Initialize in the WARMUP state
        self._phase = WARMUP
        self._steer = 0.0
        self._throttle = 0.0
        self._speed = 0.0
        self._phase_dist = 0.0
        self._phase_time = 0.0
        self._yaw_prev: float | None = None
        self._cum_yaw = 0.0
        self._phase_yaw = 0.0
        self._turned = 0.0  # signed progress angle within current phase
        self._cone_steer = 0.0
        self._avoidance_steer = 0.0
        self._inner_repulse_steer = 0.0
        self._have_centerline = False
        self._have_imu = False
        self._have_wheels = False
        self._orange_cones = []
        self._dt = 0.05

        # Early-exit debounce counters (from v2)
        self._early_count_right = 0
        self._early_count_left = 0

        # Warmup tracker
        self._node_start_time = None

        self.create_timer(self._dt, self._timer_cb)
        self.get_logger().info(
            f'skidpad_controller active for AMI states: {sorted(self._active_ami_states)}, '
            f'brakes locked for {self.warmup_delay_sec}s camera warmup.')

    # ------------------------------------------------------------------

    def _on_state_str(self, msg: String) -> None:
        match = AMI_RE.search(msg.data)
        if match:
            self._ami_state = match.group(1)

    def _on_wheel_speed(self, msg: WheelSpeed):
        rpm_mean = (msg.lf_speed + msg.rf_speed + msg.lb_speed + msg.rb_speed) / 4.0
        self._speed = max(0.0, rpm_mean * RPM_TO_RADS * self.wheel_radius)
        self._have_wheels = True

    def _on_imu(self, msg: Imu):
        yaw = _yaw_from_quat(msg.orientation)
        if self._yaw_prev is not None:
            d = yaw - self._yaw_prev
            while d > math.pi: d -= 2 * math.pi
            while d < -math.pi: d += 2 * math.pi
            self._cum_yaw += d
        self._yaw_prev = yaw
        self._have_imu = True

    def _current_mask_y(self):
        """Angle-aware opposite-circle rejection threshold. Tight for the
        bulk of the lap (matches v1's good mid-circle behaviour); widened
        only inside the crossover window at the start/end of a phase, where
        v1's flat threshold was too aggressive and started rejecting our own
        plausible lane cones / pulling in the wrong ones. Widening alone
        would reopen the door to opposite-circle cones, so it is always
        paired with the hard mask_max_range cap in _on_objects."""
        turned = abs(self._turned)
        near_start = turned < self.crossover_window
        near_end = turned > (self.lap_angle - self.crossover_window)
        if near_start or near_end:
            return self.mask_y_wide
        return self.mask_y_tight

    def _on_objects(self, msg: ObjectsStamped):
        blue, yellow, orange = [], [], []
        apf_cones = []  # EVERY tracked cone (unmasked) -> fed to the APF safety field

        mask_y = self._current_mask_y()

        for o in msg.objects:
            if not _is_tracked(o):
                continue
            x, y = float(o.position[0]), float(o.position[1])
            if x <= 0.1:
                continue

            is_orange = o.label in ('orange_cone', 'large_orange_cone')

            # The APF sees EVERY cone -- including the opposite circle and the
            # central orange gate. A potential field only ever pushes *away*
            # from cones, never toward them, so this alone keeps us off the
            # opposite circle's orange when we re-pass the crossover each lap,
            # and is also the last line of defence against inner cones if the
            # centerline mask ever gets a borderline case wrong.
            apf_cones.append((x, y, is_orange))

            if is_orange:
                orange.append(np.array([x, y]))
                continue

            if o.label not in ('yellow_cone', 'blue_cone'):
                continue

            # --- Centreline pairing mask ---
            # On the RIGHT circle the opposite (LEFT) loop sits to our left
            # (+y); on the LEFT circle, to our right (-y). Our own outer
            # cones are only ~1.5 m off, so a few-metre band keeps them while
            # dropping the opposite loop -- EXCEPT we never drop a close,
            # inner-side cone, no matter what: those are the cones a wrong
            # rejection would actually run us into.
            is_inner_side = (y < 0.0) if self._phase == RIGHT else (y > 0.0)
            is_close = x < self.inner_cone_range

            if self._phase == RIGHT and y > mask_y and not (is_inner_side and is_close):
                continue
            if self._phase == LEFT and y < -mask_y and not (is_inner_side and is_close):
                continue

            # Hard range cap: independent of the y-window above, never pair a
            # cone that's farther than this. This is what stops the widened
            # crossover window from reopening the door to the far-side circle
            # (the exact failure mode v1 had at the intersection).
            if math.hypot(x, y) > self.mask_max_range:
                continue

            if x > self.lookahead + 5.0 or abs(y) > 6.0:
                continue

            if o.label == 'blue_cone':
                blue.append(np.array([x, y]))
            else:
                yellow.append(np.array([x, y]))

        self._orange_cones = orange

        # APF repulsion. The orange centre gate gets a wider, stronger field
        # than the blue/yellow boundary cones so the opposite circle's orange
        # always gets a berth at the crossover.
        repulsion_steer = 0.0
        for x, y, is_orange in apf_cones:
            if not (0.0 < x < 3.0):
                continue
            radius = self.orange_avoid_radius if is_orange else self.avoidance_radius
            gain = self.orange_avoid_gain if is_orange else self.avoidance_gain
            dist = math.hypot(x, y)
            if dist < radius:
                urgency = (radius - dist) / radius
                repulsion_steer += math.copysign(urgency * gain, -y)

        self._avoidance_steer = repulsion_steer

        # Dedicated INNER-SIDE repulsion. The generic APF above only fires
        # for cones with 0 < x < 3.0m -- by the time a cone is that close
        # you're already nearly on top of it, so it's a last-resort save, not
        # a standoff distance. This field activates over a longer forward
        # range (inner_repulse_range) and ONLY considers blue/yellow cones on
        # the inner side of the CURRENT turn (never orange, never the outer
        # side), pushing the steer command away from the turn center earlier
        # and more progressively -- this is what actually buys extra
        # clearance from the inner ring instead of just reacting once it's
        # too late.
        inner_push = 0.0
        for x, y, is_orange in apf_cones:
            if is_orange:
                continue
            if not (0.0 < x < self.inner_repulse_range):
                continue
            is_inner_side = (y < 0.0) if self._phase == RIGHT else (y > 0.0)
            if not is_inner_side:
                continue
            dist = math.hypot(x, y)
            if dist < self.inner_repulse_range:
                urgency = (self.inner_repulse_range - dist) / self.inner_repulse_range
                # Push away from the inner side: RIGHT phase inner side is
                # -y, so push toward +steer (away from -y); LEFT phase inner
                # side is +y, so push toward -steer. copysign with -y here
                # already encodes "push toward +y if cone is at -y" and vice
                # versa, same convention as the generic APF above.
                inner_push += math.copysign(urgency * self.inner_repulse_gain, -y)

        self._inner_repulse_steer = inner_push

        if not (blue or yellow):
            self._have_centerline = False
            return

        blue.sort(key=lambda p: math.hypot(p[0], p[1]))
        yellow.sort(key=lambda p: math.hypot(p[0], p[1]))

        pts, used = [], set()

        for bcone in blue:
            best, bd, bi = None, float('inf'), -1
            for i, ycone in enumerate(yellow):
                if i in used:
                    continue
                d = math.hypot(bcone[0] - ycone[0], bcone[1] - ycone[1])
                if d < bd and d < self.pair_max_dist:
                    bd, best, bi = d, ycone, i
            if best is not None:
                used.add(bi)
                pts.append((bcone + best) / 2.0)
            else:
                pts.append(bcone + np.array([0.0, -self.half_width]))

        for i, ycone in enumerate(yellow):
            if i not in used:
                pts.append(ycone + np.array([0.0, self.half_width]))

        pts.sort(key=lambda p: math.hypot(p[0], p[1]))

        target = min(pts, key=lambda p: abs(math.hypot(p[0], p[1]) - self.lookahead))
        tx, ty = float(target[0]), float(target[1])

        ld = math.hypot(tx, ty)
        if ld < 0.5:
            self._have_centerline = False
            return

        alpha = math.atan2(ty, tx)

        active_gain = self.steering_gain
        if self._phase in (LEFT, EXIT) and self._phase_dist < 5.0:
            active_gain = 2.0

        steer = math.atan2(2.0 * self.wheelbase * math.sin(alpha), ld) * active_gain
        self._cone_steer = float(np.clip(steer, -self.max_steer, self.max_steer))
        self._have_centerline = True

    # ------------------------------------------------------------------

    def _timer_cb(self):
        if self._ami_state not in self._active_ami_states:
            return  # not gated for this mission; don't publish - avoid fighting other controllers

        if not (self._have_imu and self._have_wheels):
            self._publish(0.0, -self.max_brake)
            return

        # =================================================================
        # 1. THE WARMUP HOLD BLOCK
        # =================================================================
        if self._phase == WARMUP:
            if self._node_start_time is None:
                self._node_start_time = self.get_clock().now()

            elapsed_sec = (self.get_clock().now() - self._node_start_time).nanoseconds / 1e9

            if elapsed_sec < self.warmup_delay_sec:
                if int(elapsed_sec * 20) % 20 == 0:
                    self.get_logger().info(f"Warming up YOLO... {self.warmup_delay_sec - elapsed_sec:.1f}s remaining")

                # Output strict lock: 0 steer, full brake
                self._steer = 0.0
                self._throttle = -self.max_brake
                self._publish(0.0, -self.max_brake)
                return
            else:
                self._enter(ENTRY, 'Warmup complete. Releasing brakes and launching!')

        self._phase_dist += self._speed * self._dt
        self._phase_time += self._dt
        turned = self._cum_yaw - self._phase_yaw
        self._turned = turned

        orange_at_crossing = any(p[0] < 3.0 for p in self._orange_cones)

        # Debounced + lap-fraction-gated early exit trigger (from v2): requires
        # both a minimum fraction of the lap angle covered AND several
        # consecutive timer ticks of orange detection, so a single noisy or
        # misclassified cone can't flip the phase while still on the old
        # circle. This was an independent improvement in v2, unrelated to
        # either crash, so it's kept as-is.
        EARLY_RAD = math.radians(30.0)
        min_turned_for_early = self.lap_angle * self.early_exit_min_lap_frac

        if self._phase == RIGHT:
            if orange_at_crossing and abs(turned) >= min_turned_for_early:
                self._early_count_right += 1
            else:
                self._early_count_right = 0
        elif self._phase == LEFT:
            if orange_at_crossing and abs(turned) >= min_turned_for_early:
                self._early_count_left += 1
            else:
                self._early_count_left = 0

        # --- phase machine + target steer ---
        if self._phase == ENTRY:
            # =================================================================
            # 2. THE BLIND ENTRY BLOCK
            # Target steer is strictly 0.0. We ignore orange cones and APF completely.
            # =================================================================
            target_steer = 0.0
            if self._phase_dist >= self.entry_distance:
                self._enter(RIGHT, f'Drove {self.entry_distance}m blind -> RIGHT circle x2')

        elif self._phase == RIGHT:
            # Hold the right-hand arc for the full two laps. Do NOT coast
            # straight near the end: that freezes `turned` (no yaw change
            # while straight) so the angle trigger could never fire, and the
            # car would sail past the crossover and re-enter the next circle
            # displaced. The S-curve reversal happens naturally via steer
            # smoothing once we switch phase.
            target_steer = -self.circle_steer

            early = abs(turned) >= self.lap_angle - EARLY_RAD and self._early_count_right >= self.early_exit_debounce
            if abs(turned) >= self.lap_angle or early:
                self._enter(LEFT, '2 right laps done -> LEFT circle x2')

        elif self._phase == LEFT:
            target_steer = +self.circle_steer

            early = abs(turned) >= self.lap_angle - EARLY_RAD and self._early_count_left >= self.early_exit_debounce
            if abs(turned) >= self.lap_angle or early:
                self._enter(EXIT, '2 left laps done -> EXIT')

        elif self._phase == EXIT:
            exit_orange = [p for p in self._orange_cones if p[0] > 5.0]
            if exit_orange:
                avg_x = float(np.mean([p[0] for p in exit_orange]))
                avg_y = float(np.mean([p[1] for p in exit_orange]))
                ld = math.hypot(avg_x, avg_y)
                alpha = math.atan2(avg_y, avg_x)

                active_gain = 2.0 if self._phase_dist < 5.0 else self.steering_gain

                target_steer = float(np.clip(
                    math.atan2(2.0 * self.wheelbase * math.sin(alpha), ld) * active_gain,
                    -self.max_steer, self.max_steer))
            else:
                target_steer = 0.0

            orange_close = (self._phase_dist > 5.0
                            and any(p[0] < 1.0 for p in self._orange_cones))
            if orange_close or self._phase_dist >= self.exit_distance:
                self._enter(STOPPING, f'through the gate -> stopping in {self.stop_delay_sec}s')

        elif self._phase == STOPPING:
            # Timed stage after the last driving action, same shape as
            # hydrakon_can.cpp's inspection handlers (drive/hold for a fixed
            # duration, then advance) -- keep driving straight for
            # stop_delay_sec after clearing the gate instead of braking the
            # instant a gate cone gets close, then actually stop.
            target_steer = 0.0
            if self._phase_time >= self.stop_delay_sec:
                self._enter(DONE, 'stop delay elapsed -> STOP, mission complete')

        else:  # DONE
            target_steer = 0.0
            self._publish_mission_complete()

        # --- blend the open-loop circle with cone centreline pursuit ---
        # Kept active right through the crossover so the cones guide the
        # S-curve onto the next circle. Use v1's phase_margin CLAMP rather
        # than v2's sign-check-and-zero: clamping limits, but never discards,
        # the centerline's contribution toward the wrong side, so a slightly
        # noisy centerline near the crossover still provides a useful partial
        # correction instead of leaving the car on open-loop arc alone
        # (which is what let v2 clip its own inner cones once the centerline
        # got zeroed out).
        if self._have_centerline and self._phase in (RIGHT, LEFT):
            target_steer = (1.0 - self.cone_weight) * target_steer \
                           + self.cone_weight * self._cone_steer

            if self._phase == RIGHT:
                target_steer = min(target_steer, self.phase_margin)
            else:
                target_steer = max(target_steer, -self.phase_margin)

        # =================================================================
        # Apply the Obstacle Avoidance (APF) safety push ONLY after ENTRY
        # =================================================================
        if self._phase not in (WARMUP, ENTRY):
            if abs(self._avoidance_steer) > 0.01:
                target_steer += self._avoidance_steer

        # Dedicated inner-ring standoff push -- only meaningful while
        # actually circling (RIGHT/LEFT), where "inner side" is well-defined
        # relative to the current turn direction. This is what builds in
        # extra clearance from the inner cone ring proactively, rather than
        # relying solely on the generic close-range APF above to react once
        # a cone is nearly on top of the car.
        if self._phase in (RIGHT, LEFT):
            if abs(self._inner_repulse_steer) > 0.01:
                target_steer += self._inner_repulse_steer

        target_steer = float(np.clip(target_steer, -self.max_steer, self.max_steer))

        # --- smooth steer ---
        max_dsteer = self.steer_rate * self._dt
        self._steer += float(np.clip(target_steer - self._steer, -max_dsteer, max_dsteer))
        # hydrakon_can.cpp truncates to MAX_STEERING_RAD (21 deg) regardless
        # of what we send -- clip here too so logging reflects reality.
        self._steer = float(np.clip(self._steer, -MAX_STEERING_RAD, MAX_STEERING_RAD))

        # --- speed: cruise, or brake when done ---
        max_da = self.accel_rate * self._dt
        if self._phase == DONE:
            self._throttle += float(np.clip(-self.max_brake - self._throttle, -max_da, max_da))
        else:
            cmd = self._pid.update(self.cruise_speed - self._speed, self._dt)
            self._throttle += float(np.clip(cmd - self._throttle, -max_da, max_da))

        self._publish_command()

        self.get_logger().info(
            f'phase={_NAME[self._phase]} dist={self._phase_dist:5.1f} '
            f'turned={math.degrees(turned):6.0f} spd={self._speed:.1f} '
            f'steer={self._steer:+.3f} cone_steer={self._cone_steer:+.3f} '
            f'inner_push={self._inner_repulse_steer:+.3f} '
            f'cl={int(self._have_centerline)}',
            throttle_duration_sec=0.5)

    def _enter(self, phase, msg):
        self.get_logger().warn(f'Skidpad: {msg}')
        self._phase = phase
        self._phase_dist = 0.0
        self._phase_time = 0.0
        self._phase_yaw = self._cum_yaw
        self._turned = 0.0
        self._cone_steer = 0.0
        self._have_centerline = False
        self._early_count_right = 0
        self._early_count_left = 0

    # ------------------------------------------------------------------

    def _publish_mission_complete(self) -> None:
        # Republished every tick while DONE (cheap, and QoS depth 1 gives no
        # latching) so hydrakon_can.cpp's flagCallback reliably sees it even
        # if the first message races node startup/discovery.
        msg = Bool()
        msg.data = True
        self._mission_complete_pub.publish(msg)

    def _publish_command(self) -> None:
        # `drive.acceleration` is a single signed term (not gas/brake pair):
        # >0 => throttle/RPM request, ~0 => coast, a bit negative => engine
        # braking, more negative => mechanical brakes -- see
        # hydrakon_can.cpp::commandCallback. self._throttle is already
        # clipped to [-max_brake, max_gas] by the PID, so it maps directly.
        self._publish(self._steer, self._throttle)

    def _publish(self, steer: float, accel: float) -> None:
        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.drive.steering_angle = float(steer)
        out.drive.acceleration = float(accel)
        self._cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SkidpadController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
