#!/usr/bin/env python3
"""Animate a car driving a Formula Student UK-style track.

The track (blue cones on the left, yellow on the right) comes from the
random-track-generator library (vendored in ./random_track_generator,
MIT licensed, github.com/mvanlobensels/random-track-generator) which
produces rules-compliant, self-consistent FS layouts via bounded Voronoi
diagrams: sweepers, hairpins and chicanes, not just an oval.

The car is simulated frame-by-frame (real time-stepping, not a canned
path): each step it re-localises against the track, looks at only a
local window of nearby cone pairs to interpolate its target path (drawn
live as it slides along, "sensor horizon" style), steers toward a
lookahead point with a rate-limited pure-pursuit controller, and brakes
for upcoming curvature. That control lag is what makes it reactive -
it visibly cuts corners and catches back up, rather than teleporting
along the ideal centerline.

Usage:
    python3 scripts/track_animation.py [-o OUTPUT.gif] [--seed N] [--laps N]

Requires: numpy, scipy, shapely, pyyaml, gpxpy, matplotlib, pillow
"""
import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Polygon
from scipy.interpolate import CubicSpline

sys.path.insert(0, str(Path(__file__).parent))
from random_track_generator import generate_track  # noqa: E402

FPS = 30
DT = 1.0 / FPS
V_MAX = 14.0            # m/s, top speed on straights
V_MIN = 4.5             # m/s, minimum speed through the tightest corner
MAX_ACCEL = 5.0          # m/s^2
MAX_DECEL = 9.0          # m/s^2
MAX_TURN_RATE = 2.6      # rad/s, rate-limited steering -> makes the car "reactive"
LOOKAHEAD_MIN = 4.0      # m
LOOKAHEAD_TIME = 0.45    # s, lookahead grows with speed (pure pursuit)
CURVATURE_GAIN = 55.0    # higher = brakes harder for corners
WINDOW_BEHIND = 6.0      # m, live-interpolation window shown behind the car
WINDOW_AHEAD = 16.0      # m, live-interpolation window shown ahead of the car
TRAIL_SECONDS = 2.2      # how long the car's actual driven trail stays visible
REF_N = 1600             # resolution of the interpolated reference path


def _spline_by_arclength_fraction(points):
    pts = np.vstack([points, points[:1]])
    d = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
    s = np.concatenate([[0.0], np.cumsum(d)])
    frac = s / s[-1]
    cs_x = CubicSpline(frac, pts[:, 0], bc_type="periodic")
    cs_y = CubicSpline(frac, pts[:, 1], bc_type="periodic")
    return cs_x, cs_y


def build_reference(cones_left, cones_right, n=REF_N):
    """Interpolate the track's centerline from the cone boundaries.

    Both boundaries are resampled at matching normalized-arclength
    fractions so that corresponding indices sit opposite each other
    across the track; their midpoint is the racing centerline.
    """
    lx, ly = _spline_by_arclength_fraction(cones_left)
    rx, ry = _spline_by_arclength_fraction(cones_right)
    u = np.linspace(0.0, 1.0, n, endpoint=False)
    left_fine = np.column_stack([lx(u), ly(u)])
    right_fine = np.column_stack([rx(u), ry(u)])
    center_fine = (left_fine + right_fine) / 2.0

    seg = np.hypot(*np.diff(np.vstack([center_fine, center_fine[:1]]), axis=0).T)
    s_fine = np.concatenate([[0.0], np.cumsum(seg)])[:-1]
    total_length = s_fine[-1] + seg[-1]
    return center_fine, left_fine, right_fine, s_fine, total_length


def compute_curvature(center_fine, s_fine, total_length, pad=6):
    x = np.concatenate([center_fine[-pad:, 0], center_fine[:, 0], center_fine[:pad, 0]])
    y = np.concatenate([center_fine[-pad:, 1], center_fine[:, 1], center_fine[:pad, 1]])
    s_pad = np.concatenate([s_fine[-pad:] - total_length, s_fine, s_fine[:pad] + total_length])
    dx = np.gradient(x, s_pad)
    dy = np.gradient(y, s_pad)
    ddx = np.gradient(dx, s_pad)
    ddy = np.gradient(dy, s_pad)
    kappa = np.abs(dx * ddy - dy * ddx) / np.maximum(dx**2 + dy**2, 1e-9) ** 1.5
    return kappa[pad:-pad]


def point_at_s(s, s_fine, center_fine, total_length):
    s_mod = np.mod(s, total_length)
    x = np.interp(s_mod, s_fine, center_fine[:, 0])
    y = np.interp(s_mod, s_fine, center_fine[:, 1])
    return x, y


def nearest_index(x, y, center_fine, guess_idx, n, search=50):
    idx = (guess_idx + np.arange(-search, search + 1)) % n
    d2 = (center_fine[idx, 0] - x) ** 2 + (center_fine[idx, 1] - y) ** 2
    return idx[np.argmin(d2)]


def wrap_to_pi(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def simulate(center_fine, s_fine, kappa, total_length, laps):
    n = len(s_fine)
    tx0, ty0 = center_fine[1] - center_fine[0]
    heading = np.arctan2(ty0, tx0)
    x, y = center_fine[0]
    speed = V_MIN
    idx = 0
    distance_travelled = 0.0
    target_distance = total_length * laps

    xs, ys, headings, speeds, idxs = [], [], [], [], []
    while distance_travelled < target_distance:
        idx = nearest_index(x, y, center_fine, idx, n)
        window = (idx + np.arange(0, 40)) % n
        v_target = np.clip(V_MAX / (1.0 + CURVATURE_GAIN * kappa[window].max()), V_MIN, V_MAX)
        accel_limit = MAX_ACCEL if v_target > speed else MAX_DECEL
        speed += np.clip(v_target - speed, -accel_limit * DT, accel_limit * DT)

        lookahead = LOOKAHEAD_MIN + LOOKAHEAD_TIME * speed
        tgt_x, tgt_y = point_at_s(s_fine[idx] + lookahead, s_fine, center_fine, total_length)
        desired_heading = np.arctan2(tgt_y - y, tgt_x - x)
        heading += np.clip(wrap_to_pi(desired_heading - heading), -MAX_TURN_RATE * DT, MAX_TURN_RATE * DT)

        x += speed * DT * np.cos(heading)
        y += speed * DT * np.sin(heading)
        distance_travelled += speed * DT

        xs.append(x); ys.append(y); headings.append(heading); speeds.append(speed); idxs.append(idx)

    return (np.array(xs), np.array(ys), np.array(headings), np.array(speeds), np.array(idxs))


CAR_SHAPE = np.array([[0.9, 0.0], [-0.7, 0.55], [-0.7, -0.55]])


def rotated_car(x, y, heading):
    c, s = np.cos(heading), np.sin(heading)
    R = np.array([[c, -s], [s, c]])
    return (CAR_SHAPE @ R.T) + np.array([x, y])


def make_animation(output, seed, laps, n_points, n_regions, bound, mode):
    track = generate_track(n_points, n_regions, 0.0, bound, mode=mode, seed=seed)
    cones_left, cones_right = track.as_tuple()

    center_fine, left_fine, right_fine, s_fine, total_length = build_reference(cones_left, cones_right)
    kappa = compute_curvature(center_fine, s_fine, total_length)
    n = len(s_fine)

    xs, ys, headings, speeds, idxs = simulate(center_fine, s_fine, kappa, total_length, laps)
    n_frames = len(xs)
    trail_len = max(1, int(TRAIL_SECONDS * FPS))

    margin = 8
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.set_aspect("equal")
    ax.axis("off")
    all_x = np.concatenate([cones_left[:, 0], cones_right[:, 0]])
    all_y = np.concatenate([cones_left[:, 1], cones_right[:, 1]])
    ax.set_xlim(all_x.min() - margin, all_x.max() + margin)
    ax.set_ylim(all_y.min() - margin, all_y.max() + margin)

    ax.scatter(cones_left[:, 0], cones_left[:, 1], c="tab:blue", s=22, zorder=2)
    ax.scatter(cones_right[:, 0], cones_right[:, 1], c="gold", s=22, zorder=2)

    live_path_line, = ax.plot([], [], color="lime", lw=2.5, zorder=3, solid_capstyle="round")
    live_pts = ax.scatter([], [], c="lime", s=18, zorder=4)
    trail_line, = ax.plot([], [], color="red", lw=1.4, alpha=0.6, zorder=3)
    car_patch = Polygon(rotated_car(xs[0], ys[0], headings[0]), closed=True, fc="black", zorder=5)
    ax.add_patch(car_patch)

    def update(frame):
        x, y, heading, idx = xs[frame], ys[frame], headings[frame], idxs[frame]

        s_behind = s_fine[idx] - WINDOW_BEHIND
        s_ahead = s_fine[idx] + WINDOW_AHEAD
        u = np.linspace(s_behind, s_ahead, 40)
        wx, wy = point_at_s(u, s_fine, center_fine, total_length)
        live_path_line.set_data(wx, wy)
        live_pts.set_offsets(np.column_stack([wx[::4], wy[::4]]))

        lo = max(0, frame - trail_len)
        trail_line.set_data(xs[lo:frame + 1], ys[lo:frame + 1])

        car_patch.set_xy(rotated_car(x, y, heading))
        return live_path_line, live_pts, trail_line, car_patch

    anim = FuncAnimation(fig, update, frames=n_frames, interval=1000 / FPS, blit=True)
    anim.save(output, writer=PillowWriter(fps=FPS))
    plt.close(fig)
    print(f"track length: {total_length:.1f} m, frames: {n_frames}, sim time: {n_frames / FPS:.1f} s")


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("-o", "--output", default="scripts/track_animation.gif")
    p.add_argument("--seed", type=int, default=13)
    p.add_argument("--laps", type=float, default=1.15)
    p.add_argument("--n-points", type=int, default=60)
    p.add_argument("--n-regions", type=int, default=20)
    p.add_argument("--bound", type=float, default=150.0)
    p.add_argument("--mode", default="extend", choices=["expand", "extend", "random"])
    return p.parse_args()


def main():
    args = parse_args()
    make_animation(args.output, args.seed, args.laps, args.n_points, args.n_regions, args.bound, args.mode)
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
