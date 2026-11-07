#!/usr/bin/env python3
"""LIMO local planner: two-stage DWA around Pure Pursuit. Only what the car runs.

Global path : /pp_path (map frame)     Scan : /scan     Pose : TF map->base_link
In          : /pure_pursuit/ackermann_cmd     Out : /final_ackermann_cmd
RViz        : /obstacles, /local_path, /local_candidates, /obstacle_detected

Route clear: Pure Pursuit passes through under a speed cap (FOLLOW). Something on the route: pick
the best of 63 two-arc shapes, largest radius that fits, stop if none fits (AVOID). Forward only.

Run: python3 src/dwa_planner.py --ros-args -p use_sim_time:=true
Node name stays frenet_local_planner: the monitor and preflight look for it.
"""
import math
import time
from types import SimpleNamespace

import numpy as np

WHEELBASE = 0.24            # m
MAX_STEER = 0.42            # rad
MAX_CURV = math.tan(MAX_STEER) / WHEELBASE      # ~1.86 1/m
NARROW_SPACING = 0.05       # [m] spacing the speed limit assumes along a path


def default_params():
    return SimpleNamespace(
        # route and scan
        corridor=0.28,          # [m] scan point this close to the route ahead = blocked
        detect_dist=4.0,        # [m] look-ahead for blocking points
        plan_range=5.0,         # [m] scan points farther than this are ignored
        body_r=0.12,            # [m] scan points this close to base_link = own body, ignored
        recover_off=0.40,       # [m] farther than this from the route: AVOID
        return_off=0.30,        # [m] back on the route: FOLLOW
        clear_hold=0.3,         # [s] keep AVOID this long after the way is clear
        cluster_gap=0.30,       # [m] RViz only: points closer than this are one object
        # body and clearance
        r_robot=0.20,           # [m] largest radius tried
        r_min=0.17,             # [m] smallest radius: half width 0.155 + 0.015. Below this: stop
        radius_factors=(1.0, 0.95, 0.9, 0.85, 0.8, 0.0),   # 0.0 maps to r_min
        footprint_length=0.33,  # [m]
        footprint_width=0.31,   # [m]
        stop_front=0.40,        # [m] emergency stop box: a point this close ahead ...
        stop_half=0.12,         # [m] ... and this close to the centre line
        # speed
        max_speed=0.8,          # [m/s] global cap
        avoid_max_speed=0.48,   # [m/s]
        free_speed=0.55,        # [m/s] AVOID with the way clear
        min_speed=0.12,         # [m/s]
        wall_speed=0.30,        # [m/s] lowest cap near walls
        slow_dist=1.0,          # [m] look this far along the route for walls
        wall_near=0.26,         # [m] closer than this to the route: wall_speed
        wall_far=0.45,          # [m] farther than this: no slowdown
        a_lat=0.32,             # [m/s^2] corner limit v = sqrt(a_lat / curvature)
        curve_min=0.28,         # [m/s] never slower than this because of a corner
        curve_look=2.0,         # [m] look this far along the route for corners
        steer_slow=0.30,        # [rad] |steer| at which speed has dropped to curve_min
        squeeze_speed=0.15,     # [m/s] cap below 0.9 r_robot
        narrow_look=2.0,        # [m] clearance speed limit: look this far along the path
        narrow_min_speed=0.06,  # [m/s] lowest speed it asks for
        narrow_keep=0.02,       # [m] clearance to keep after stopping
        narrow_stop_time=1.2,   # [s] stop request to standstill
        narrow_decel=0.15,      # [m/s^2] braking assumed ahead of a lower limit
        narrow_drift_time=0.25, # [s] allowed tracking drift at this speed
        # points the LiDAR saw but cannot confirm now: nearer than range_min or outside its view
        use_unseen_memory=True,
        memory_hold=15.0,       # [s] forget after this long
        memory_keep_r=1.0,      # [m] only points this close to base_link
        memory_cell=0.02,       # [m] one remembered point per cell
        memory_rmin_margin=0.03,  # [m] treat just beyond range_min as unconfirmed too
        # DWA
        dwa_w_clear=1.2,
        dwa_clear_cap=0.35,     # [m] clearance above this earns nothing
        dwa_through=0.10,       # [m] route this close to a point counts as running through it
        dwa_turn_far=2.2,       # [m] route blocked this close: start easing off the route
        dwa_turn_near=0.9,      # [m] route term at its lowest from here in
        dwa_path_min=0.15,      # lowest share of the route term
        dwa_past_block=0.8,     # [m] aim this far past the farthest blocking point
        dwa_retry_wait=0.5,     # [s] stopped with no free shape: look again after this
        dwa_target_dist=1.5,    # [m] route point the arcs aim for
        dwa_keep_gap=0.0,       # [m] body nearer than this to a point: pass over first arcs that close on it; 0 = off
    )


# ----------------------------------------------------------------- helpers
def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def periodic_cubic(s, y, sn):
    """Closed C2 cubic spline through (s, y), evaluated at sn. y[0] must equal y[-1]."""
    n = len(s) - 1
    h = np.diff(s)
    A = np.zeros((n, n))
    rhs = np.zeros(n)
    for i in range(n):
        before, after = h[i - 1], h[i]
        A[i, (i - 1) % n] += before
        A[i, i] += 2.0 * (before + after)
        A[i, (i + 1) % n] += after
        rhs[i] = 6.0 * ((y[(i + 1) % n] - y[i]) / after - (y[i] - y[(i - 1) % n]) / before)
    m = np.linalg.solve(A, rhs)
    m = np.r_[m, m[0]]
    j = np.clip(np.searchsorted(s, sn, side='right') - 1, 0, n - 1)
    d = sn - s[j]
    e = s[j + 1] - sn
    hj = h[j]
    return (m[j] * e ** 3 + m[j + 1] * d ** 3) / (6.0 * hj) \
        + (y[j] / hj - m[j] * hj / 6.0) * e + (y[j + 1] / hj - m[j + 1] * hj / 6.0) * d


def resample_closed(xy, step):
    """Closed path resampled to ~uniform spacing on a spline through its points.

    Straight-line interpolation of waypoints leaves a heading jump at each one, which
    measures as false curvature at this sampling step (2.14 1/m on a route whose real
    maximum is about 0.9) and made candidates fail the curvature limit on open track.
    """
    xy = np.asarray(xy, float)[:, :2]
    if np.hypot(*(xy[0] - xy[-1])) < 1e-6:
        xy = xy[:-1]
    c = np.vstack([xy, xy[:1]])
    seg = np.hypot(*np.diff(c, axis=0).T)
    s = np.r_[0.0, np.cumsum(seg)]
    n = max(int(round(s[-1] / step)), 8)
    sn = np.linspace(0.0, s[-1], n, endpoint=False)
    return np.c_[periodic_cubic(s, c[:, 0], sn), periodic_cubic(s, c[:, 1], sn)], s[-1]


def to_base(pts, pose):
    x, y, yaw = pose
    d = np.asarray(pts, float) - np.array([x, y])
    c, s = math.cos(yaw), math.sin(yaw)
    return np.c_[c * d[..., 0] + s * d[..., 1], -s * d[..., 0] + c * d[..., 1]]


def to_map(pts, pose):
    x, y, yaw = pose
    p = np.asarray(pts, float)
    c, s = math.cos(yaw), math.sin(yaw)
    return np.c_[c * p[..., 0] - s * p[..., 1] + x, s * p[..., 0] + c * p[..., 1] + y]


def scan_to_points(ranges, angle_min, angle_inc, range_min, range_max,
                   max_range, laser_tf=(0.0, 0.0, 0.0)):
    """LaserScan -> Nx2 points in base_link (order kept for clustering)."""
    r = np.asarray(ranges, float)
    ang = angle_min + angle_inc * np.arange(len(r))
    ok = np.isfinite(r) & (r > range_min) & (r < min(range_max, max_range))
    x, y = r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])
    tx, ty, tyaw = laser_tf
    c, s = math.cos(tyaw), math.sin(tyaw)
    return np.c_[c * x - s * y + tx, s * x + c * y + ty]


class UnseenMemory:
    """Recent scan points in a fixed frame, returned only where the sensor cannot check them."""

    STOP_WINDOW = 0.5   # [s]
    STOP_MOVE = 0.01    # [m] less than this over the window counts as stopped (0.02 m/s)

    def __init__(self, hold, keep_r, cell, rmin_margin):
        self.hold, self.keep_r, self.cell, self.rmin_margin = hold, keep_r, cell, rmin_margin
        self.clear()

    def clear(self):
        self.pts = np.zeros((0, 2))
        self.stamp = np.zeros(0)
        self.anchor = None

    def hold_while_stopped(self, now, pose):
        """A stopped car learns nothing new about a blind corner, so its memory must not age.

        Otherwise a corner beside the car expires and a path through it appears.
        """
        if self.anchor is None:
            self.anchor = (now, pose[0], pose[1])
            return
        t0, x0, y0 = self.anchor
        if now - t0 < self.STOP_WINDOW:
            return
        if math.hypot(pose[0] - x0, pose[1] - y0) < self.STOP_MOVE:
            self.stamp = self.stamp + (now - t0)
        self.anchor = (now, pose[0], pose[1])

    def add(self, now, pts_base, pose):
        """Store points given in the base frame at pose (x, y, yaw) of the fixed frame."""
        fresh = pts_base[np.hypot(pts_base[:, 0], pts_base[:, 1]) < self.keep_r] \
            if len(pts_base) else pts_base
        keep = now - self.stamp < self.hold
        pts = np.vstack([to_map(fresh, pose), self.pts[keep]]) if len(fresh) else self.pts[keep]
        stamp = np.r_[np.full(len(fresh), now), self.stamp[keep]]
        if len(pts):                         # first occurrence is the newest
            _, first = np.unique(np.floor(pts / self.cell).astype(int), axis=0, return_index=True)
            pts, stamp = pts[first], stamp[first]
        self.pts, self.stamp = pts, stamp

    def unseen(self, now, pose, laser_xy, range_min, half_fov):
        """Base-frame points now nearer than range_min to the laser or outside its view."""
        self.hold_while_stopped(now, pose)
        ok = now - self.stamp < self.hold
        if not ok.any():
            return np.zeros((0, 2))
        base = to_base(self.pts[ok], pose)
        rx, ry = base[:, 0] - laser_xy[0], base[:, 1] - laser_xy[1]
        blind = (np.hypot(rx, ry) < range_min + self.rmin_margin) | (np.abs(np.arctan2(ry, rx)) > half_fov)
        return base[blind & (np.hypot(base[:, 0], base[:, 1]) < self.keep_r)]


def clusters_of(pts, gap):
    """Split ordered scan points into objects; return centroids + sizes."""
    if len(pts) == 0:
        return []
    cut = np.where(np.hypot(*np.diff(pts, axis=0).T) > gap)[0] + 1
    out = []
    for g in np.split(pts, cut):
        out.append((g.mean(axis=0), len(g),
                    float(np.hypot(*(g.max(axis=0) - g.min(axis=0))))))
    return out


def closing_speed_limit(poses, pts, half_length, half_width, keep, stop_time, drift_time):
    """Highest speed per pose that can stop short of every point, keeping `keep` metres.

    A point counts by how fast the gap to it shrinks when driving ahead: fully straight ahead,
    not at all beside the car. Clearance alone also limits speed through drift_time.
    """
    dx = pts[None, :, 0] - poses[:, None, 0]
    dy = pts[None, :, 1] - poses[:, None, 1]
    c, s = np.cos(poses[:, 2])[:, None], np.sin(poses[:, 2])[:, None]
    u = c * dx + s * dy
    ox = np.maximum(np.abs(u) - half_length, 0.0)
    oy = np.maximum(np.abs(-s * dx + c * dy) - half_width, 0.0)
    dist = np.hypot(ox, oy)
    closing = np.where(u > 0.0, ox / np.maximum(dist, 1e-6), 0.0)
    room = np.maximum(dist - keep, 0.0)
    by_point = room / np.maximum(stop_time * closing, 1e-3)
    return np.minimum(by_point.min(axis=1), room.min(axis=1) / drift_time)


def arc(curv, length, samples):
    """Poses along an arc from the origin, heading +x: samples x (x, y, yaw), without the start."""
    d = length * np.arange(1, samples + 1) / samples
    if abs(curv) < 1e-9:
        return np.c_[d, np.zeros(samples), np.zeros(samples)]
    h = curv * d
    return np.c_[np.sin(h) / curv, (1.0 - np.cos(h)) / curv, h]


class TwoStageDWA:
    """Two arcs in a row: enough to line up and then pass, which one arc cannot see.

    A shape counts up to its first contact and needs at least min_free metres clear. Picks a
    shape only; the caller sets the speed. Forward only.

    Distances are worked out once per cycle in see(); each radius then only compares against them,
    and full clearance is computed for feasible shapes alone. Same result, far less work.
    """

    def __init__(self, wheelbase, max_steer, w_clear, clear_cap, n_first=9, n_second=7, first=0.40,
                 second=1.00, spacing=0.10, min_free=0.50, w_target=1.0, w_yaw=0.25, w_change=0.10,
                 w_free=0.5, w_path=1.5, path_scale=0.5, keep_gap=0.0, keep_half=(0.165, 0.155), rear_lr=0.12,
                 keep_tol=0.002):
        self.max_steer, self.spacing, self.min_free = max_steer, spacing, min_free
        self.w_target, self.w_yaw, self.w_clear, self.w_change, self.w_free, self.clear_cap = \
            w_target, w_yaw, w_clear, w_change, w_free, clear_cap
        self.w_path, self.path_scale = w_path, path_scale
        s1 = np.linspace(-max_steer, max_steer, n_first)
        s2 = np.linspace(-max_steer, max_steer, n_second)
        n1, n2 = int(round(first / spacing)), int(round(second / spacing))
        shapes, firsts = [], []
        for a in s1:
            head = arc(math.tan(a) / wheelbase, first, n1)
            x, y, yaw = head[-1]
            c, s = math.cos(yaw), math.sin(yaw)
            for b in s2:
                tail = arc(math.tan(b) / wheelbase, second, n2)
                shapes.append(np.vstack([head, np.c_[x + c * tail[:, 0] - s * tail[:, 1],
                                                     y + s * tail[:, 0] + c * tail[:, 1], yaw + tail[:, 2]]]))
                firsts.append(a)
        self.shapes = np.stack(shapes)            # (M, n, 3)
        self.firsts = np.array(firsts)
        self.M, self.n = self.shapes.shape[:2]
        flat = self.shapes.reshape(-1, 3)
        self.px, self.py = flat[:, 0, None], flat[:, 1, None]
        self.pc, self.ps = np.cos(flat[:, 2])[:, None], np.sin(flat[:, 2])[:, None]
        self.along = self.across = None
        self.stats = dict(calls=0, none=0, keep_on=0, keep_cut=0, keep_all=0)
        # For the closing test only: the car turns about its rear axle, rear_lr behind base_link.
        self.keep_gap, self.keep_half, self.keep_tol = keep_gap, keep_half, keep_tol
        self.first_steers, self.first_of = np.unique(self.firsts, return_inverse=True)
        self.keep_poses = []
        for a in self.first_steers:
            h = arc(math.tan(a) / wheelbase, first, n1)
            self.keep_poses.append(np.c_[h[:, 0] - rear_lr + rear_lr * np.cos(h[:, 2]), h[:, 1] + rear_lr * np.sin(h[:, 2]), h[:, 2]])
        self.first_ok = None

    def see(self, pts):
        """Each point's distance along and across every pose. Once per cycle."""
        near = pts[(np.abs(pts[:, 0]) < 2.0) & (np.abs(pts[:, 1]) < 2.0)] if len(pts) else pts
        self.first_ok = self.first_gap = None
        if not len(near):
            self.along = self.across = None
            return
        if self.keep_gap > 0.0:
            hl, hw = self.keep_half
            gap = lambda q: np.hypot(np.maximum(np.abs(np.cos(q[:, 2])[:, None] * (near[None, :, 0] - q[:, 0, None])
                                                       + np.sin(q[:, 2])[:, None] * (near[None, :, 1] - q[:, 1, None])) - hl, 0.0),
                                     np.maximum(np.abs(-np.sin(q[:, 2])[:, None] * (near[None, :, 0] - q[:, 0, None])
                                                       + np.cos(q[:, 2])[:, None] * (near[None, :, 1] - q[:, 1, None])) - hw, 0.0)).min(axis=1)
            now_gap = float(gap(np.zeros((1, 3)))[0])
            if now_gap < self.keep_gap:
                self.first_gap = np.array([float(gap(q).min()) for q in self.keep_poses])
                self.first_ok = self.first_gap >= now_gap - self.keep_tol
        dx = near[None, :, 0] - self.px
        dy = near[None, :, 1] - self.py
        self.along = np.abs(self.pc * dx + self.ps * dy)
        self.across = np.abs(-self.ps * dx + self.pc * dy)

    def plan(self, target, target_yaw, steer0, half_length, half_width, route=None, path_gain=1.0):
        """(first steering, poses up to the first contact) of the best shape, or None."""
        self.stats['calls'] += 1
        M, n = self.M, self.n
        steps = np.arange(n)
        if self.along is None:
            free = np.full(M, n)
        else:
            hit = ((self.along <= half_length) & (self.across <= half_width)).any(axis=1).reshape(M, n)
            free = np.where(hit.any(axis=1), hit.argmax(axis=1), n)
        ok = free * self.spacing >= self.min_free
        if not ok.any():
            self.stats['none'] += 1
            return None
        if self.first_ok is not None:            # beside something: leave out arcs that close on it, if any other fits
            self.stats['keep_on'] += 1
            kept = ok & self.first_ok[self.first_of]
            if not kept.any():                   # every arc that fits closes on it: the ones that close least
                self.stats['keep_all'] += 1
                kept = ok & (self.first_gap[self.first_of] >= self.first_gap[self.first_of[ok]].max() - self.keep_tol)
            self.stats['keep_cut'] += int(ok.sum() - kept.sum())
            ok = kept
        idx = np.nonzero(ok)[0]
        free = free[idx]
        use = steps[None, :] < free[:, None]     # the clear part of each feasible shape
        low = np.full(len(idx), np.inf)
        if self.along is not None:
            flat = (idx[:, None] * n + steps[None, :])[use]
            ox = self.along[flat] - half_length
            oy = self.across[flat] - half_width
            ox[ox < 0.0] = 0.0
            oy[oy < 0.0] = 0.0
            clr = np.sqrt(np.min(ox * ox + oy * oy, axis=1))
            np.minimum.at(low, np.repeat(np.arange(len(idx)), free), clr)
        ends = self.shapes[idx, np.maximum(free - 1, 0), :]
        dist = np.hypot(target[0] - ends[:, 0], target[1] - ends[:, 1]) / max(1e-3, math.hypot(*target))
        dyaw = np.abs((target_yaw - ends[:, 2] + math.pi) % (2.0 * math.pi) - math.pi) / math.pi
        score = (-self.w_target * dist - self.w_yaw * dyaw
                 + self.w_clear * np.minimum(low, self.clear_cap) / self.clear_cap
                 + self.w_free * free / n
                 - self.w_change * np.abs(self.firsts[idx] - steer0) / self.max_steer)
        if route is not None and len(route):       # stay on the route unless something is in the way
            d = np.hypot(ends[:, 0, None] - route[None, :, 0], ends[:, 1, None] - route[None, :, 1]).min(axis=1)
            score -= path_gain * self.w_path * np.minimum(d, 3.0 * self.path_scale) / self.path_scale
        i = int(score.argmax())
        return float(self.firsts[idx[i]]), self.shapes[idx[i], :int(free[i])]


class DWAPlanner:
    STEP = 0.05

    def __init__(self, path_xy, params=None):
        self.p = params or default_params()
        self.ref, self.length = resample_closed(path_xy, self.STEP)
        self.n = len(self.ref)
        self.ds = self.length / self.n
        nxt = np.roll(self.ref, -1, axis=0)
        prv = np.roll(self.ref, 1, axis=0)
        self.theta = np.arctan2(nxt[:, 1] - prv[:, 1], nxt[:, 0] - prv[:, 0])
        self.normal = np.c_[-np.sin(self.theta), np.cos(self.theta)]   # left
        self.idx = None
        self.mode = 'FOLLOW'
        self.last_blocked = -1e9
        self.steer = 0.0
        self.last_rad = self.p.r_robot
        self.min_rad = self.p.r_robot           # smallest radius used since last read (log only)
        self.radii = list(dict.fromkeys(max(self.p.r_min, f * self.p.r_robot) for f in self.p.radius_factors))
        # Shapes stay within 0.9 of the physical curvature.
        self.dwa = TwoStageDWA(WHEELBASE, math.atan(0.9 * MAX_CURV * WHEELBASE),
                               w_clear=self.p.dwa_w_clear, clear_cap=self.p.dwa_clear_cap,
                               keep_gap=float(self.p.dwa_keep_gap))
        self.none_since = None
        self.none_xy = (1e9, 1e9)

    def reset(self):
        self.mode = 'FOLLOW'
        self.steer = 0.0

    def nearest(self, pos):
        if self.idx is not None:
            ids = (self.idx + np.arange(-20, 140)) % self.n
            d = np.hypot(*(self.ref[ids] - pos).T)
            if d.min() < 1.5:
                self.idx = int(ids[np.argmin(d)])
                return self.idx
        self.idx = int(np.argmin(np.hypot(*(self.ref - pos).T)))
        return self.idx

    def ahead(self, idx, length):
        return self.ref[(idx + np.arange(int(length / self.ds))) % self.n]

    def rect(self, rad):
        margin = max(0.0, rad - self.p.footprint_width / 2.0)
        return self.p.footprint_length / 2.0 + margin, self.p.footprint_width / 2.0 + margin

    def narrow_speed(self, b, pts):
        """Speed limit from the bare footprint's clearance along the path ahead, braking in time."""
        p = self.p
        look = np.vstack([np.zeros((1, 3)), b[:max(2, int(p.narrow_look / NARROW_SPACING))]])
        if not len(pts):
            return p.avoid_max_speed
        hl, hw = self.rect(0.0)
        d = NARROW_SPACING * np.arange(len(look))
        limit = closing_speed_limit(look, pts, hl, hw, p.narrow_keep, p.narrow_stop_time, p.narrow_drift_time)
        limit = np.clip(limit, p.narrow_min_speed, p.avoid_max_speed)
        return float(np.sqrt(limit ** 2 + 2.0 * p.narrow_decel * d).min())

    def step(self, pose, pts, pp_v, pp_steer, now, unseen=None, odom_pose=None):
        """pose = (x, y, yaw) in map, pts = Nx2 scan points in base_link. Returns (speed, steer, info)."""
        p = self.p
        odom = pose if odom_pose is None else odom_pose
        x, y, yaw = pose
        idx = self.nearest(np.array([x, y]))
        off0 = float((np.array([x, y]) - self.ref[idx]) @ self.normal[idx])  # +left

        if len(pts):
            rr = np.hypot(pts[:, 0], pts[:, 1])
            pts = pts[(rr < p.plan_range) & (rr > p.body_r)]
            if len(pts) > 220:                  # keep the cycle fast
                pts = pts[::int(math.ceil(len(pts) / 220.0))]
        if unseen is not None and len(unseen):  # after thinning: every remembered point counts
            unseen = unseen[np.hypot(unseen[:, 0], unseen[:, 1]) > p.body_r]
            pts = np.vstack([pts, unseen]) if len(pts) else unseen
        stop_box = np.zeros(len(pts), dtype=bool)
        if len(pts):                           # emergency stop box in front of the car
            stop_box = (pts[:, 0] > 0.05) & (pts[:, 0] < p.stop_front) & (np.abs(pts[:, 1]) < p.stop_half)
        ahead_b = to_base(self.ahead(idx, p.detect_dist + 1.5), pose)
        det_b = ahead_b[:int(p.detect_dist / self.ds)]

        near = np.zeros(len(pts), bool)
        near_s = np.full(len(pts), p.detect_dist, dtype=float)
        route_gap = math.inf
        if len(pts) and len(det_b):
            d2 = ((pts[:, None, :] - det_b[None, :, :]) ** 2).sum(-1)
            near = d2.min(axis=1) < p.corridor ** 2
            near_s = d2.argmin(axis=1) * self.ds
            route_gap = float(np.sqrt(d2.min()))     # how close the route itself comes to any point
        near |= stop_box
        block_s = float(near_s[near].min()) if near.any() else None
        block_far = float(near_s[near].max()) if near.any() else None
        # wall speed limit: clearance between the scan and the route ahead
        cap = p.max_speed
        if len(pts):
            sl = ahead_b[:max(2, int(p.slow_dist / self.ds)):2]
            clr = float(np.sqrt(((pts[:, None, :] - sl[None, :, :]) ** 2).sum(-1).min()))
            clr = min(clr, float(np.hypot(pts[:, 0], pts[:, 1]).min()) - p.r_robot + p.wall_near)
            f = (clr - p.wall_near) / max(1e-6, p.wall_far - p.wall_near)
            cap = p.wall_speed + float(np.clip(f, 0.0, 1.0)) * (p.max_speed - p.wall_speed)
        # corner speed limit from the route's curvature ahead and the current steering
        w = 10                                  # 0.5 m windows
        ids = (idx + np.arange(0, int(p.curve_look / self.ds))) % self.n
        dth = np.abs(np.array([wrap(self.theta[(i + w) % self.n] - self.theta[i]) for i in ids[::5]]))
        kmax = float(dth.max()) / (w * self.ds) if len(dth) else 0.0
        v_curve = math.sqrt(p.a_lat / max(kmax, 1e-3))
        v_steer = p.max_speed - (p.max_speed - p.curve_min) * min(1.0, abs(pp_steer) / p.steer_slow)
        cap = min(cap, max(p.curve_min, min(v_curve, v_steer)))
        blocked = bool(near.any())
        if blocked:
            self.last_blocked = now
        clusters = clusters_of(pts[near], p.cluster_gap) if blocked else []

        if self.mode == 'FOLLOW':
            if blocked or abs(off0) > p.recover_off:
                self.mode = 'AVOID'
        elif not blocked and now - self.last_blocked > p.clear_hold and abs(off0) < p.return_off:
            self.mode = 'FOLLOW'

        info = dict(mode=self.mode, blocked=blocked, clusters=clusters, cands=[], best=None, off=off0, cap=cap)
        if self.mode == 'FOLLOW':
            self.steer = float(np.clip(pp_steer, -MAX_STEER, MAX_STEER))
            return float(min(pp_v, cap)), self.steer, info

        v = float(np.clip(pp_v, p.min_speed, p.free_speed if not blocked else p.avoid_max_speed))
        v = min(v, max(cap, p.wall_speed))
        dist_t = p.dwa_target_dist
        path_gain = 1.0
        # The route runs through something: turn off it in time. Merely passing close to
        # something, it stays the best line and gets centred by the clearance term.
        if blocked and block_s is not None and route_gap < p.dwa_through:
            path_gain = float(np.clip((block_s - p.dwa_turn_near) / (p.dwa_turn_far - p.dwa_turn_near),
                                      p.dwa_path_min, 1.0))
            if block_far is not None and path_gain < 1.0:
                dist_t = max(dist_t, block_far + p.dwa_past_block)
        if self.none_since is not None and now - self.none_since < p.dwa_retry_wait \
                and math.hypot(odom[0] - self.none_xy[0], odom[1] - self.none_xy[1]) < 0.01:
            info['stuck'] = True
            return 0.0, self.steer, info
        i_t = min(len(ahead_b) - 1, int(dist_t / self.ds))
        j_t = (idx + i_t) % self.n
        self.dwa.see(pts)
        res = None
        for rad in self.radii:
            res = self.dwa.plan(ahead_b[i_t], self.theta[j_t] - yaw, self.steer, *self.rect(rad),
                                route=ahead_b[::2], path_gain=path_gain)
            if res is not None:
                self.last_rad = rad
                self.min_rad = min(self.min_rad, rad)
                break
        if res is None:
            info['stuck'] = True                 # no free shape: stop, no creep
            self.none_since, self.none_xy = now, odom[:2]
            return 0.0, self.steer, info
        self.none_since = None
        vd = min(v, self.narrow_speed(res[1], pts))
        # The same corner rules as elsewhere, on the steering actually commanded.
        k_cmd = abs(math.tan(res[0])) / WHEELBASE
        vd = min(vd, max(p.curve_min, math.sqrt(p.a_lat / max(k_cmd, 1e-3))))
        vd = min(vd, p.max_speed - (p.max_speed - p.curve_min) * min(1.0, abs(res[0]) / p.steer_slow))
        if self.last_rad < 0.9 * p.r_robot:
            vd = min(vd, p.squeeze_speed)
        info['cands'], info['best'] = [res[1][:, :2]], res[1][:, :2]
        self.steer = res[0]
        return vd, self.steer, info


try:
    import rclpy
    from ackermann_msgs.msg import AckermannDriveStamped
    from geometry_msgs.msg import Point, PoseStamped
    from nav_msgs.msg import Path
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool
    from tf2_ros import Buffer, TransformListener
    from visualization_msgs.msg import Marker, MarkerArray
    HAVE_ROS = True
except ImportError:          # lets the algorithm be tested without ROS
    HAVE_ROS = False


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))



if HAVE_ROS:
    class DWANode(Node):
        def __init__(self):
            super().__init__('frenet_local_planner')
            self.p = default_params()
            for k, v in vars(self.p).items():
                self.declare_parameter(k, list(v) if isinstance(v, tuple) else v)
                val = self.get_parameter(k).value
                setattr(self.p, k, tuple(val) if isinstance(v, tuple) else val)
            self.declare_parameter('scan_topic', '/scan')
            self.declare_parameter('pp_cmd_topic', '/pure_pursuit/ackermann_cmd')
            self.declare_parameter('out_topic', '/final_ackermann_cmd')
            self.declare_parameter('path_topic', '/pp_path')
            self.declare_parameter('map_frame', 'map')
            self.declare_parameter('base_frame', 'base_link')
            self.declare_parameter('odom_frame', 'odom')
            self.declare_parameter('cmd_timeout', 0.5)
            g = self.get_parameter
            self.map_frame = g('map_frame').value
            self.base_frame = g('base_frame').value
            self.odom_frame = g('odom_frame').value
            self.cmd_timeout = g('cmd_timeout').value

            self.planner = None
            self.scan = None
            self.pp = None                     # (speed, steer, time)
            self.last_zero = -1e9
            self.tf_buffer = Buffer()
            # own thread for /tf: a long planning cycle must not starve the TF buffer
            self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
            self._step_ms = []
            self.memory = UnseenMemory(self.p.memory_hold, self.p.memory_keep_r, self.p.memory_cell,
                                       self.p.memory_rmin_margin)
            self._mem_stamp = None
            self._mem_n = 0
            self._pts_stamp = None
            self._pts = None

            self.create_subscription(Path, g('path_topic').value, self.on_path, 1)
            self.create_subscription(LaserScan, g('scan_topic').value,
                                     self.on_scan, qos_profile_sensor_data)
            self.create_subscription(AckermannDriveStamped,
                                     g('pp_cmd_topic').value, self.on_pp, 1)
            self.pub_cmd = self.create_publisher(
                AckermannDriveStamped, g('out_topic').value, 1)
            self.pub_obs = self.create_publisher(MarkerArray, '/obstacles', 1)
            self.pub_path = self.create_publisher(Path, '/local_path', 1)
            self.pub_cand = self.create_publisher(MarkerArray, '/local_candidates', 1)
            self.pub_flag = self.create_publisher(Bool, '/obstacle_detected', 1)
            self.create_timer(0.05, self.tick)
            self.get_logger().info(
                f"dwa planner: {g('pp_cmd_topic').value} -> {g('out_topic').value}")

        # -- inputs ---------------------------------------------------------
        def now_s(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def on_path(self, msg):
            xy = np.array([[q.pose.position.x, q.pose.position.y] for q in msg.poses])
            if len(xy) < 8:
                return
            if self.planner is not None and len(self.planner.ref) and \
                    getattr(self, '_sig', None) == (len(xy), float(xy[0, 0]), float(xy[-1, 1])):
                return
            self._sig = (len(xy), float(xy[0, 0]), float(xy[-1, 1]))
            self.planner = DWAPlanner(xy, self.p)
            self.get_logger().info(f'global path: {len(xy)} pts, '
                                   f'{self.planner.length:.1f} m')

        def on_scan(self, msg):
            self.scan = msg

        def on_pp(self, msg):
            self.pp = (msg.drive.speed, msg.drive.steering_angle, self.now_s())

        def pose(self):
            try:
                t = self.tf_buffer.lookup_transform(
                    self.map_frame, self.base_frame, Time(), Duration(seconds=0.05))
            except Exception:                  # noqa: BLE001
                return None
            return (t.transform.translation.x, t.transform.translation.y,
                    yaw_of(t.transform.rotation))

        def scan_points(self):
            s = self.scan
            if s is None:
                return np.zeros((0, 2))
            stamp = (s.header.stamp.sec, s.header.stamp.nanosec)
            if stamp == self._pts_stamp:       # same scan as last cycle
                return self._pts
            tf = (0.0, 0.0, 0.0)
            try:
                t = self.tf_buffer.lookup_transform(
                    self.base_frame, s.header.frame_id, Time(), Duration(seconds=0.05))
                tf = (t.transform.translation.x, t.transform.translation.y,
                      yaw_of(t.transform.rotation))
                self._laser_tf = tf
            except Exception:                  # noqa: BLE001
                if self.get_clock().now().nanoseconds * 1e-9 - getattr(self, '_tfw', -1e9) > 3.0:
                    self._tfw = self.get_clock().now().nanoseconds * 1e-9
                    self.get_logger().warn(
                        f'no TF {self.base_frame} <- {s.header.frame_id}: scan used WITHOUT mounting offset')
            self._pts_stamp = stamp
            self._pts = scan_to_points(s.ranges, s.angle_min, s.angle_increment,
                                       s.range_min, s.range_max, self.p.plan_range, tf)
            return self._pts

        def odom_pose(self, stamp=None):
            t = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, Time() if stamp is None else Time.from_msg(stamp))
            return (t.transform.translation.x, t.transform.translation.y,
                    yaw_of(t.transform.rotation))


        def with_unseen(self, pts, now):
            """Remembered points the sensor cannot confirm. Odometry frame: AMCL jumps must
            not move them. Without odometry or the laser mount the memory is dropped."""
            self._mem_n = 0
            none = np.zeros((0, 2))
            if not self.p.use_unseen_memory:
                return none
            laser = getattr(self, '_laser_tf', None)
            try:
                if laser is None:
                    raise ValueError('laser mount unknown')
                pose_now = self.odom_pose()
                s = self.scan
                if self._mem_stamp != (s.header.stamp.sec, s.header.stamp.nanosec):
                    self._mem_stamp = (s.header.stamp.sec, s.header.stamp.nanosec)
                    try:
                        pose_scan = self.odom_pose(s.header.stamp)
                    except Exception:  # noqa: BLE001
                        pose_scan = pose_now
                    self.memory.add(now, pts, pose_scan)
                extra = self.memory.unseen(now, pose_now, laser[:2], s.range_min, s.angle_max)
            except Exception as e:  # noqa: BLE001
                self.memory.clear()
                if now - getattr(self, '_mem_warn', -1e9) > 3.0:
                    self._mem_warn = now
                    self.get_logger().warn(f'unseen-point memory off: {e}')
                return none
            self._mem_n = len(extra)
            return extra

        # -- main loop ------------------------------------------------------
        def tick(self):
            t = self.now_s()
            pp = self.pp
            if self.planner is None:           # no global path yet: pass through
                if pp is not None and t - pp[2] <= self.cmd_timeout:
                    self.send(pp[0], pp[1])
                    if t - getattr(self, '_warn_t', -1e9) > 3.0:
                        self._warn_t = t
                        self.get_logger().warn(
                            'no /pp_path received yet -> passing Pure Pursuit through')
                return
            if pp is None or t - pp[2] > self.cmd_timeout:      # PP not running
                self.planner.reset()
                self.memory.clear()               # the car may be moved while stopped
                if t - self.last_zero > 1.0:
                    self.send(0.0, 0.0)
                    self.last_zero = t
                return
            pose = self.pose()
            sc = self.scan
            stale = (sc is None or
                     t - (sc.header.stamp.sec + sc.header.stamp.nanosec * 1e-9) > 0.5)
            if pose is None or stale:          # blind -> stop (fail safe)
                self.send(0.0, 0.0)
                if t - getattr(self, '_warn2_t', -1e9) > 2.0:
                    self._warn2_t = t
                    self.get_logger().warn(
                        'no TF map->base_link' if pose is None else 'no fresh /scan')
                return
            t_step = time.perf_counter()
            pts = self.scan_points()
            try:
                odom = self.odom_pose()
            except Exception:                  # noqa: BLE001
                odom = None
            v, steer, info = self.planner.step(
                pose, pts, pp[0], pp[1], t, self.with_unseen(pts, t), odom)
            self._step_ms.append((time.perf_counter() - t_step) * 1e3)
            self.send(v, steer)
            if t - getattr(self, '_log_t', -1e9) > 1.0:
                last = getattr(self, '_last_pose', None)
                meas = ''
                if last is not None and t > last[2]:
                    dx, dy = pose[0] - last[0], pose[1] - last[1]
                    mv = (dx * math.cos(pose[2]) + dy * math.sin(pose[2])) / (t - last[2])
                    meas = f' real_v={mv:.2f}'
                    speed_mismatch = abs(mv) > 2.0 * abs(v) + 0.25
                    direction_mismatch = abs(mv) > 0.08 and abs(v) > 0.05 and mv * v < 0.0
                    if speed_mismatch or direction_mismatch:
                        self.get_logger().warn(
                            f'odometry speed={mv:.2f} m/s disagrees with out_v={v:.2f}; '
                            'check command routing and controller response')
                self._last_pose = (pose[0], pose[1], t)
                self._log_t = t
                tick = ''
                if self._step_ms:
                    tick = (f" tick_ms={sum(self._step_ms) / len(self._step_ms):.0f}"
                            f"/{max(self._step_ms):.0f}(avg/max)")
                    if max(self._step_ms) > 100.0:
                        self.get_logger().warn(
                            f'planning cycle took {max(self._step_ms):.0f} ms (> 100 ms): '
                            'scan/TF may be stale; reduce n_lat / horizons if this repeats')
                    self._step_ms = []
                rj = info.get('rej', {})
                why = ''
                if info.get('blocked'):
                    why = (f" blk n_ok={len(info['cands'])} rej(curv/back/coll)="
                           f"{rj.get('curv', 0)}/{rj.get('back', 0)}/{rj.get('coll', 0)}"
                           f" fb={info.get('fallback', 0)}"
                           f"{' STUCK' if info.get('stuck') else ''}")
                elif info.get('stuck'):              # the monitor must see a stop off the route too
                    why = ' STUCK'
                why += f' mem={self._mem_n} min_rad={self.planner.min_rad:.3f}'
                self.planner.min_rad = self.p.r_robot
                self.get_logger().info(
                    f"mode={info['mode']} off={info['off']:+.2f} "
                    f"cap={info['cap']:.2f} pp_v={pp[0]:.2f} out_v={v:.2f}{meas}{tick}{why}")
            if t - getattr(self, '_chk_t', -1e9) > 5.0:
                self._chk_t = t
                n = self.count_subscribers(self.get_parameter('pp_cmd_topic').value)
                if n != 1:
                    self.get_logger().warn(
                        f'{n} nodes subscribe to the Pure Pursuit command topic '
                        '(expected only this planner): a bridge bypasses the planner')
            self.visualize(pose, info)

        def send(self, v, steer):
            m = AckermannDriveStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = self.base_frame
            m.drive.speed = float(v)
            m.drive.steering_angle = float(steer)
            self.pub_cmd.publish(m)

        # -- RViz -----------------------------------------------------------
        def marker(self, mid, ns, mtype, rgba, scale):
            mk = Marker()
            mk.header.frame_id = self.map_frame
            mk.header.stamp = self.get_clock().now().to_msg()
            mk.ns, mk.id, mk.type, mk.action = ns, mid, mtype, Marker.ADD
            mk.pose.orientation.w = 1.0
            mk.scale.x, mk.scale.y, mk.scale.z = scale
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = rgba
            mk.lifetime = Duration(seconds=0.3).to_msg()
            return mk

        def visualize(self, pose, info):
            self.pub_flag.publish(Bool(data=bool(info['blocked'])))
            obs = MarkerArray()
            for i, (c, n, size) in enumerate(info['clusters']):
                mk = self.marker(i, 'obstacle', Marker.CYLINDER,
                                 (1.0, 0.1, 0.1, 0.8), (max(size, 0.15), max(size, 0.15), 0.3))
                cm = to_map(c[None, :], pose)[0]
                mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = \
                    float(cm[0]), float(cm[1]), 0.15
                obs.markers.append(mk)
            self.pub_obs.publish(obs)

            arr = MarkerArray()
            now_v = self.now_s()
            if info['cands'] and now_v - getattr(self, '_cand_t', -1e9) < 0.25:
                info = dict(info, cands=None)            # RViz only: refresh candidates at 4 Hz
            else:
                self._cand_t = now_v
            if info['cands'] is None:
                pass
            elif info['cands']:
                lines = self.marker(0, 'candidates', Marker.LINE_LIST,
                                    (0.2, 0.8, 0.3, 0.35), (0.01, 0.0, 0.0))
                for b in info['cands']:
                    mp = to_map(b, pose)
                    for a, c in zip(mp[:-1:2], mp[1::2]):
                        lines.points.append(Point(x=float(a[0]), y=float(a[1]), z=0.05))
                        lines.points.append(Point(x=float(c[0]), y=float(c[1]), z=0.05))
                arr.markers.append(lines)
            if info['cands'] is not None:
                self.pub_cand.publish(arr)

            path = Path()
            path.header.frame_id = self.map_frame
            path.header.stamp = self.get_clock().now().to_msg()
            if info['best'] is not None:
                for q in to_map(info['best'], pose):
                    ps = PoseStamped()
                    ps.header = path.header
                    ps.pose.position.x, ps.pose.position.y = float(q[0]), float(q[1])
                    ps.pose.orientation.w = 1.0
                    path.poses.append(ps)
            self.pub_path.publish(path)
            if info['mode'] != getattr(self, '_last_mode', None):
                self._last_mode = info['mode']
                self.get_logger().info(f"mode -> {info['mode']}")


def main():
    if not HAVE_ROS:
        raise SystemExit('rclpy not found: source /opt/ros/humble/setup.bash')
    rclpy.init()
    node = DWANode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.send(0.0, 0.0)
        except Exception:                      # noqa: BLE001  (context already shut down)
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
