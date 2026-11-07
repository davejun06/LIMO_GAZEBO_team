#!/usr/bin/env python3
"""Measure one run: telemetry, lap times, safety aborts. Sends only the start/stop selector.

Ground truth is /odom: the Gazebo plugin reports the model's world pose (checked against
`gz model -p`). Contacts come from Gazebo itself. All times are simulation seconds.

Stops the run (selector 'stop') on: non-floor contact, stall, repeated missing input,
localization error above the gate, command/odometry mismatch, timeout, or laps done.
"""
import argparse
import csv
import json
import math
import os
import re
import subprocess
import threading
import time

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from rcl_interfaces.msg import Log
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

CONTACT_TOPIC = '/gazebo/ifac_roboracer/physics/contacts'
PAIR = re.compile(r'collision1: "([^"]+)" collision2: "([^"]+)"')
# centre x, centre y, half x, half y: from the world file
OBSTACLES = {'o1': (5.0, -0.087847578, 0.15, 0.4), 'o2': (-3.0, 3.054242265, 0.15, 0.4), 'o3': (-8.0, 1.145160234, 0.15, 0.5)}
FOOT_HALF = (0.165, 0.155)  # planner footprint_length/2, footprint_width/2
STOP_LATENCY = 0.7   # [s] selector to odometry deceleration: estimate from exp-006b, see changelog


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def box_clearance(x, y, yaw, box):
    """Exact gap between the rotated car footprint and an axis-aligned box [m], 0 if they overlap.

    Two disjoint convex polygons have a closest pair that includes a vertex, so checking the
    corners of each against the other is exact; overlap is found by separating axes.
    """
    cx, cy, hx, hy = box
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = cx - x, cy - y
    # Separating axes: the box's x and y, the car's x and y.
    ex = FOOT_HALF[0] * abs(c) + FOOT_HALF[1] * abs(s)
    ey = FOOT_HALF[0] * abs(s) + FOOT_HALF[1] * abs(c)
    bu = hx * abs(c) + hy * abs(s)
    bv = hx * abs(s) + hy * abs(c)
    u, v = c * dx + s * dy, -s * dx + c * dy
    if (abs(dx) <= hx + ex and abs(dy) <= hy + ey
            and abs(u) <= FOOT_HALF[0] + bu and abs(v) <= FOOT_HALF[1] + bv):
        return 0.0
    best = math.inf
    for a in (-FOOT_HALF[0], FOOT_HALF[0]):      # car corners against the box
        for b in (-FOOT_HALF[1], FOOT_HALF[1]):
            px, py = x + c * a - s * b, y + s * a + c * b
            best = min(best, math.hypot(max(abs(px - cx) - hx, 0.0), max(abs(py - cy) - hy, 0.0)))
    for a in (-hx, hx):                          # box corners against the car
        for b in (-hy, hy):
            qx, qy = cx + a - x, cy + b - y
            lu, lv = c * qx + s * qy, -s * qx + c * qy
            best = min(best, math.hypot(max(abs(lu) - FOOT_HALF[0], 0.0),
                                        max(abs(lv) - FOOT_HALF[1], 0.0)))
    return best


class Monitor(Node):
    def __init__(self, a):
        super().__init__('run_monitor', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.a = a
        xy = np.loadtxt(a.path, delimiter=',')[:, :2]
        seg = np.hypot(*(np.roll(xy, -1, 0) - xy).T)
        self.xy, self.s, self.L = xy, np.r_[0, np.cumsum(seg)[:-1]], float(seg.sum())
        self.odom = self.amcl = self.final = self.pp = self.scan = None
        self.scan_t = -1e9
        self.final_t = None
        self.pp_count = self.final_count = 0
        self.gap_hist = {}
        self.emergency_pub = None
        self.scan_wall = []
        self.wall0 = self.sim0 = None
        self.flag = False
        self.contacts = []
        self.contact_alive = None
        self.stuck_run = self.crawl_run = self.blind_run = 0
        self.slow_run = 0
        self.slow_seen = False
        self.stuck_total = self.crawl_total = 0
        os.makedirs(os.path.dirname(a.out), exist_ok=True)
        self.plog = open(a.out.replace('.csv', '.planner.log'), 'w')
        self.stop_reason = None
        self.t_go = self.t_move = None
        self.laps = []
        self.dist = 0.0
        self.last_xy = None
        self.lap_ref = 0.0
        self.max_ld = 0.0
        self.stalls = 0
        self.in_stall = False
        self.stall_since = None
        self.mismatch_since = None
        self.blind_since = None
        self.min_scan = math.inf
        self.min_obs = {k: math.inf for k in OBSTACLES}
        self.max_loc = self.max_v = self.max_cmd = self.max_steer = self.max_dev = 0.0
        self.loc_samples = []
        self.tf = Buffer()
        self.tl = TransformListener(self.tf, self, spin_thread=True)
        self.sel = self.create_publisher(String, '/control_selector', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(AckermannDriveStamped, '/final_ackermann_cmd',
                                 self.on_final, 10)
        self.create_subscription(AckermannDriveStamped, '/pure_pursuit/ackermann_cmd',
                                 self.on_pp, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Log, '/rosout', self.on_log, 100)
        self.create_subscription(Bool, '/obstacle_detected',
                                 lambda m: setattr(self, 'flag', m.data), 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            lambda m: setattr(self, 'amcl', m),
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.f = open(a.out, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['wall', 'scan_hz', 't', 'x', 'y', 'yaw', 'v', 'map_x', 'map_y', 'map_yaw', 'loc_err',
                         'amcl_stamp', 'cov_x', 'cov_y', 'cov_yaw', 'cmd_v', 'cmd_steer',
                         'pp_v', 'pp_steer', 'scan_min', 'obstacle', 'progress', 'dev'])
        threading.Thread(target=self.watch_contacts, daemon=True).start()
        self.create_timer(0.05, self.tick)
        self.create_timer(0.5, self.selector)

    def on_pp(self, m):
        self.pp = m.drive
        if self.wall0 is not None:
            self.pp_count += 1

    def on_final(self, m):
        self.final = m.drive
        self.final_t = self.now()
        if self.wall0 is not None:
            self.final_count += 1

    def on_odom(self, m):
        self.odom = m

    def on_scan(self, m):
        self.scan = m
        self.scan_t = self.now()
        self.scan_wall.append(time.time())
        self.scan_wall = self.scan_wall[-40:]

    def on_log(self, m):
        """Planner status lines carry what no topic does: STUCK, CRAWL, blind warnings."""
        if 'frenet' not in m.name:
            return
        self.plog.write(f'{self.now():.2f} {m.msg}\n')
        self.plog.flush()
        if self.t_move is None:
            return
        if 'unseen-point memory off' in m.msg:
            self.request_stop('planner lost its unseen-point memory: ' + m.msg[:60])
        elif 'no TF' in m.msg or 'no fresh /scan' in m.msg:
            self.blind_run += 1
            if self.blind_run >= 3:
                self.request_stop('repeated planner input warning: ' + m.msg[:40])
        elif 'planning cycle took' in m.msg:
            self.slow_run += 1
            self.slow_seen = True
            if self.slow_run >= self.a.slow_abort:
                self.request_stop(f'planner cycle over 100 ms for {self.slow_run} status lines in a row')
        elif m.msg.startswith('mode='):
            self.blind_run = 0
            if not self.slow_seen:
                self.slow_run = 0
            self.slow_seen = False
            stuck = 'STUCK' in m.msg
            crawl = m.msg.startswith('mode=CRAWL')
            self.stuck_run = self.stuck_run + 1 if stuck else 0
            self.crawl_run = self.crawl_run + 1 if crawl else 0
            self.stuck_total += stuck
            self.crawl_total += crawl
            if self.stuck_run >= self.a.stuck_abort:
                self.request_stop(f'no safe candidate for {self.stuck_run} status lines')
            if self.crawl_run >= self.a.crawl_abort:
                self.request_stop(f'CRAWL repeated for {self.crawl_run} status lines')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def watch_contacts(self):
        p = subprocess.Popen(['gz', 'topic', '-u', '-e', CONTACT_TOPIC], stdout=subprocess.PIPE,
                             stderr=subprocess.DEVNULL, text=True, errors='replace')
        for line in p.stdout:
            for c1, c2 in PAIR.findall(line):
                if 'limo::' not in c1 + c2:
                    continue
                other = c2 if c1.startswith('limo::') else c1
                if '::floor' in other:
                    # Wheels always touch the floor: proof the parser sees real contacts.
                    self.contact_alive = self.now()
                    continue
                if other.startswith('limo::'):
                    continue
                self.contacts.append((self.now(), c1, c2))
                self.request_stop('contact ' + other)

    def request_stop(self, why):
        if self.stop_reason is None:
            self.stop_reason = why
            self.get_logger().warn('STOP: ' + why)
            msg = String()
            msg.data = 'stop'
            self.sel.publish(msg)        # now, not at the next 2 Hz selector tick

    def emergency_zero(self):
        """Last resort when the planner or bridge has died: nothing else would send zero.

        Created only on failure so that /cmd_vel has one publisher in normal running.
        """
        if self.emergency_pub is None:
            self.get_logger().error('EMERGENCY: publishing zero on /cmd_vel')
            self.emergency_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.emergency_pub.publish(Twist())

    def scan_hz(self):
        w = self.scan_wall
        return (len(w) - 1) / (w[-1] - w[0]) if len(w) > 1 and w[-1] > w[0] else float('nan')

    def selector(self):
        """Short start signal, resent only until motion is seen; stop is sent while finishing."""
        msg = String()
        if self.stop_reason is not None:
            msg.data = 'stop'
            self.sel.publish(msg)
        elif self.t_go is not None and self.t_move is None:
            msg.data = 'pure_pursuit'
            self.sel.publish(msg)

    def tick(self):
        t = self.now()
        if self.odom is None or t <= 0.0:
            return
        if self.t_go is None and self.contact_alive is None:
            return  # do not open the gate until the contact stream is proven alive
        if self.t_go is None and self.sel.get_subscription_count() >= 1:
            self.t_go = t
            self.t_end = t + self.a.timeout
            self.get_logger().info(f'start signal at sim t={t:.2f}')
        o = self.odom
        x, y = o.pose.pose.position.x, o.pose.pose.position.y
        yaw = yaw_of(o.pose.pose.orientation)
        v = o.twist.twist.linear.x
        try:
            tr = self.tf.lookup_transform('map', 'odom', Time())
            tx, ty = tr.transform.translation.x, tr.transform.translation.y
            ta = yaw_of(tr.transform.rotation)
            mx = tx + math.cos(ta) * x - math.sin(ta) * y
            my = ty + math.sin(ta) * x + math.cos(ta) * y
            myaw = ta + yaw
            have_tf = True
        except Exception:  # noqa: BLE001
            mx = my = myaw = float('nan')
            have_tf = False
        # map->odom only moves when AMCL updates, so freshness is judged on the base_link chain.
        try:
            tb = self.tf.lookup_transform('odom', 'base_link', Time())
            if abs(t - (tb.header.stamp.sec + tb.header.stamp.nanosec * 1e-9)) > 1.0:
                have_tf = False
        except Exception:  # noqa: BLE001
            have_tf = False
        d = np.hypot(*(self.xy - (x, y)).T)
        i = int(d.argmin())
        dev = float(d[i])
        err = math.hypot(mx - x, my - y) if have_tf else float('nan')
        scan_min = math.inf
        if self.scan is not None:
            r = np.asarray(self.scan.ranges)
            r = r[np.isfinite(r)]
            if len(r):
                scan_min = float(r.min())
        cmd_v = self.final.speed if self.final else 0.0
        cmd_s = self.final.steering_angle if self.final else 0.0
        amcl = self.amcl
        cov = amcl.pose.covariance if amcl else [float('nan')] * 36
        stamp = (amcl.header.stamp.sec + amcl.header.stamp.nanosec * 1e-9) if amcl else float('nan')
        self.w.writerow([f'{time.time():.3f}', f'{self.scan_hz():.2f}', f'{t:.3f}', f'{x:.4f}', f'{y:.4f}', f'{yaw:.4f}', f'{v:.3f}',
                         f'{mx:.4f}', f'{my:.4f}', f'{myaw:.4f}', f'{err:.4f}', f'{stamp:.3f}',
                         f'{cov[0]:.5f}', f'{cov[7]:.5f}', f'{cov[35]:.5f}',
                         f'{cmd_v:.3f}', f'{cmd_s:.3f}',
                         f'{self.pp.speed if self.pp else 0:.3f}',
                         f'{self.pp.steering_angle if self.pp else 0:.3f}',
                         f'{scan_min:.3f}', int(self.flag), f'{self.s[i]:.3f}', f'{dev:.3f}'])
        if self.t_go is None:
            return
        if self.t_move is None:
            if abs(v) > 0.03:
                self.t_move = t
                self.last_xy = (x, y)
                self.get_logger().info(f'motion at sim t={t:.2f} '
                                       f'({t - self.t_go:.2f} s after signal)')
            elif t - self.t_go > 15.0:
                self.request_stop('no motion 15 s after start signal')
            return
        if self.wall0 is None:
            self.wall0, self.sim0 = time.time(), t
        # safety bookkeeping
        silent = self.final_t is None or t - self.final_t > 2.0
        no_bridge = self.count_publishers('/cmd_vel') == 0
        # Once active, keep sending: our own publisher makes the count nonzero, and a single
        # message right after creating it would likely be lost to discovery.
        if silent or no_bridge or self.emergency_pub is not None:
            self.request_stop('planner or bridge lost' if no_bridge else
                              '/final_ackermann_cmd silent for 2 s')
            self.emergency_zero()
        if t - self.contact_alive > 3.0:
            self.request_stop('contact stream silent for 3 s: collision detection lost')
        if dev > self.a.dev_abort:
            self.request_stop(f'route deviation {dev:.2f} m at s={self.s[i]:.2f}')
        self.dist += math.hypot(x - self.last_xy[0], y - self.last_xy[1])
        self.last_xy = (x, y)
        self.max_v, self.max_cmd = max(self.max_v, abs(v)), max(self.max_cmd, abs(cmd_v))
        self.max_steer, self.max_dev = max(self.max_steer, abs(cmd_s)), max(self.max_dev, dev)
        self.min_scan = min(self.min_scan, scan_min)
        for k, c in OBSTACLES.items():
            g = box_clearance(x, y, yaw, c)
            self.min_obs[k] = min(self.min_obs[k], g)
            # Test safety only, not control: stop before the box if the gap keeps closing at
            # this rate through the stop latency. Counts as a failed run.
            prev = self.gap_hist.setdefault(k, [])
            prev.append((t, g))
            del prev[:-6]
            rate = (g - prev[0][1]) / max(t - prev[0][0], 1e-3) if len(prev) >= 2 else 0.0
            if g + min(rate, 0.0) * STOP_LATENCY < self.a.near_stop:
                self.request_stop(f'near contact {k}: gap {g:.3f} m, closing {-rate:.2f} m/s')
        if have_tf:
            yerr = abs(math.atan2(math.sin(myaw - yaw), math.cos(myaw - yaw)))
            self.max_loc = max(self.max_loc, err)
            self.loc_samples.append((err, yerr))
            if err > self.a.loc_abort:
                self.request_stop(f'localization error {err:.2f} m')
        # stall: speed commanded but no motion for 5 s, or zero speed for a long time
        stopped = abs(v) < 0.03
        if stopped and self.stall_since is None:
            self.stall_since = t
        if not stopped:
            self.stall_since = None
            self.in_stall = False
        if self.stall_since is not None and t - self.stall_since > 1.0 and not self.in_stall:
            self.stalls += 1
            self.in_stall = True
        if self.stall_since is not None and t - self.stall_since > self.a.stall_abort:
            self.request_stop(f'stalled {self.a.stall_abort:.0f} s at s={self.s[i]:.2f}')
        if abs(cmd_v) > 0.05 and abs(v) < 0.03:
            self.mismatch_since = self.mismatch_since or t
            if t - self.mismatch_since > 5.0:
                self.request_stop('command without motion for 5 s (possible contact)')
        else:
            self.mismatch_since = None
        if t - self.scan_t > 2.0 or not have_tf:
            self.blind_since = self.blind_since or t
            if t - self.blind_since > 3.0:
                self.request_stop('no fresh /scan or TF for 3 s')
        else:
            self.blind_since = None
        # lap: travelled most of the route and returned to the start point along the route
        if self.dist - self.lap_ref > 0.8 * self.L and self.s[i] < 3.0 and dev < 1.5:
            last = self.laps[-1][1] if self.laps else self.t_move
            self.laps.append((t - last, t))
            self.lap_ref = self.dist
            self.get_logger().info(f'lap {len(self.laps)}: {t - last:.2f} s')
            if len(self.laps) >= self.a.laps:
                self.request_stop('laps complete')
        if t > self.t_end:
            self.request_stop('timeout')

    def summary(self):
        rtf = pp_hz = final_hz = None
        if self.wall0 is not None and time.time() > self.wall0 + 1.0:
            wall = time.time() - self.wall0
            rtf = round((self.now() - self.sim0) / wall, 3)
            # Received message rates over the whole run, including the final seconds.
            pp_hz, final_hz = round(self.pp_count / wall, 1), round(self.final_count / wall, 1)
        p = np.array(self.loc_samples) if self.loc_samples else np.zeros((1, 2))
        return dict(
            exp=self.a.exp, rtf=rtf, pp_cmd_hz=pp_hz, final_cmd_hz=final_hz, laps=[round(x[0], 3) for x in self.laps],
            worst_lap=round(max(x[0] for x in self.laps), 3) if self.laps else None,
            stop_reason=self.stop_reason, contacts=len(self.contacts),
            first_contacts=self.contacts[:3], min_scan=round(self.min_scan, 3),
            min_obstacle_gap={k: round(v, 3) for k, v in self.min_obs.items()},
            stuck_lines=self.stuck_total, crawl_lines=self.crawl_total,
            stalls=self.stalls, distance=round(self.dist, 2),
            start_delay=None if self.t_move is None else round(self.t_move - self.t_go, 2),
            max_odom_v=round(self.max_v, 3), max_cmd_v=round(self.max_cmd, 3),
            max_steer=round(self.max_steer, 3), max_route_dev=round(self.max_dev, 3),
            loc_err_mean=round(float(p[:, 0].mean()), 3),
            loc_err_p95=round(float(np.percentile(p[:, 0], 95)), 3),
            loc_err_max=round(float(p[:, 0].max()), 3),
            yaw_err_p95=round(float(np.percentile(p[:, 1], 95)), 3))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--exp', required=True)
    ap.add_argument('--path', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--laps', type=int, default=1)
    ap.add_argument('--timeout', type=float, default=180.0)
    ap.add_argument('--stall-abort', type=float, default=12.0)
    ap.add_argument('--loc-abort', type=float, default=0.25)
    ap.add_argument('--dev-abort', type=float, default=1.3)
    ap.add_argument('--near-stop', type=float, default=0.02)
    ap.add_argument('--stuck-abort', type=int, default=5)
    ap.add_argument('--slow-abort', type=int, default=5)
    ap.add_argument('--crawl-abort', type=int, default=8)
    a = ap.parse_args()
    rclpy.init()
    n = Monitor(a)
    end = None
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.05)
            if n.stop_reason is not None:
                end = end or n.now()
                if n.now() - end > 3.0 and n.odom and abs(n.odom.twist.twist.linear.x) < 0.02:
                    break
                if n.now() - end > 15.0:
                    break
    finally:
        # Whatever ended the loop (crash included), close the gate before leaving.
        stop = String()
        stop.data = 'stop'
        try:
            for _ in range(5):
                n.sel.publish(stop)
                rclpy.spin_once(n, timeout_sec=0.1)
        except Exception as e:  # noqa: BLE001  (context gone after a signal: still save results)
            print('could not send final stop:', e, flush=True)
        n.f.close()
        n.plog.close()
        s = n.summary()
        s['contact_stream_ok'] = n.contact_alive is not None
        s['end_v'] = round(n.odom.twist.twist.linear.x, 4) if n.odom else None
        s['end_cmd'] = round(n.final.speed, 4) if n.final else None
        with open(a.out.replace('.csv', '.json'), 'w') as f:
            json.dump(s, f, indent=1)
        print(json.dumps(s, indent=1), flush=True)
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
