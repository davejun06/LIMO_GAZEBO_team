#!/usr/bin/env python3
"""Pre-run gate. Prints each check; exit code 1 if any fails. Sends no commands."""
import json
import os
import math
import subprocess
import sys
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

SINGLE = ['/ackermann_twist_adapter', '/limo_ackermann_controller', '/gazebo',
          '/robot_state_publisher', '/pure_pursuit_node', '/frenet_local_planner',
          '/ackermann_to_twist', '/pp_path_pub', '/amcl', '/map_server']
GATES = dict(xy=0.15, yaw=0.10, var_xy=0.04, var_yaw=0.01, tf_age=0.25)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    pp_expected = '--no-control' not in sys.argv
    record = '--record' in sys.argv       # recording a bag: a static map->odom, no SLAM, no AMCL
    mapping = '--mapping' in sys.argv or record     # AMCL and map_server must be absent
    expected_map = os.environ.get('EXPECT_MAP', os.path.abspath('src/Raceline-Optimization/maps/track_map3.yaml'))
    rclpy.init()
    n = Node('preflight', parameter_overrides=[
        Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    data = dict(scan=[], odom=None, amcl=None, final=[], path=None)
    n.create_subscription(LaserScan, '/scan', lambda m: data['scan'].append(time.time()),
                          qos_profile_sensor_data)
    n.create_subscription(Odometry, '/odom', lambda m: data.__setitem__('odom', m), 10)
    n.create_subscription(Path, '/pp_path', lambda m: data.__setitem__('path', m), 1)
    n.create_subscription(AckermannDriveStamped, '/final_ackermann_cmd',
                          lambda m: data['final'].append(m.drive.speed), 10)
    n.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                          lambda m: data.__setitem__('amcl', m),
                          QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
    tf = Buffer()
    TransformListener(tf, n, spin_thread=True)
    t0 = time.time()
    while time.time() - t0 < 5.0:
        rclpy.spin_once(n, timeout_sec=0.05)
    names = [ns.rstrip('/') + '/' + name for name, ns in n.get_node_names_and_namespaces()]
    fails = []

    def check(label, ok, detail=''):
        print(('PASS ' if ok else 'FAIL ') + label + (f'  {detail}' if detail else ''))
        if not ok:
            fails.append(label)

    for s in SINGLE:
        count = names.count(s)
        control = s in ('/pure_pursuit_node', '/frenet_local_planner', '/ackermann_to_twist',
                        '/pp_path_pub')
        want = 1 if (pp_expected or not control) else 0
        if mapping and s in ('/amcl', '/map_server'):
            want = 0
        check(f'node {s} x{want}', count == want, f'found {count}')
    if mapping and not record:
        check('node /slam_toolbox x1', names.count('/slam_toolbox') == 1, f"found {names.count('/slam_toolbox')}")
    ps = subprocess.run("ps -eo cmd | grep -E '^(gzserver|gzclient)' | cut -d' ' -f1 | sort | uniq -c",
                        shell=True, capture_output=True, text=True).stdout.split('\n')
    check('one gzserver', any('1 gzserver' in x for x in ps), ' '.join(x.strip() for x in ps))
    check('no gzclient during runs (CPU)', not any('gzclient' in x for x in ps))
    check('/ackermann_controller/cmd_vel publishers = 1',
          n.count_publishers('/ackermann_controller/cmd_vel') == 1,
          str(n.count_publishers('/ackermann_controller/cmd_vel')))
    if pp_expected:
        check('/cmd_vel publishers = 1', n.count_publishers('/cmd_vel') == 1,
              str(n.count_publishers('/cmd_vel')))
        check('/pp_path received', data['path'] is not None)
    # Measured between received messages, so discovery time at start does not undercount.
    w = data['scan']
    rate = (len(w) - 1) / (w[-1] - w[0]) if len(w) > 1 else 0.0
    check('/scan 7-9 Hz', 7.0 <= rate <= 9.0, f'{rate:.1f}')
    check('/odom received', data['odom'] is not None)
    out = {}
    if data['odom'] is not None:
        o = data['odom']
        out['speed'] = o.twist.twist.linear.x
        check('robot stationary', abs(o.twist.twist.linear.x) < 0.02, f'{o.twist.twist.linear.x:.4f}')
        try:
            t = tf.lookup_transform('map', 'base_link', Time())
            x, y = t.transform.translation.x, t.transform.translation.y
            yaw = yaw_of(t.transform.rotation)
            ox, oy = o.pose.pose.position.x, o.pose.pose.position.y
            err = math.hypot(x - ox, y - oy)
            yerr = abs(math.atan2(math.sin(yaw - yaw_of(o.pose.pose.orientation)),
                                  math.cos(yaw - yaw_of(o.pose.pose.orientation))))
            check(f"map->base_link error <= {GATES['xy']} m", err <= GATES['xy'], f'{err:.3f}')
            check(f"map->base_link yaw error <= {GATES['yaw']} rad", yerr <= GATES['yaw'], f'{yerr:.3f}')
            age = n.get_clock().now().nanoseconds * 1e-9 - (
                t.header.stamp.sec + t.header.stamp.nanosec * 1e-9)
            check(f"TF age <= {GATES['tf_age']} s", abs(age) <= GATES['tf_age'], f'{age:+.3f}')
        except Exception as e:  # noqa: BLE001
            check('TF map->base_link', False, str(e)[:100])
    if mapping:
        pass
    elif data['amcl'] is not None:
        c = data['amcl'].pose.covariance
        stamp = data['amcl'].header.stamp.sec + data['amcl'].header.stamp.nanosec * 1e-9
        try:
            placed = float(open('/tmp/place_t').read())
        except (OSError, ValueError):
            placed = float('inf')
        # /amcl_pose is transient-local: an old retained sample must not pass as current.
        check('AMCL pose newer than last start placement', placed > 0.0 and stamp >= placed,
              f'stamp {stamp:.2f} placed {placed:.2f}')
        check('AMCL x/y variance gate', max(c[0], c[7]) <= GATES['var_xy'], f'{c[0]:.4f}/{c[7]:.4f}')
        check('AMCL yaw variance gate', c[35] <= GATES['var_yaw'], f'{c[35]:.4f}')
    else:
        check('AMCL pose received', False)
    if pp_expected:
        check('gate closed: /final_ackermann_cmd is 0 and present',
              bool(data['final']) and max(abs(v) for v in data['final']) < 1e-6,
              f"n={len(data['final'])}")
    if '--no-params' not in sys.argv and not mapping:
        hung = []

        def param(node, name):
            # a hung call must fail the check, not wait forever with the gate closed
            if hung:
                return ''
            try:
                r = subprocess.run(['ros2', 'param', 'get', node, name], capture_output=True,
                                   text=True, timeout=15)
            except subprocess.TimeoutExpired:
                hung.append(f'{node} {name}')
                return ''
            return r.stdout.strip().split(' ')[-1]

        def same(a, b):
            try:
                return float(a) == float(b)
            except ValueError:
                return False
        yaml_file = param('/map_server', 'yaml_filename')
        check(f'map_server uses {expected_map}', yaml_file == expected_map, yaml_file)
        live = {k: param('/amcl', k) for k in ('alpha1', 'alpha2', 'alpha3', 'alpha4', 'sigma_hit',
                                               'z_hit', 'z_rand', 'max_beams', 'update_min_d',
                                               'update_min_a', 'transform_tolerance',
                                               'use_sim_time')}
        print('AMCL live params:', json.dumps(live))
        for item in [x for x in sys.argv[1:] if '=' in x]:
            k, v = item.split('=')
            check(f'AMCL {k} == {v}', k in live and same(live[k], v), live.get(k, ''))
        check('parameter queries answered', not hung and all(live.values()),
              ','.join(hung) or ','.join(k for k, v in live.items() if not v))
    print(json.dumps(dict(fails=fails)))
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if fails else 0)


if __name__ == '__main__':
    main()
