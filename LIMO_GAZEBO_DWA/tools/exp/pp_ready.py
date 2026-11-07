#!/usr/bin/env python3
"""Gate-closed readiness check of Pure Pursuit. Exit 0 only if its pose lookup really works.

Needs: process alive, use_fused_odometry false (else the map lookup is skipped), the lookahead
circle published after the lookup seen in the last 3 s, and no lookup warning in that window.
"""
import re
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

LOG = '/tmp/pp.log'
TOPIC = sys.argv[1] if len(sys.argv) > 1 else '/lookahead_circle'
fails = []


def check(label, ok, detail=''):
    print(('PASS ' if ok else 'FAIL ') + label + (f'  {detail}' if detail else ''))
    if not ok:
        fails.append(label)


alive = subprocess.run(['pgrep', '-f', 'lib/pure_pursuit/pure_pursuit_node'],
                       capture_output=True, text=True).stdout.split()
check('Pure Pursuit process alive', len(alive) == 1, str(alive))
fused = subprocess.run(['ros2', 'param', 'get', '/pure_pursuit_node', 'use_fused_odometry'],
                       capture_output=True, text=True, timeout=15).stdout.strip()
check('use_fused_odometry false', fused.endswith('False'), fused)
rclpy.init()
n = Node('pp_ready')
seen = []
n.create_subscription(Marker, TOPIC, lambda m: seen.append(time.time()), 10)
t0 = time.time()
while time.time() - t0 < 3.0:
    rclpy.spin_once(n, timeout_sec=0.05)
check(f'{TOPIC} arriving', len(seen) > 0, f'{len(seen)} msgs in 3 s')
now = time.time()
recent = []
for line in open(LOG, errors='replace'):
    m = re.match(r'\[WARN\] \[(\d+\.\d+)\].*Transform not available', line)
    if m and now - float(m.group(1)) < 3.0:
        recent.append(line[:90])
check("no 'Transform not available' in last 3 s", not recent, f'{len(recent)}')
n.destroy_node()
rclpy.shutdown()
sys.exit(1 if fails else 0)
