#!/usr/bin/env python3
"""Put the stopped LIMO on the fixed start pose and reinitialize AMCL there, with a tight spread."""
import math
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.parameter import Parameter

X, Y, YAW = -0.0222, 0.2205, -0.1375  # whole body behind the start line, on the route tangent; user's decision. Old pose A: 7.4926918, 0.0260169, 0.6297
Z = 0.1439


def main():
    subprocess.run(['gz', 'model', '-m', 'limo', '-x', str(X), '-y', str(Y), '-z', str(Z),
                    '-R', '0', '-P', '0', '-Y', str(YAW)], check=True, timeout=15)
    time.sleep(2.0)
    rclpy.init()
    n = Node('place_start', parameter_overrides=[
        Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    pub = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
    t0 = time.time()
    while pub.get_subscription_count() < 1 and time.time() - t0 < 15:
        rclpy.spin_once(n, timeout_sec=0.1)
    if pub.get_subscription_count() < 1:
        sys.exit('no /initialpose subscriber (AMCL running?)')
    t0 = time.time()
    while n.get_clock().now().nanoseconds == 0 and time.time() - t0 < 10:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.get_clock().now().nanoseconds == 0:
        sys.exit('no /clock received')
    got = []
    n.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', got.append, 10)
    placed = n.get_clock().now().nanoseconds * 1e-9
    accepted = False
    for _ in range(5):                       # a message sent right after matching can be lost
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = n.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = X, Y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(YAW / 2), math.cos(YAW / 2)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.01
        m.pose.covariance[35] = 0.005
        pub.publish(m)
        t0 = time.time()
        while time.time() - t0 < 2.0 and not accepted:
            rclpy.spin_once(n, timeout_sec=0.1)
            for g in got:
                stamp = g.header.stamp.sec + g.header.stamp.nanosec * 1e-9
                near = math.hypot(g.pose.pose.position.x - X, g.pose.pose.position.y - Y) < 0.15
                accepted = accepted or (stamp >= placed and near)
        if accepted:
            break
    if not accepted:
        sys.exit('AMCL did not accept the start pose')
    with open('/tmp/place_t', 'w') as f:
        f.write(str(placed))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
