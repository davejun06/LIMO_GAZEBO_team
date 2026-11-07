#!/usr/bin/env python3
"""Translate standard Twist yaw rate to Gazebo's steering-angle Twist input."""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def convert_twist(speed, yaw_rate, wheelbase=0.24, max_steer=0.42, max_speed=1.0,
                  wheel_track=0.213):
    if not math.isfinite(speed) or not math.isfinite(yaw_rate):
        return 0.0, 0.0
    speed = max(-max_speed, min(max_speed, speed))
    if abs(speed) < 1e-6:
        return 0.0, 0.0  # An Ackermann vehicle cannot rotate in place.
    # The Gazebo plugin itself multiplies this angle by sign(speed).
    # Its final steering angle is therefore atan(wheelbase * yaw_rate / speed).
    angle = math.atan(wheelbase * yaw_rate / abs(speed))
    angle = max(-max_steer, min(max_steer, angle))
    physical_angle = angle * math.copysign(1.0, speed)
    # Gazebo controls the RIGHT REAR wheel speed, not the axle-center speed.
    # Compensate its larger/smaller path radius so left/right turns have the
    # same commanded center speed. Clamping preserves the steering curvature.
    right_speed = speed * (1 + wheel_track / (2 * wheelbase) * math.tan(physical_angle))
    return max(-max_speed, min(max_speed, right_speed)), angle


class AckermannTwistAdapter(Node):
    def __init__(self):
        super().__init__('ackermann_twist_adapter')
        self.publisher = self.create_publisher(Twist, '/ackermann_controller/cmd_vel', 1)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.receive, 1)

    def receive(self, msg):
        out = Twist()
        out.linear.x, out.angular.z = convert_twist(msg.linear.x, msg.angular.z)
        self.publisher.publish(out)


def main():
    rclpy.init()
    node = AckermannTwistAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
