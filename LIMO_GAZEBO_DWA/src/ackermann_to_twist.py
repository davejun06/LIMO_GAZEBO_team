import math
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist

L = 0.24  # wheelbase [m]


class Bridge(Node):
    def __init__(self):
        super().__init__('ackermann_to_twist')
        self.declare_parameter('speed_scale', 1.0)
        self.declare_parameter('max_speed', 1.0)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.create_subscription(
            AckermannDriveStamped, '/pure_pursuit/ackermann_cmd', self.cb, 1)
        # Neither the adapter nor the Gazebo plugin times out: without this, a dead planner
        # leaves the last nonzero command running.
        self.declare_parameter('cmd_timeout', 0.5)
        self.last_cmd = None
        self.create_timer(0.1, self.watchdog)

    def watchdog(self):
        if self.last_cmd is None:
            return
        age = (self.get_clock().now() - self.last_cmd).nanoseconds * 1e-9
        if age > self.get_parameter('cmd_timeout').value:
            self.pub.publish(Twist())

    def cb(self, msg):
        self.last_cmd = self.get_clock().now()
        s = self.get_parameter('speed_scale').value
        m = self.get_parameter('max_speed').value
        v = max(-m, min(m, msg.drive.speed * s))
        w = v * math.tan(max(-0.42,min(0.42,msg.drive.steering_angle))) / L
        t = Twist()
        t.linear.x = v
        t.angular.z = w
        self.pub.publish(t)


def main():
    rclpy.init()
    n = Bridge()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.pub.publish(Twist())
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


main()
