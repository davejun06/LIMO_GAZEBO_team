#!/usr/bin/env python3
import csv
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class PPPathPub(Node):
    def __init__(self):
        super().__init__("pp_path_pub")
        self.declare_parameter("csv_path", "")
        self.declare_parameter("path_topic", "/pp_path")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("speed_factor", 1.0)
        self.declare_parameter("period", 1.0)
        g = lambda n: self.get_parameter(n).value
        self.frame_id, self.factor = g("frame_id"), float(g("speed_factor"))

        self.pts, skipped = [], 0
        with open(g("csv_path"), newline="") as f:
            for row in csv.reader(f):
                try:
                    self.pts.append((float(row[0]), float(row[1]), float(row[2])))
                except (ValueError, IndexError):
                    skipped += 1                      # 헤더, 빈 줄 건너뜀
        if not self.pts:
            raise SystemExit("경로를 읽지 못했습니다: csv_path 확인")
        vs = [p[2] * self.factor for p in self.pts]
        self.get_logger().info(
            f"{len(self.pts)}점 로드 (건너뜀 {skipped}), 속도 {min(vs):.2f}~{max(vs):.2f} m/s, 토픽 {g('path_topic')}")

        self.pub = self.create_publisher(Path, g("path_topic"), 10)
        self.last_subs = -1
        self.create_timer(float(g("period")), self.publish)

    def publish(self):
        msg = Path()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y, v in self.pts:
            p = PoseStamped()
            p.header.frame_id = self.frame_id
            p.pose.position.x, p.pose.position.y = x, y
            p.pose.position.z = v * self.factor       # 속도를 높이로 표시: RViz에서 확인용, 추종에는 쓰지 않음
            p.pose.orientation.w = v * self.factor    # pure_pursuit이 속도를 읽는 필드
            msg.poses.append(p)
        self.pub.publish(msg)
        n = self.pub.get_subscription_count()
        if n != self.last_subs:
            self.get_logger().info(f"구독자 수: {n}")
            self.last_subs = n


def main():
    rclpy.init()
    node = PPPathPub()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
