
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def convertir_scan(msg):
    points = []

    for i, distance in enumerate(msg.ranges):
        if not math.isfinite(distance):
            continue

        if not msg.range_min <= distance <= msg.range_max:
            continue

        angle = msg.angle_min + i * msg.angle_increment

        x = distance * math.cos(angle)
        y = distance * math.sin(angle)

        points.append((x, y))

    return points


class ScanToPoints(Node):
    def __init__(self):
        super().__init__("scan_to_points")

        self.points = []
        self.subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_cb,
            qos_profile_sensor_data
        )

    def scan_cb(self, msg):
        self.points = convertir_scan(msg)

        self.get_logger().info(
            f"{len(self.points)} points valides | "
            f"Exemple : {self.points[:3]}"
        )


def main():
    rclpy.init()
    node = ScanToPoints()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
