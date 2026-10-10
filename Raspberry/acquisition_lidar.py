
import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class LidarCapture(Node):
    def __init__(self):
        super().__init__("lidar_capture")

        self.scan = None

        self.subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.on_scan,
            qos_profile_sensor_data
        )

    def on_scan(self, msg):
        # Conserver le premier scan reçu
        if self.scan is None:
            self.scan = msg


def save_scan(msg):
    path = Path.home() / "scan_lidar.csv"
    valid_count = 0

    with path.open("w", newline="") as file:
        writer = csv.writer(file)

        writer.writerow([
            "index", "angle_rad", "distance_m",
            "valid", "x_m", "y_m"
        ])

        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment

            valid = (
                math.isfinite(distance)
                and msg.range_min <= distance <= msg.range_max
            )

            x = distance * math.cos(angle) if valid else ""
            y = distance * math.sin(angle) if valid else ""

            valid_count += int(valid)

            writer.writerow([
                i, angle, distance, valid, x, y
            ])

    print(f"Repère LiDAR : {msg.header.frame_id}")
    print(f"Mesures reçues : {len(msg.ranges)}")
    print(f"Mesures valides : {valid_count}")
    print(f"Fichier sauvegardé : {path}")


def main():
    rclpy.init()
    node = LidarCapture()

    try:
        print("En attente d'un scan sur /scan ...")

        while rclpy.ok() and node.scan is None:
            rclpy.spin_once(node, timeout_sec=1.0)

        if node.scan is not None:
            save_scan(node.scan)

    except KeyboardInterrupt:
        print("Capture interrompue.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
