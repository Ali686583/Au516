__author__ = "Johvany GUSTAVE"
__copyright__ = "Copyright 2026, AU516, IPSA 2026"
__credits__ = ["Johvany GUSTAVE"]
__license__ = "Apache License 2.0"
__version__ = "1.0.0"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanViewer(Node):
    """
    A custom ROS 2 Node that listens to LiDAR data (LaserScan messages)
    and prints a summary of each scan to the console.
    """
    def __init__(self):
        Node.__init__(self, "scan_viewer")

        """
        Create a subscription to listen to incoming LaserScan messages.
        Arguments:
        1. LaserScan: The type of message expected on the topic.
        2. "/scan": The name of the topic we want to listen to.
        3. self.scan_cb: The callback method triggered every time a message arrives.
        4. 10: The Quality of Service (QoS) queue size (stores up to 10 messages if busy).
        """
        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)


    def scan_cb(self, msg : LaserScan):
        """
        This is the callback function. It runs automatically in the background
        whenever a new message is published on the '/scan' topic.

        Parameters:
            msg (LaserScan): The actual sensor data message received from the robot/simulation.
        """
        self.get_logger().info(f"Received scan: {len(msg.ranges)} ranges. First 5: {msg.ranges[:5]}")


def main(args=None):
    rclpy.init(args=args)

    node = ScanViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()