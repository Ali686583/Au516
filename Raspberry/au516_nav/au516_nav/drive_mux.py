"""
Chooses the command sent to the robot: keyboard (move) or autonomous navigation (local_navigator).

serial_bridge stays the only node that talks to the Pico. In robot_launch.py its /cmd_drive input
is remapped to /cmd_drive_mux, which is published by this node only:

    move (keyboard)  --/cmd_drive [speed, pulse]-->  +-----------+
                                                     | drive_mux |--/cmd_drive_mux--> serial_bridge
    local_navigator  --/cmd_vel   Twist (v, w)  -->  +-----------+
                     --/nav/enabled (Bool, latched): selects the source

  MANUAL (navigation off): the keyboard commands go through unchanged.
  AUTO   (navigation on) : /cmd_vel is converted to [speed, pulse] with the bicycle model and the
                           same calibration as odometry_node; keyboard commands are ignored.
                           Any key of move.py switches the navigation off (manual takeover).

A stop [0, 1500] is sent on every mode change. In AUTO, without /cmd_vel for cmd_timeout seconds,
a stop is sent too (serial_bridge and the Pico also have their own watchdogs).
The current mode is published on /drive/mode (latched).
"""

import math

from au516_nav.nav_core import DriveCalibration, twist_to_ackermann, VehicleParams
from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32MultiArray, String


MANUAL = 'MANUAL'
AUTO = 'AUTO'


def latched_qos():
    """Late subscribers still get the last message (like a ROS 1 latched topic)."""
    return QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )


class DriveMux(Node):
    def __init__(self, **kwargs):
        super().__init__('drive_mux', **kwargs)
        dp = self.declare_parameter

        # Same values as odometry_node.py and the servo calibration of the team
        self.cal = DriveCalibration(
            wheel_diameter=dp('wheel_diameter', 0.065).value,
            ticks_per_rev=dp('ticks_per_rev', 1260.0).value,
            max_steer=dp('max_steer', math.radians(20.0)).value,
            servo_center=dp('servo_center_us', 1500).value,
            servo_span=dp('servo_us_at_max_steer', 500).value,
            steering_sign=dp('steering_sign', -1.0).value,
            servo_min=dp('servo_min_us', 1000).value,
            servo_max=dp('servo_max_us', 2000).value,
            speed_min=dp('speed_min', -10).value,
            speed_max=dp('speed_max', 50).value,
        )
        # Must match vehicle.wheelbase of local_navigator (omega = v * tan(delta) / L)
        self.vehicle = VehicleParams(
            wheelbase=dp('wheelbase', 0.165).value, max_steer=self.cal.max_steer
        )
        self.cmd_timeout = dp('cmd_timeout', 0.3).value
        output_topic = dp('output_topic', '/cmd_drive_mux').value

        self.mode = MANUAL
        self.delta = 0.0  # last steering angle, kept when v = 0
        self.last_nav_time = None
        self.nav_timed_out = False

        self.out_pub = self.create_publisher(Int32MultiArray, output_topic, 10)
        self.mode_pub = self.create_publisher(String, '/drive/mode', latched_qos())
        self.create_subscription(Int32MultiArray, '/cmd_drive', self.manual_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.nav_cb, 10)
        self.create_subscription(Bool, '/nav/enabled', self.enabled_cb, latched_qos())
        self.create_timer(0.05, self.check_timeout)

        self.publish_mode()
        self.get_logger().info(
            f'drive_mux ready ({self.mode}) -> {output_topic}, '
            f'k_v={self.cal.k_v:.4f} m/s per speed unit'
        )

    # ------------------------------------------------------------------ inputs
    def manual_cb(self, msg):
        if len(msg.data) != 2:
            self.get_logger().warn('/cmd_drive expects [speed, steering_pwm]')
            return
        if self.mode == MANUAL:
            self.publish_command(msg.data[0], msg.data[1])
        elif list(msg.data) != [0, self.cal.servo_center]:  # move keeps sending a neutral command
            self.get_logger().warn(
                'Keyboard command ignored: autonomous navigation is active '
                '(any key of move takes the control back)',
                throttle_duration_sec=5.0,
            )

    def nav_cb(self, msg):
        if self.mode != AUTO:
            return
        v, w = msg.linear.x, msg.angular.z
        self.last_nav_time = self.get_clock().now()
        self.nav_timed_out = False
        if not (math.isfinite(v) and math.isfinite(w)):
            self.get_logger().error('Invalid /cmd_vel (NaN or inf): car stopped')
            self.stop()
            return
        v, self.delta = twist_to_ackermann(v, w, self.vehicle, self.delta)
        speed, pulse = self.cal.to_command(v, self.delta)
        self.publish_command(speed, pulse)

    def enabled_cb(self, msg):
        mode = AUTO if msg.data else MANUAL
        if mode == self.mode:
            return
        self.mode = mode
        self.delta = 0.0
        self.last_nav_time = None
        self.stop()
        self.publish_mode()
        self.get_logger().info(f'Mode {mode}')

    def check_timeout(self):
        if self.mode != AUTO or self.last_nav_time is None or self.nav_timed_out:
            return
        age = (self.get_clock().now() - self.last_nav_time).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            self.nav_timed_out = True
            self.stop()
            self.get_logger().warn(f'No /cmd_vel for {age:.1f} s: car stopped')

    # ------------------------------------------------------------------ outputs
    def publish_command(self, speed, pulse):
        msg = Int32MultiArray()
        msg.data = [int(speed), int(pulse)]
        self.out_pub.publish(msg)

    def stop(self):
        self.publish_command(0, self.cal.servo_center)

    def publish_mode(self):
        self.mode_pub.publish(String(data=self.mode))


def main():
    rclpy.init()
    node = DriveMux()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # CTRL+C (ROS 2 Jazzy raises ExternalShutdownException)
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
