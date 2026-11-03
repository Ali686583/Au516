
import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdometryNode(Node):

    def __init__(self):
        super().__init__("odometry_node")

        # Robot parameters
        self.wheel_diameter = 0.065
        self.wheelbase = 0.165
        self.ticks_per_rev = 1260.0

        # Steering parameters
        self.steering_center = 1500
        self.steering_pwm = 1500

        # Set after steering calibration
        self.max_steering_deg = 0.0
        self.steering_sign = 1.0

        # Robot position
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_encoders = None
        self.last_time = None

        self.meters_per_tick = (
            math.pi * self.wheel_diameter / self.ticks_per_rev
        )

        # ROS subscriptions
        self.create_subscription(
            Int32MultiArray,
            "/encoders",
            self.encoder_callback,
            10
        )

        self.create_subscription(
            Int32,
            "/steering_pwm",
            self.steering_callback,
            10
        )

        # ROS publishers
        self.odom_pub = self.create_publisher(
            Odometry,
            "/odom",
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Odometry node started")


    def steering_callback(self, msg):
        self.steering_pwm = max(
            1000, min(2000, msg.data)
        )


    def encoder_callback(self, msg):

        if len(msg.data) != 2:
            return

        now = self.get_clock().now()

        enc_left = msg.data[0]
        enc_right = msg.data[1]

        if self.last_encoders is None:
            self.last_encoders = (enc_left, enc_right)
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds / 1e9

        if dt <= 0:
            self.last_encoders = (enc_left, enc_right)
            self.last_time = now
            return

        # Encoder increments
        delta_left = -(enc_left - self.last_encoders[0])
        delta_right = enc_right - self.last_encoders[1]

        self.last_encoders = (enc_left, enc_right)
        self.last_time = now

        # Distance travelled by each wheel
        distance_left = delta_left * self.meters_per_tick
        distance_right = delta_right * self.meters_per_tick

        distance = (distance_left + distance_right) / 2.0

        # Ackermann steering model
        steering_ratio = (
            self.steering_pwm - self.steering_center
        ) / 500.0

        steering_angle = math.radians(
            steering_ratio
            * self.max_steering_deg
            * self.steering_sign
        )

        delta_theta = (
            distance * math.tan(steering_angle)
            / self.wheelbase
        )

        # Position integration
        theta_mid = self.theta + delta_theta / 2.0

        self.x += distance * math.cos(theta_mid)
        self.y += distance * math.sin(theta_mid)
        self.theta += delta_theta

        self.theta = math.atan2(
            math.sin(self.theta),
            math.cos(self.theta)
        )

        linear_velocity = distance / dt
        angular_velocity = delta_theta / dt

        self.publish_odometry(
            now, linear_velocity, angular_velocity
        )


    def publish_odometry(self, now, linear, angular):

        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        msg = Odometry()

        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y

        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.twist.twist.linear.x = linear
        msg.twist.twist.angular.z = angular

        # Conservative initial uncertainty estimates
        msg.pose.covariance[0] = 0.02
        msg.pose.covariance[7] = 0.02
        msg.pose.covariance[14] = 1e6
        msg.pose.covariance[21] = 1e6
        msg.pose.covariance[28] = 1e6
        msg.pose.covariance[35] = 1.0

        self.odom_pub.publish(msg)

        # TF: odom -> base_link
        tf = TransformStamped()

        tf.header = msg.header
        tf.child_frame_id = "base_link"

        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)

    node = OdometryNode()

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
