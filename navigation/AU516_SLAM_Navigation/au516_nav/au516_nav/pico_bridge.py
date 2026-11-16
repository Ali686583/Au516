"""Pont série Raspberry Pi 5 <-> Pico W, odométrie et TF odom -> base_link.

Protocole (inchangé, celui de Codes/main.py sur le Pico) :
    Pi  -> Pico : "C <speed> <steering_us>\\n"   speed en impulsions/10 ms [-10, 50], servo en µs
    Pico -> Pi  : "E <enc_L> <enc_R>\\n"         compteurs cumulés (~1260 ticks/tour)

Ce nœud est le SEUL propriétaire du port série : il remplace encoder_reader au lancement
(il republie /encoders pour rester compatible avec l'existant).

Entrée  : /cmd_vel (geometry_msgs/Twist : v = linear.x, ω = angular.z)
Sorties : /odom (nav_msgs/Odometry), TF odom -> base_link, /encoders, /wheel_speeds
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from tf2_ros import TransformBroadcaster

from au516_nav.nav_core.control import twist_to_ackermann, clamp


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class PicoBridge(Node):
    def __init__(self):
        super().__init__('pico_bridge')
        dp = self.declare_parameter
        self.port = dp('port', '/dev/serial/by-id/usb-FTDI_TTL232R_FTBG33U7-if00-port0').value
        self.baud = dp('baudrate', 115200).value
        self.wheel_d = dp('wheel_diameter', 0.065).value  # m, à mesurer
        self.ticks = dp('ticks_per_rev', 1260.0).value
        self.sign_l = dp('encoder_left_sign', -1.0).value  # M1 est commandé avec -speed
        self.sign_r = dp('encoder_right_sign', 1.0).value
        self.track = dp('track', 0.18).value
        self.L = dp('wheelbase', 0.165).value
        self.max_steer = dp('max_steer', 0.34907).value  # 20°
        self.servo_center = dp('servo_center_us', 1500).value
        self.servo_span = dp('servo_us_at_max_steer', 500).value  # µs ajoutés pour delta_max
        self.servo_sign = dp('steering_sign', -1.0).value  # 1000 us = gauche
        self.servo_min = dp('servo_min_us', 1000).value
        self.servo_max = dp('servo_max_us', 2000).value
        self.speed_min = dp('speed_min', -10).value
        self.speed_max = dp('speed_max', 50).value
        self.cmd_timeout = dp('cmd_timeout', 0.3).value
        self.rate = dp('command_rate', 50.0).value
        self.use_encoder_yaw = dp('use_encoder_yaw', False).value
        self.odom_frame = dp('odom_frame', 'odom').value
        self.base_frame = dp('base_frame', 'base_link').value
        self.publish_tf = dp('publish_tf', True).value
        self.dry_run = dp('dry_run', False).value  # True : pas de port série

        self.m_per_tick = math.pi * self.wheel_d / self.ticks
        self.k_v = 100.0 * self.m_per_tick  # m/s par unité de speed (impulsions / 10 ms)

        self.ser = None
        if not self.dry_run:
            import serial

            self.ser = serial.Serial(self.port, self.baud, timeout=0)
        self.buffer = b''

        self.v_cmd, self.delta_cmd, self.last_cmd = 0.0, 0.0, None
        self.x = self.y = self.yaw = 0.0
        self.prev_enc, self.prev_t = None, None

        self.odom_pub = self.create_publisher(Odometry, '/odom', 20)
        self.enc_pub = self.create_publisher(Int32MultiArray, '/encoders', 10)
        self.wheel_pub = self.create_publisher(Float32MultiArray, '/wheel_speeds', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.create_timer(1.0 / self.rate, self.send_command)
        self.create_timer(0.005, self.read_serial)
        self.get_logger().info(
            f'Pico bridge sur {self.port} (dry_run={self.dry_run}), '
            f'k_v={self.k_v:.4f} m/s par unité'
        )

    # ------------------------------------------------------------------ commande
    def cmd_cb(self, msg):
        v, delta = twist_to_ackermann(msg.linear.x, msg.angular.z, self, self.delta_cmd)
        self.v_cmd, self.delta_cmd = v, delta
        self.last_cmd = self.get_clock().now()

    @property
    def wheelbase(self):  # utilisé par twist_to_ackermann
        return self.L

    def send_command(self):
        now = self.get_clock().now()
        if self.last_cmd is None or (now - self.last_cmd).nanoseconds * 1e-9 > self.cmd_timeout:
            self.v_cmd = 0.0  # perte de /cmd_vel : arrêt (en plus du chien de garde du Pico)
        speed = int(round(clamp(self.v_cmd / self.k_v, self.speed_min, self.speed_max)))
        pulse = (
            self.servo_center + self.servo_sign * self.servo_span * self.delta_cmd / self.max_steer
        )
        pulse = int(round(clamp(pulse, self.servo_min, self.servo_max)))
        if self.ser is not None:
            self.ser.write(f'C {speed} {pulse}\n'.encode())
        if (
            self.dry_run
        ):  # sans robot : odométrie intégrée depuis la commande (option C de SLAM_2D.md)
            self.integrate(speed * self.k_v, 1.0 / self.rate, None, None)

    # ------------------------------------------------------------------ odométrie
    def read_serial(self):
        if self.ser is None:
            return
        self.buffer += self.ser.read(self.ser.in_waiting or 1)
        *lines, self.buffer = self.buffer.split(b'\n')
        for line in lines:
            parts = line.decode(errors='ignore').split()
            if len(parts) != 3 or parts[0] != 'E':
                continue
            try:
                enc_l, enc_r = int(parts[1]), int(parts[2])
            except ValueError:
                continue
            self.enc_pub.publish(Int32MultiArray(data=[enc_l, enc_r]))
            self.on_encoders(enc_l, enc_r)

    def on_encoders(self, enc_l, enc_r):
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.prev_enc is None:
            self.prev_enc, self.prev_t = (enc_l, enc_r), t
            return
        dt = t - self.prev_t
        if dt <= 0.0:
            return
        dl = self.sign_l * (enc_l - self.prev_enc[0]) * self.m_per_tick
        dr = self.sign_r * (enc_r - self.prev_enc[1]) * self.m_per_tick
        self.prev_enc, self.prev_t = (enc_l, enc_r), t
        vl, vr = dl / dt, dr / dt
        self.wheel_pub.publish(Float32MultiArray(data=[vl, vr]))
        self.integrate((vl + vr) / 2.0, dt, vl, vr)

    def integrate(self, v, dt, vl, vr):
        if self.use_encoder_yaw and vl is not None:
            omega = (vr - vl) / self.track
        else:
            omega = v * math.tan(self.delta_cmd) / self.L  # modèle bicyclette
        self.x += v * math.cos(self.yaw + omega * dt / 2) * dt
        self.y += v * math.sin(self.yaw + omega * dt / 2) * dt
        self.yaw = math.atan2(math.sin(self.yaw + omega * dt), math.cos(self.yaw + omega * dt))
        self.publish_odom(v, omega)

    def publish_odom(self, v, omega):
        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quat(self.yaw)
        od = Odometry()
        od.header.stamp, od.header.frame_id, od.child_frame_id = (
            stamp,
            self.odom_frame,
            self.base_frame,
        )
        od.pose.pose.position.x, od.pose.pose.position.y = self.x, self.y
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = qz, qw
        od.twist.twist.linear.x, od.twist.twist.angular.z = v, omega
        od.pose.covariance[0] = od.pose.covariance[7] = 0.01
        od.pose.covariance[35] = 0.05
        self.odom_pub.publish(od)
        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = (
                stamp,
                self.odom_frame,
                self.base_frame,
            )
            tf.transform.translation.x, tf.transform.translation.y = self.x, self.y
            tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
            self.tf.sendTransform(tf)

    def destroy_node(self):
        if self.ser is not None:
            try:
                self.ser.write(b'C 0 %d\n' % int(self.servo_center))
            finally:
                self.ser.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = PicoBridge()
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
