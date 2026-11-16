"""Simulateur 2D ROS 2 (remplace le robot réel quand Gazebo n'est pas disponible).

Deux modes :

* emulate_pico = false (mode direct, celui d'origine) : publie /scan (repère laser), /odom et
  TF odom -> base_link ; s'abonne à /cmd_vel.

* emulate_pico = true (utilisé par robot_launch.py sim:=true) : le simulateur se comporte comme
  le MATÉRIEL. Il crée un faux port série (pseudo-terminal, lien `pico_link`) qui parle exactement
  comme Codes/main.py sur le Pico : il reçoit "C <speed> <pulse>", applique le chien de garde de
  500 ms, et renvoie "E <enc_G> <enc_D>" toutes les 20 ms (roue gauche comptée en négatif, comme
  le moteur M1 commandé avec -speed). Il publie aussi /scan comme le RPLIDAR. Tout le reste du
  robot tourne donc sans modification : serial_bridge, odometry_node, drive_mux, slam_toolbox,
  local_navigator.

Dans les deux modes : la voiture suit le modèle bicyclette (Ackermann), les murs du scénario sont
publiés sur /sim/walls pour RViz, /sim/collision passe à True en cas de contact et la vraie pose
(vérité terrain, alignée sur le départ) est publiée sur /sim/ground_truth.

    ros2 launch au516_nav robot_launch.py sim:=true autostart:=true scenario:=D_zigzag
Scénarios : A_straight B_left C_right D_zigzag E_narrow_ok E_narrow_refused
            F_obstacle F_blocked G_dead_end T_junction ring
"""

import fcntl
import math
import os
import tty

from au516_nav.nav_core import DriveCalibration, twist_to_ackermann, VehicleParams
from au516_nav.nav_core.sim2d import Car, World
from au516_nav.nav_core.worlds import SCENARIOS
from geometry_msgs.msg import Point, TransformStamped, Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, ColorRGBA
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

PICO_WATCHDOG = 0.5  # s, WATCHDOG_MS de Codes/main.py


def yaw_to_quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class FakePico:
    """Pico W émulé sur un pseudo-terminal (même protocole que Codes/main.py)."""

    def __init__(self, link, cal):
        self.cal = cal
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)  # pas d'écho ni de conversion des fins de ligne
        flags = fcntl.fcntl(self.master, fcntl.F_GETFL)
        fcntl.fcntl(self.master, fcntl.F_SETFL, flags | os.O_NONBLOCK)
        self.link = link
        if os.path.lexists(link):
            os.remove(link)
        os.symlink(os.ttyname(self.slave), link)
        self.buffer = b''
        self.speed, self.pulse = 0, cal.servo_center
        self.last_cmd = None
        self.enc = [0.0, 0.0]  # ticks cumulés (gauche, droite)
        self.nb_cmds = 0

    def receive(self, now):
        """Lit les commandes "C <speed> <pulse>" et applique le chien de garde du Pico."""
        try:
            self.buffer += os.read(self.master, 4096)
        except (BlockingIOError, OSError):
            pass
        while b'\n' in self.buffer:
            line, self.buffer = self.buffer.split(b'\n', 1)
            parts = line.strip().split()
            if len(parts) != 3 or parts[0] != b'C':
                continue
            try:
                speed, pulse = int(parts[1]), int(parts[2])
            except ValueError:
                continue
            c = self.cal
            self.speed = max(c.speed_min, min(c.speed_max, speed))  # MotorDriverI2C.clamp
            self.pulse = max(c.servo_min, min(c.servo_max, pulse))  # set_steering
            self.last_cmd = now
            self.nb_cmds += 1
        if self.last_cmd is not None and now - self.last_cmd > PICO_WATCHDOG:
            self.speed, self.pulse, self.last_cmd = 0, self.cal.servo_center, None  # stop_robot

    def send_encoders(self, ticks):
        """Ajoute `ticks` aux deux roues et envoie "E <enc_G> <enc_D>"."""
        self.enc[0] -= ticks  # M1 (gauche) est commandé avec -speed : compte en négatif
        self.enc[1] += ticks
        line = f'E {int(round(self.enc[0]))} {int(round(self.enc[1]))}\n'.encode()
        try:
            os.write(self.master, line)
        except (BlockingIOError, OSError):
            pass  # personne ne lit encore le port : la ligne est perdue, comme sur l'UART

    def close(self):
        for fd in (self.master, self.slave):
            try:
                os.close(fd)
            except OSError:
                pass
        if os.path.islink(self.link):
            os.remove(self.link)


class Sim2D(Node):
    def __init__(self):
        super().__init__('sim_2d')
        dp = self.declare_parameter
        self.scenario = dp('scenario', 'ring').value
        self.veh = VehicleParams(
            wheelbase=dp('vehicle.wheelbase', 0.165).value,
            width=dp('vehicle.width', 0.20).value,
            rear_overhang=dp('vehicle.rear_overhang', 0.06).value,
            front_overhang=dp('vehicle.front_overhang', 0.30).value,
            max_steer=dp('vehicle.max_steer', 0.34907).value,
            laser_x=dp('vehicle.laser_x', 0.22).value,
            laser_y=dp('vehicle.laser_y', 0.0).value,
            laser_yaw=dp('vehicle.laser_yaw', 0.0).value,
        )
        self.beams = dp('beams', 450).value  # RPLIDAR C1 : ~450-720 points / tour à 10 Hz
        self.noise = dp('range_noise', 0.01).value
        self.odom_noise = dp('odom_drift', 0.02).value  # dérive relative de l'odométrie
        self.dt = 1.0 / dp('physics_rate', 50.0).value
        self.scan_period = 1.0 / dp('scan_rate', 10.0).value
        self.emulate_pico = dp('emulate_pico', False).value
        pico_link = dp('pico_link', '/tmp/au516_fake_pico').value

        segs, (x, y, yaw) = SCENARIOS[self.scenario]()
        self.segments = segs
        self.world, self.car = World(segs), Car(self.veh, x, y, yaw)
        self.odom = [0.0, 0.0, 0.0]  # odométrie (dérive), départ à l'origine de odom
        self.start = (x, y, yaw)
        self.cmd = (0.0, 0.0)
        self.delta = 0.0
        self.last_cmd_t = self.get_clock().now()
        self.rng = np.random.default_rng(0)
        self.scan_acc = 0.0
        self.collided = False

        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.coll_pub = self.create_publisher(Bool, '/sim/collision', 10)
        self.walls_pub = self.create_publisher(MarkerArray, '/sim/walls', 1)
        self.truth_pub = self.create_publisher(Odometry, '/sim/ground_truth', 10)

        self.pico = None
        if self.emulate_pico:
            # Calibration de l'équipe (odometry_node.py, Commands list), même que drive_mux
            self.cal = DriveCalibration(max_steer=self.veh.max_steer)
            self.pico = FakePico(pico_link, self.cal)
            mode = f'Pico émulé sur {pico_link} -> {os.ttyname(self.pico.slave)}'
        else:
            self.odom_pub = self.create_publisher(Odometry, '/odom', 20)
            self.tf = TransformBroadcaster(self)
            self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
            mode = 'commande directe /cmd_vel'
        self.create_timer(self.dt, self.tick)
        self.create_timer(1.0, self.publish_walls)
        self.get_logger().info(f'Simulation 2D : scénario {self.scenario}, {mode}')

    def cmd_cb(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)
        self.last_cmd_t = self.get_clock().now()

    def tick(self):
        if self.pico is not None:
            self.tick_pico()
        else:
            self.tick_direct()
        if self.world.collides(self.car.polygon()):
            if not self.collided:
                self.get_logger().error('COLLISION !')
            self.collided = True
        self.coll_pub.publish(Bool(data=self.collided))
        self.publish_ground_truth()
        self.scan_acc += self.dt
        if self.scan_acc >= self.scan_period:
            self.scan_acc = 0.0
            self.publish_scan()

    def tick_direct(self):
        if (self.get_clock().now() - self.last_cmd_t).nanoseconds * 1e-9 > 0.5:
            self.cmd = (0.0, 0.0)  # même chien de garde que le Pico
        v, self.delta = twist_to_ackermann(self.cmd[0], self.cmd[1], self.veh, self.delta)
        if self.collided:
            v = 0.0
        ox, oy, oyaw = self.car.x, self.car.y, self.car.yaw
        self.car.step(v, self.delta, self.dt)
        # odométrie bruitée : on intègre le déplacement réel avec une erreur d'échelle
        ds = math.hypot(self.car.x - ox, self.car.y - oy) * math.copysign(1.0, v or 1.0)
        dyaw = self.car.yaw - oyaw
        ds *= 1.0 + self.rng.normal(0, self.odom_noise)
        dyaw *= 1.0 + self.rng.normal(0, self.odom_noise)
        self.odom[0] += ds * math.cos(self.odom[2] + dyaw / 2)
        self.odom[1] += ds * math.sin(self.odom[2] + dyaw / 2)
        self.odom[2] += dyaw
        self.publish_odom(v, v * math.tan(self.delta) / self.veh.wheelbase)

    def tick_pico(self):
        """Une période de 20 ms du Pico : commandes reçues -> moteurs/servo -> encodeurs."""
        self.pico.receive(self.get_clock().now().nanoseconds * 1e-9)
        speed = 0 if self.collided else self.pico.speed
        v = self.cal.speed_to_velocity(speed)  # asservissement du driver : vitesse tenue
        self.car.step(v, self.cal.pulse_to_steer(self.pico.pulse), self.dt)
        # speed = impulsions par 10 ms ; erreur d'échelle aléatoire comme le mode direct
        ticks = speed * 100.0 * self.dt * (1.0 + self.rng.normal(0, self.odom_noise))
        self.pico.send_encoders(ticks)

    def publish_odom(self, v, omega):
        stamp = self.get_clock().now().to_msg()
        x, y, yaw = self.odom
        qz, qw = yaw_to_quat(yaw)
        od = Odometry()
        od.header.stamp, od.header.frame_id, od.child_frame_id = stamp, 'odom', 'base_link'
        od.pose.pose.position.x, od.pose.pose.position.y = x, y
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = qz, qw
        od.twist.twist.linear.x, od.twist.twist.angular.z = v, omega
        self.odom_pub.publish(od)
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, 'odom', 'base_link'
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
        self.tf.sendTransform(tf)

    def start_frame(self, px, py):
        """Point du monde -> repère aligné sur la pose de départ (= odom au démarrage)."""
        x0, y0, yaw0 = self.start
        c, s = math.cos(-yaw0), math.sin(-yaw0)
        dx, dy = px - x0, py - y0
        return c * dx - s * dy, s * dx + c * dy

    def publish_ground_truth(self):
        """Vraie pose de la voiture (pour vérifier l'odométrie, le SLAM et compter les tours)."""
        x, y = self.start_frame(self.car.x, self.car.y)
        yaw = self.car.yaw - self.start[2]
        qz, qw = yaw_to_quat(yaw)
        gt = Odometry()
        gt.header.stamp, gt.header.frame_id, gt.child_frame_id = (
            self.get_clock().now().to_msg(),
            'odom',
            'base_link_truth',
        )
        gt.pose.pose.position.x, gt.pose.pose.position.y = x, y
        gt.pose.pose.orientation.z, gt.pose.pose.orientation.w = qz, qw
        gt.twist.twist.linear.x = self.car.v
        self.truth_pub.publish(gt)

    def publish_scan(self):
        lx, ly, lyaw = self.car.laser_pose()
        r, amin, inc = self.world.raycast(
            lx, ly, lyaw, n_beams=self.beams, noise=self.noise, rng=self.rng
        )
        msg = LaserScan()
        msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), 'laser'
        msg.angle_min, msg.angle_increment = amin, inc
        msg.angle_max = amin + inc * (self.beams - 1)
        msg.range_min, msg.range_max = 0.05, 12.0
        msg.scan_time = self.scan_period
        msg.ranges = [float(x) for x in r]
        self.scan_pub.publish(msg)

    def publish_walls(self):
        """Murs dans le repère odom, alignés sur la pose de départ (vérité terrain)."""
        m = Marker()
        m.header.frame_id, m.ns, m.id, m.type = 'odom', 'walls', 0, Marker.LINE_LIST
        m.scale.x = 0.03
        m.color = ColorRGBA(r=0.6, g=0.6, b=0.6, a=1.0)
        m.pose.orientation.w = 1.0
        for ax, ay, bx, by in self.segments:
            for px, py in ((ax, ay), (bx, by)):
                x, y = self.start_frame(px, py)
                m.points.append(Point(x=x, y=y))
        self.walls_pub.publish(MarkerArray(markers=[m]))

    def destroy_node(self):
        if self.pico is not None:
            self.pico.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = Sim2D()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # CTRL+C (ROS 2 Jazzy raises ExternalShutdownException)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
