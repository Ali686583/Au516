"""Simulateur 2D ROS 2 (remplace le robot réel quand Gazebo n'est pas disponible).

Publie /scan (repère laser), /odom et TF odom -> base_link ; s'abonne à /cmd_vel.
La voiture suit le modèle bicyclette (Ackermann). Les murs du scénario sont publiés sur
/sim/walls pour RViz et /sim/collision passe à True en cas de contact.

    ros2 launch au516_nav sim.launch.py scenario:=D_zigzag
Scénarios : A_straight B_left C_right D_zigzag E_narrow_ok E_narrow_refused
            F_obstacle F_blocked G_dead_end T_junction ring
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster

from au516_nav.nav_core import VehicleParams, twist_to_ackermann
from au516_nav.nav_core.sim2d import World, Car
from au516_nav.nav_core.worlds import SCENARIOS


class Sim2D(Node):
    def __init__(self):
        super().__init__('sim_2d')
        dp = self.declare_parameter
        self.scenario = dp('scenario', 'ring').value
        self.veh = VehicleParams(
            wheelbase=dp('vehicle.wheelbase', 0.25).value,
            width=dp('vehicle.width', 0.20).value,
            rear_overhang=dp('vehicle.rear_overhang', 0.06).value,
            front_overhang=dp('vehicle.front_overhang', 0.30).value,
            max_steer=dp('vehicle.max_steer', 0.45).value,
            laser_x=dp('vehicle.laser_x', 0.22).value,
        )
        self.beams = dp('beams', 450).value  # RPLIDAR C1 : ~450-720 points / tour à 10 Hz
        self.noise = dp('range_noise', 0.01).value
        self.odom_noise = dp('odom_drift', 0.02).value  # dérive relative de l'odométrie
        self.dt = 1.0 / dp('physics_rate', 50.0).value
        self.scan_period = 1.0 / dp('scan_rate', 10.0).value

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
        self.odom_pub = self.create_publisher(Odometry, '/odom', 20)
        self.coll_pub = self.create_publisher(Bool, '/sim/collision', 10)
        self.walls_pub = self.create_publisher(MarkerArray, '/sim/walls', 1)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.create_timer(self.dt, self.tick)
        self.create_timer(1.0, self.publish_walls)
        self.get_logger().info(f'Simulation 2D : scénario {self.scenario}')

    def cmd_cb(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)
        self.last_cmd_t = self.get_clock().now()

    def tick(self):
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
        if self.world.collides(self.car.polygon()):
            if not self.collided:
                self.get_logger().error('COLLISION !')
            self.collided = True
        self.coll_pub.publish(Bool(data=self.collided))
        self.publish_odom(v, v * math.tan(self.delta) / self.veh.wheelbase)
        self.scan_acc += self.dt
        if self.scan_acc >= self.scan_period:
            self.scan_acc = 0.0
            self.publish_scan()

    def publish_odom(self, v, omega):
        stamp = self.get_clock().now().to_msg()
        x, y, yaw = self.odom
        od = Odometry()
        od.header.stamp, od.header.frame_id, od.child_frame_id = stamp, 'odom', 'base_link'
        od.pose.pose.position.x, od.pose.pose.position.y = x, y
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(
            yaw / 2
        )
        od.twist.twist.linear.x, od.twist.twist.angular.z = v, omega
        self.odom_pub.publish(od)
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, 'odom', 'base_link'
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.tf.sendTransform(tf)

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
        x0, y0, yaw0 = self.start
        c, s = math.cos(-yaw0), math.sin(-yaw0)
        m = Marker()
        m.header.frame_id, m.ns, m.id, m.type = 'odom', 'walls', 0, Marker.LINE_LIST
        m.scale.x = 0.03
        m.color = ColorRGBA(r=0.6, g=0.6, b=0.6, a=1.0)
        m.pose.orientation.w = 1.0
        for ax, ay, bx, by in self.segments:
            for px, py in ((ax, ay), (bx, by)):
                dx, dy = px - x0, py - y0
                m.points.append(Point(x=c * dx - s * dy, y=s * dx + c * dy))
        self.walls_pub.publish(MarkerArray(markers=[m]))


def main():
    rclpy.init()
    node = Sim2D()
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
