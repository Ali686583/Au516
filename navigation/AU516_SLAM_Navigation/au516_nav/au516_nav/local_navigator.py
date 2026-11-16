"""Nœud de navigation autonome : /scan (+ /map et pose SLAM) -> θ_cible -> /cmd_vel.

Rôles (cf. README_NAVIGATION.md) :
  * SLAM Toolbox : pose (TF map -> odom) et carte /map. Ce nœud ne fait PAS de SLAM.
  * Navigation locale (ici) : passages traversables, θ_cible, évitement, commande v/ω.
  * Navigation globale (ici, couche légère) : la carte pénalise les zones inconnues et les
    obstacles connus, et la mémoire d'impasses évite de retourner dans un cul-de-sac.

Ce nœud est le SEUL éditeur de /cmd_vel (pas de conflit entre contrôleurs).
"""

import math
from dataclasses import fields

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String, Bool, Float32MultiArray, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

from au516_nav.nav_core import (
    VehicleParams,
    NavParams,
    LocalNavigator,
    GridMap,
    DeadEndMemory,
    scan_to_points,
    differential_wheel_speeds,
    update_dataclass,
)


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _declare_dataclass(node, prefix, obj):
    values = {}
    for f in fields(obj):
        default = getattr(obj, f.name)
        if isinstance(default, tuple):
            default = [float(x) for x in default]
        values[f.name] = node.declare_parameter(f'{prefix}.{f.name}', default).value
    return update_dataclass(obj, values)


class LocalNavigatorNode(Node):
    def __init__(self):
        super().__init__('local_navigator')
        self.veh = _declare_dataclass(self, 'vehicle', VehicleParams())
        self.p = _declare_dataclass(self, 'nav', NavParams())
        dp = self.declare_parameter
        self.map_frame = dp('map_frame', 'map').value
        self.odom_frame = dp('odom_frame', 'odom').value
        self.base_frame = dp('base_frame', 'base_link').value
        self.use_map = dp('use_map', True).value
        self.enabled = dp('autostart', True).value
        self.beam_stride = int(dp('beam_stride', 1).value)  # sous-échantillonnage si CPU limité

        self.memory = DeadEndMemory(self.p.deadend_memory_radius)
        self.nav = LocalNavigator(self.veh, self.p, self.memory)
        self.gridmap = None
        self.last_scan_t = None
        self.last_pose_frame = None
        self.path = Path()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, map_qos)
        self.create_subscription(Bool, '/nav/enable', self.enable_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.theta_pub = self.create_publisher(Float32, '/nav/theta_target', 10)
        self.state_pub = self.create_publisher(String, '/nav/state', 10)
        self.wheel_pub = self.create_publisher(Float32MultiArray, '/nav/wheel_speeds_ideal', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/nav/markers', 10)
        self.path_pub = self.create_publisher(Path, '/nav/path', 10)
        self.create_timer(0.05, self.watchdog)
        self.get_logger().info('local_navigator prêt (autostart=%s)' % self.enabled)

    # ------------------------------------------------------------------ entrées
    def enable_cb(self, msg):
        self.enabled = msg.data
        if not msg.data:
            self.stop('désactivé')

    def map_cb(self, msg):
        if not self.use_map:
            return
        o = msg.info.origin
        self.gridmap = GridMap(
            msg.data,
            msg.info.width,
            msg.info.height,
            msg.info.resolution,
            o.position.x,
            o.position.y,
            quat_to_yaw(o.orientation),
        )

    def lookup_pose(self, stamp):
        """Pose dans map (SLAM) si disponible, sinon dans odom (mode dégradé)."""
        for frame in (self.map_frame, self.odom_frame):
            try:
                tf = self.tf_buffer.lookup_transform(frame, self.base_frame, Time())
                age = (stamp - Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
                if frame == self.map_frame and age > self.p.pose_timeout:
                    continue
                t, q = tf.transform.translation, tf.transform.rotation
                return frame, (t.x, t.y, quat_to_yaw(q))
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                continue
        return None, None

    def scan_cb(self, msg):
        now = self.get_clock().now()
        dt = (
            0.1
            if self.last_scan_t is None
            else max(1e-3, (now - self.last_scan_t).nanoseconds * 1e-9)
        )
        self.last_scan_t = now
        if not self.enabled:
            return
        s = self.beam_stride
        pts = scan_to_points(
            msg.ranges[::s],
            msg.angle_min,
            msg.angle_increment * s,
            msg.range_min,
            msg.range_max,
            self.veh,
        )

        frame, pose = self.lookup_pose(now)
        if frame != self.last_pose_frame and self.last_pose_frame is not None:
            self.get_logger().warn(f'Repère de pose : {self.last_pose_frame} -> {frame}')
            self.memory.points.clear()  # mémoire d'impasses liée au repère courant
        self.last_pose_frame = frame
        gm = self.gridmap if frame == self.map_frame else None
        factor = (
            1.0 if frame == self.map_frame or not self.use_map else self.p.degraded_speed_factor
        )
        if pose is None:
            factor = self.p.degraded_speed_factor

        out = self.nav.step(pts, dt, pose=pose, gridmap=gm, speed_factor=factor)
        self.publish(out, frame, pose, msg.header.stamp)

    def watchdog(self):
        """Perte du LiDAR : arrêt immédiat (le Pico s'arrête aussi s'il ne reçoit plus rien)."""
        if self.last_scan_t is None:
            return
        age = (self.get_clock().now() - self.last_scan_t).nanoseconds * 1e-9
        if age > self.p.scan_timeout and self.enabled:
            self.stop(f'pas de /scan depuis {age:.1f} s')

    def stop(self, reason):
        self.cmd_pub.publish(Twist())
        self.nav.ctrl.reset()
        self.state_pub.publish(String(data=f'SAFE_STOP: {reason}'))
        self.get_logger().warn(f'Arrêt de sécurité : {reason}', throttle_duration_sec=2.0)

    # ------------------------------------------------------------------ sorties
    def publish(self, out, frame, pose, stamp):
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(out.v), float(out.omega)
        self.cmd_pub.publish(cmd)
        self.theta_pub.publish(Float32(data=float(math.degrees(out.theta))))
        self.state_pub.publish(String(data=out.state))
        vl, vr = differential_wheel_speeds(out.v, out.omega, self.veh.track)
        self.wheel_pub.publish(Float32MultiArray(data=[float(vl), float(vr)]))

        if pose is not None:
            ps = PoseStamped()
            ps.header.frame_id, ps.header.stamp = frame, stamp
            ps.pose.position.x, ps.pose.position.y = pose[0], pose[1]
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(pose[2] / 2), math.cos(
                pose[2] / 2
            )
            if self.path.header.frame_id != frame:
                self.path = Path()
            self.path.header.frame_id, self.path.header.stamp = frame, stamp
            self.path.poses.append(ps)
            self.path.poses = self.path.poses[-3000:]
            self.path_pub.publish(self.path)
        self.publish_markers(out, frame, stamp)

    def publish_markers(self, out, frame, stamp):
        ma = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)
        origin = Point(x=self.veh.wheelbase * 0.5)

        def mk(mid, mtype, ns, color, scale=0.02):
            m = Marker()
            m.header.frame_id, m.header.stamp = self.base_frame, stamp
            m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = scale
            m.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
            m.pose.orientation.w = 1.0
            return m

        for i, q in enumerate(out.passages):  # passages candidats (orange) et choisi (vert)
            chosen = out.selected is not None and q.direction == out.selected.direction
            m = mk(
                10 + i,
                Marker.LINE_STRIP,
                'passages',
                (0.1, 0.9, 0.2, 0.9) if chosen else (1.0, 0.6, 0.1, 0.6),
                0.03 if chosen else 0.015,
            )
            r = min(q.depth, self.p.lookahead_cap)
            m.points.append(origin)
            n = max(2, int(abs(q.hi - q.lo) / 0.05))
            for k in range(n + 1):
                a = q.lo + (q.hi - q.lo) * k / n
                m.points.append(Point(x=origin.x + r * math.cos(a), y=r * math.sin(a)))
            m.points.append(origin)
            ma.markers.append(m)
            t = mk(100 + i, Marker.TEXT_VIEW_FACING, 'passage_info', (1, 1, 1, 0.9), 0.08)
            t.pose.position.x = origin.x + 0.6 * math.cos(q.direction)
            t.pose.position.y = 0.6 * math.sin(q.direction)
            t.text = f'{math.degrees(q.direction):.0f}° l={q.width:.2f} d={q.depth:.1f}'
            ma.markers.append(t)

        arrow = mk(1, Marker.ARROW, 'theta_cible', (0.2, 0.4, 1.0, 1.0), 0.04)
        arrow.points = [
            origin,
            Point(x=origin.x + 0.8 * math.cos(out.theta), y=0.8 * math.sin(out.theta)),
        ]
        arrow.scale.y, arrow.scale.z = 0.08, 0.1
        ma.markers.append(arrow)

        fp = mk(2, Marker.LINE_STRIP, 'footprint', (0.9, 0.9, 0.9, 1.0), 0.015)
        v = self.veh
        for x, y in (
            (v.front_overhang, v.width / 2),
            (-v.rear_overhang, v.width / 2),
            (-v.rear_overhang, -v.width / 2),
            (v.front_overhang, -v.width / 2),
            (v.front_overhang, v.width / 2),
        ):
            fp.points.append(Point(x=x, y=y))
        ma.markers.append(fp)

        st = mk(3, Marker.TEXT_VIEW_FACING, 'state', (1, 1, 0.2, 1), 0.12)
        st.pose.position.z = 0.4
        st.text = f'{out.state}  θ={math.degrees(out.theta):.0f}°  v={out.v:.2f}' + (
            '  ÉTROIT' if out.narrow else ''
        )
        ma.markers.append(st)

        for i, (x, y) in enumerate(self.memory.points):  # impasses connues (repère global)
            d = mk(
                200 + i,
                Marker.CYLINDER,
                'impasses',
                (1.0, 0.1, 0.1, 0.5),
                self.p.deadend_memory_radius,
            )
            d.header.frame_id = frame or self.odom_frame
            d.scale.z = 0.05
            d.pose.position.x, d.pose.position.y = x, y
            ma.markers.append(d)
        self.marker_pub.publish(ma)


def main():
    rclpy.init()
    node = LocalNavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
