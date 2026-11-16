"""Communication tests between the real nodes of the package (needs ROS 2, skipped otherwise).

The Pico is replaced by the emulator of sim_2d (FakePico) on a pseudo-terminal, so serial_bridge
really opens a serial port and speaks the protocol of Codes/main.py.

    colcon test --packages-select au516_nav
    python3 -m pytest test/test_integration_ros.py
"""

import math
import os
import tempfile
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from au516_nav.drive_mux import DriveMux, latched_qos  # noqa: E402,I100
from au516_nav.local_navigator import LocalNavigatorNode  # noqa: E402
from au516_nav.nav_core import DriveCalibration, VehicleParams  # noqa: E402
from au516_nav.nav_core.sim2d import Car, World  # noqa: E402
from au516_nav.nav_core.worlds import SCENARIOS  # noqa: E402
from au516_nav.odometry_node import OdometryNode  # noqa: E402
from au516_nav.serial_bridge import SerialBridge  # noqa: E402
from au516_nav.sim_2d import FakePico  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from std_msgs.msg import Bool, Int32MultiArray, String  # noqa: E402

CAL = DriveCalibration()
L = VehicleParams().wheelbase


@pytest.fixture(scope='module')
def pico():
    """Emulated Pico; serial_bridge gets its port through the global `port` parameter."""
    link = os.path.join(tempfile.mkdtemp(), 'fake_pico')
    fake = FakePico(link, CAL)
    rclpy.init(args=['--ros-args', '-p', f'port:={link}'])
    yield fake
    rclpy.shutdown()
    fake.close()


class Probe(Node):
    """Publishes test inputs and records everything the nodes under test publish."""

    def __init__(self):
        super().__init__('probe')
        self.msgs = {}
        for topic, mtype, qos in (
            ('/cmd_drive_mux', Int32MultiArray, 10),
            ('/encoders', Int32MultiArray, 10),
            ('/odom', Odometry, 10),
            ('/cmd_vel', Twist, 10),
            ('/nav/state', String, 10),
            ('/drive/mode', String, latched_qos()),
            ('/nav/enabled', Bool, latched_qos()),
        ):
            self.msgs[topic] = []
            self.create_subscription(
                mtype, topic, lambda m, t=topic: self.msgs[t].append(m), qos
            )
        self.drive = self.create_publisher(Int32MultiArray, '/cmd_drive', 10)
        self.twist = self.create_publisher(Twist, '/cmd_vel', 10)
        self.enabled = self.create_publisher(Bool, '/nav/enabled', latched_qos())
        self.enable = self.create_publisher(Bool, '/nav/enable', 10)
        self.scan = self.create_publisher(LaserScan, '/scan', 10)

    def send_drive(self, speed, pulse):
        self.drive.publish(Int32MultiArray(data=[speed, pulse]))

    def send_twist(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.twist.publish(t)

    def last(self, topic):
        return self.msgs[topic][-1] if self.msgs[topic] else None

    def clear(self):
        for v in self.msgs.values():
            v.clear()


@pytest.fixture
def system(pico):
    """Executor + probe; nodes added by a test are destroyed at its end."""
    ex = SingleThreadedExecutor()
    probe = Probe()
    nodes = [probe]
    ex.add_node(probe)

    def add(node):
        nodes.append(node)
        ex.add_node(node)
        return node

    def spin(seconds, every=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            ex.spin_once(timeout_sec=0.005)
            if every:
                every()

    yield add, spin, probe
    for n in reversed(nodes):
        ex.remove_node(n)
        n.destroy_node()


def data(msg):
    return list(msg.data) if msg is not None else None


# ---------------------------------------------------------------------------------- drive_mux
def test_mux_manual_mode_forwards_the_keyboard(system):
    add, spin, probe = system
    add(DriveMux())
    spin(0.3)
    assert probe.last('/drive/mode').data == 'MANUAL'
    probe.send_drive(20, 1300)
    spin(0.2)
    assert data(probe.last('/cmd_drive_mux')) == [20, 1300]
    probe.send_twist(0.3, 0.5)  # navigation output ignored while it is off
    spin(0.2)
    assert data(probe.last('/cmd_drive_mux')) == [20, 1300]


def test_mux_auto_mode_converts_cmd_vel_and_ignores_keyboard(system):
    add, spin, probe = system
    add(DriveMux())
    spin(0.2)
    probe.enabled.publish(Bool(data=True))
    spin(0.3)
    assert probe.last('/drive/mode').data == 'AUTO'
    assert data(probe.last('/cmd_drive_mux')) == [0, 1500]  # stop on mode change

    v, delta = 0.3, 0.2  # 11.5 deg to the left
    probe.send_twist(v, v * math.tan(delta) / L)
    spin(0.1)
    expected = list(CAL.to_command(v, delta))
    assert data(probe.last('/cmd_drive_mux')) == expected
    assert expected[1] < 1500  # left = pulse below 1500

    probe.send_drive(40, 2000)  # keyboard ignored in AUTO
    spin(0.1)
    assert data(probe.last('/cmd_drive_mux')) == expected

    spin(0.5)  # no more /cmd_vel: stop after cmd_timeout
    assert data(probe.last('/cmd_drive_mux')) == [0, 1500]

    probe.send_twist(float('nan'), 0.0)  # invalid command: stop
    spin(0.1)
    assert data(probe.last('/cmd_drive_mux')) == [0, 1500]

    probe.enabled.publish(Bool(data=False))
    spin(0.3)
    assert probe.last('/drive/mode').data == 'MANUAL'
    probe.send_drive(15, 1500)
    spin(0.1)
    assert data(probe.last('/cmd_drive_mux')) == [15, 1500]


# ------------------------------------------------------------------- serial_bridge + Pico
def test_serial_bridge_talks_to_the_pico(system, pico):
    add, spin, probe = system
    add(SerialBridge())

    def pico_period():
        pico.receive(time.monotonic())

    for _ in range(5):
        probe.send_drive(12, 1400)
        spin(0.05, pico_period)
    assert (pico.speed, pico.pulse) == (12, 1400)

    pico.enc = [0.0, 0.0]
    pico.send_encoders(50)
    spin(0.2, pico_period)
    assert data(probe.last('/encoders')) == [-50, 50]  # left wheel counts down (M1 = -speed)

    spin(0.6, pico_period)  # no more /cmd_drive: serial_bridge sends "C 0 1500"
    assert (pico.speed, pico.pulse) == (0, 1500)


# ------------------------------------------------- whole chain: cmd_vel -> Pico -> odometry
def test_navigation_command_reaches_the_pico_and_the_odometry(system, pico):
    add, spin, probe = system
    add(DriveMux(parameter_overrides=[Parameter('output_topic', value='/cmd_drive')]))
    add(SerialBridge())
    add(OdometryNode())
    spin(0.3)
    probe.enabled.publish(Bool(data=True))
    spin(0.3)

    v, delta = 0.3, 0.2
    speed, pulse = CAL.to_command(v, delta)
    state = {'next_twist': 0.0, 'next_enc': 0.0}

    def loop():
        now = time.monotonic()
        if now >= state['next_twist']:  # local_navigator publishes at the scan rate
            probe.send_twist(v, v * math.tan(delta) / L)
            state['next_twist'] = now + 0.1
        if now >= state['next_enc']:  # Pico: commands in, encoders out every 20 ms
            pico.receive(now)
            pico.send_encoders(pico.speed * 100 * 0.02)
            state['next_enc'] = now + 0.02

    spin(2.0, loop)
    assert (pico.speed, pico.pulse) == (speed, pulse)
    odom = probe.last('/odom')
    assert odom is not None
    yaw = 2 * math.atan2(odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
    dist = math.hypot(odom.pose.pose.position.x, odom.pose.pose.position.y)
    assert odom.pose.pose.position.x > 0.2  # forward
    assert yaw > 0.0  # turning left, as asked by the navigation
    # arc of a circle: yaw = s * tan(delta) / L, the chord is slightly shorter than the arc
    assert yaw == pytest.approx(dist * math.tan(CAL.pulse_to_steer(pulse)) / L, rel=0.1)


# ----------------------------------------------------------------------------- local_navigator
def make_scan(car, world):
    lx, ly, lyaw = car.laser_pose()
    r, amin, inc = world.raycast(lx, ly, lyaw, n_beams=450)
    msg = LaserScan()
    msg.header.frame_id = 'laser'
    msg.angle_min, msg.angle_increment = amin, inc
    msg.angle_max = amin + inc * (len(r) - 1)
    msg.range_min, msg.range_max = 0.05, 12.0
    msg.ranges = [float(x) for x in r]
    return msg


def test_local_navigator_drives_reports_its_state_and_stops_without_lidar(system):
    add, spin, probe = system
    add(LocalNavigatorNode())  # autostart default: true
    spin(0.3)
    assert probe.last('/nav/enabled').data is True

    segs, (x, y, yaw) = SCENARIOS['A_straight']()
    world, car = World(segs), Car(VehicleParams(), x, y, yaw)
    for _ in range(10):
        probe.scan.publish(make_scan(car, world))
        spin(0.1)
    cmd = probe.last('/cmd_vel')
    assert cmd is not None and cmd.linear.x > 0.0  # free corridor: it drives
    assert probe.last('/nav/state').data == 'DRIVE'

    spin(0.8)  # LiDAR lost
    assert probe.last('/nav/state').data.startswith('SAFE_STOP')
    assert probe.last('/cmd_vel').linear.x == 0.0

    probe.enable.publish(Bool(data=False))
    spin(0.3)
    assert probe.last('/nav/enabled').data is False
