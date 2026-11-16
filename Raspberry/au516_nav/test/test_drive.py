"""
Consistency between the navigation command and the robot code (no ROS needed).

Run: python3 -m pytest test/test_drive.py -v
"""

import math
import os
import re
import sys

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, '..'))
from au516_nav.nav_core import DriveCalibration, twist_to_ackermann, VehicleParams  # noqa: E402
import pytest  # noqa: E402,I100

CAL = DriveCalibration()


def team_value(filename, name):
    """Read `self.<name> = <number>` in a node of the package (the real values of the car)."""
    with open(os.path.join(HERE, '..', 'au516_nav', filename)) as f:
        m = re.search(r'self\.%s\s*=\s*(-?[\d.]+)' % name, f.read())
    assert m, f'{name} not found in {filename}'
    return float(m.group(1))


def odometry_steering(pulse):
    """Compute the steering angle with the formula of OdometryNode.encoder_callback."""
    ratio = (pulse - team_value('odometry_node.py', 'steering_center')) / 500.0
    return math.radians(
        ratio
        * team_value('odometry_node.py', 'max_steering_deg')
        * team_value('odometry_node.py', 'steering_sign')
    )


def test_calibration_matches_odometry_node():
    assert CAL.wheel_diameter == team_value('odometry_node.py', 'wheel_diameter')
    assert CAL.ticks_per_rev == team_value('odometry_node.py', 'ticks_per_rev')
    assert VehicleParams().wheelbase == team_value('odometry_node.py', 'wheelbase')
    assert math.isclose(VehicleParams().max_steer, CAL.max_steer, abs_tol=1e-5)


@pytest.mark.parametrize('pulse', [1000, 1100, 1250, 1500, 1600, 1800, 2000])
def test_pulse_to_steer_is_the_odometry_formula(pulse):
    assert math.isclose(CAL.pulse_to_steer(pulse), odometry_steering(pulse), abs_tol=1e-9)


def test_servo_convention():
    # 1000 us = 20 deg to the left (positive angle in ROS), 2000 us = 20 deg to the right
    assert CAL.to_command(0.0, math.radians(20))[1] == 1000
    assert CAL.to_command(0.0, 0.0)[1] == 1500
    assert CAL.to_command(0.0, math.radians(-20))[1] == 2000
    assert CAL.to_command(0.0, math.radians(45))[1] == 1000  # clamped like the servo


@pytest.mark.parametrize('deg', [-20, -12.5, -3, 0, 4.7, 10, 20])
def test_steering_round_trip(deg):
    pulse = CAL.to_command(0.3, math.radians(deg))[1]
    # one microsecond of servo = 0.04 deg
    assert abs(math.degrees(CAL.pulse_to_steer(pulse)) - deg) < 0.05


def test_speed_conversion_and_limits():
    assert math.isclose(CAL.k_v, 100 * math.pi * 0.065 / 1260)
    assert CAL.to_command(0.6, 0.0)[0] == 37  # v_max of nav.yaml
    assert CAL.to_command(5.0, 0.0)[0] == 50  # MotorDriverI2C.clamp
    assert CAL.to_command(-0.15, 0.0)[0] == -9  # reverse speed of the U-turn
    assert CAL.to_command(-5.0, 0.0)[0] == -10
    assert math.isclose(CAL.speed_to_velocity(50), 50 * CAL.k_v)


def test_twist_from_navigator_gives_back_the_steering():
    veh = VehicleParams()
    for v, delta in ((0.3, 0.2), (0.12, -0.3), (-0.15, 0.34)):
        omega = v * math.tan(delta) / veh.wheelbase  # what local_navigator publishes
        _, d = twist_to_ackermann(v, omega, veh)
        assert math.isclose(d, delta, abs_tol=1e-9)


def test_navigation_config_uses_the_car_values():
    yaml = pytest.importorskip('yaml')
    with open(os.path.join(HERE, '..', 'config', 'nav.yaml')) as f:
        p = yaml.safe_load(f)['local_navigator']['ros__parameters']
    assert p['vehicle']['wheelbase'] == team_value('odometry_node.py', 'wheelbase')
    assert math.isclose(p['vehicle']['max_steer'], math.radians(20), abs_tol=1e-4)
    assert p['autostart'] is False  # the real robot waits for /nav/enable
    assert p['nav']['v_max'] < CAL.speed_to_velocity(CAL.speed_max)
