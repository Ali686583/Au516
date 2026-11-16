"""Conversion between the navigation command (v, delta) and the robot command [speed, pulse].

Same conventions as the rest of the package and the Pico (Codes/main.py):
  * speed : motor setpoint in encoder pulses per 10 ms, limited to [-10, 50]
            (MotorDriverI2C.clamp on the Pico, SPEED_MIN / SPEED_MAX in serial_bridge.py);
  * pulse : servo pulse in microseconds, 1000 = 20 deg to the left, 1500 = straight,
            2000 = 20 deg to the right ("Commands list", odometry_node.py: steering_sign = -1).
  * delta : steering angle in rad, positive to the left (ROS convention).

odometry_node.py turns a pulse back into a steering angle with the same formula as
pulse_to_steer(), so what the navigation asks is what the odometry integrates.
"""

from dataclasses import dataclass
import math

from .control import clamp


@dataclass
class DriveCalibration:
    wheel_diameter: float = 0.065  # m (odometry_node.py, Info_complementaires)
    ticks_per_rev: float = 1260.0  # encoder ticks per wheel revolution
    max_steer: float = math.radians(20.0)  # rad, reached at servo_center -/+ servo_span
    servo_center: int = 1500  # us, wheels straight
    servo_span: int = 500  # us added to the center for max_steer
    steering_sign: float = -1.0  # a pulse above 1500 turns to the right (negative angle)
    servo_min: int = 1000
    servo_max: int = 2000
    speed_min: int = -10
    speed_max: int = 50

    @property
    def meters_per_tick(self):
        return math.pi * self.wheel_diameter / self.ticks_per_rev

    @property
    def k_v(self):
        """m/s for one unit of speed (1 pulse per 10 ms = 100 pulses per s)."""
        return 100.0 * self.meters_per_tick

    def to_command(self, v, delta):
        """(v in m/s, delta in rad) -> (speed, pulse) integers ready for serial_bridge."""
        speed = int(round(clamp(v / self.k_v, self.speed_min, self.speed_max)))
        pulse = self.servo_center + self.steering_sign * self.servo_span * delta / self.max_steer
        pulse = int(round(clamp(pulse, self.servo_min, self.servo_max)))
        return speed, pulse

    def speed_to_velocity(self, speed):
        """Motor setpoint -> forward speed in m/s (what the closed-loop driver produces)."""
        return clamp(speed, self.speed_min, self.speed_max) * self.k_v

    def pulse_to_steer(self, pulse):
        """Servo pulse -> steering angle in rad (same formula as odometry_node.py)."""
        pulse = clamp(pulse, self.servo_min, self.servo_max)
        return self.steering_sign * (pulse - self.servo_center) / self.servo_span * self.max_steer
