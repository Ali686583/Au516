import sys
import select
import utime
from machine import Pin, PWM
from motor_driver import MotorDriverI2C

PERIOD_MS = 20  # 50 Hz odometry stream

motor_driver = MotorDriverI2C()
servo = PWM(Pin(18))
servo.freq(50)

poll = select.poll()
poll.register(sys.stdin, select.POLLIN)

next_t = utime.ticks_ms()
while True:
    # Commands from the Pi: "C <speed> <steer_pulse_us>"
    while poll.poll(0):
        parts = sys.stdin.readline().split()
        if len(parts) == 3 and parts[0] == "C":
            motor_driver.set_speed(int(parts[1]))
            pulse = min(max(int(parts[2]), 1000), 2000)
            servo.duty_u16(pulse * 65536 // 20000)

    # Encoder ticks to the Pi
    if utime.ticks_diff(utime.ticks_ms(), next_t) >= 0:
        next_t = utime.ticks_add(next_t, PERIOD_MS)
        enc_L, enc_R = motor_driver.read_encoders_values()
        print("E", enc_L, enc_R)
