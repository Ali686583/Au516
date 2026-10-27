import utime
from machine import Pin, PWM, UART
from motor_driver import MotorDriverI2C

PERIOD_MS = 20  # 50 Hz odometry stream

# Link with the Pi through the FTDI cable (Pi side: /dev/ttyUSB0)
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))

motor_driver = MotorDriverI2C()
servo = PWM(Pin(18))
servo.freq(50)

rx_buffer = b""
next_t = utime.ticks_ms()
while True:
    # Commands from the Pi: "C <speed> <steer_pulse_us>"
    if uart.any():
        rx_buffer += uart.read()
        *lines, rx_buffer = rx_buffer.split(b"\n")
        for line in lines:
            parts = line.split()
            if len(parts) == 3 and parts[0] == b"C":
                motor_driver.set_speed(int(parts[1]))
                pulse = min(max(int(parts[2]), 1000), 2000)
                servo.duty_u16(pulse * 65536 // 20000)

    # Encoder ticks to the Pi
    if utime.ticks_diff(utime.ticks_ms(), next_t) >= 0:
        next_t = utime.ticks_add(next_t, PERIOD_MS)
        enc_L, enc_R = motor_driver.read_encoders_values()
        uart.write("E {} {}\n".format(enc_L, enc_R))
        print("E", enc_L, enc_R)  # also visible in Thonny for debugging
