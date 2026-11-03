
import utime
from machine import Pin, PWM, UART
from motor_driver import MotorDriverI2C


# ==========================================
# Configuration
# ==========================================

UART_BAUDRATE = 115200
PERIOD_MS = 20
WATCHDOG_MS = 500

SERVO_CENTER = 1500
SERVO_MIN = 1000
SERVO_MAX = 2000

MAX_BUFFER = 128


# ==========================================
# Initialisation
# ==========================================

uart = UART(
    0,
    baudrate=UART_BAUDRATE,
    tx=Pin(0),
    rx=Pin(1)
)

motor = MotorDriverI2C()

servo = PWM(Pin(18))
servo.freq(50)


# ==========================================
# Fonctions
# ==========================================

def set_steering(pulse_us):
    pulse_us = max(SERVO_MIN, min(SERVO_MAX, pulse_us))
    duty = pulse_us * 65536 // 20000
    servo.duty_u16(duty)


def stop_robot():
    motor.set_speed(0)
    set_steering(SERVO_CENTER)


def process_command(line):
    parts = line.split()

    if len(parts) != 3 or parts[0] != b"C":
        return False

    try:
        speed = int(parts[1])
        steering = int(parts[2])

        motor.set_speed(speed)
        set_steering(steering)

        return True

    except (ValueError, OSError):
        return False


def send_encoders():
    left, right = motor.read_encoders_values()
    uart.write("E {} {}\n".format(left, right))


# ==========================================
# Boucle principale
# ==========================================

stop_robot()

rx_buffer = b""

next_stream = utime.ticks_ms()
last_command = utime.ticks_ms()

command_active = False

while True:

    now = utime.ticks_ms()

    # Réception des commandes de la Pi
    if uart.any():

        data = uart.read()

        if data:
            rx_buffer += data

            if len(rx_buffer) > MAX_BUFFER:
                rx_buffer = b""

            while b"\n" in rx_buffer:

                line, rx_buffer = rx_buffer.split(b"\n", 1)

                if process_command(line.strip()):
                    last_command = utime.ticks_ms()
                    command_active = True

    # Sécurité : perte de communication
    if command_active:

        if utime.ticks_diff(now, last_command) > WATCHDOG_MS:
            stop_robot()
            command_active = False

    # Transmission automatique des encodeurs
    if utime.ticks_diff(now, next_stream) >= 0:

        next_stream = utime.ticks_add(now, PERIOD_MS)

        try:
            send_encoders()

        except OSError:
            stop_robot()

    utime.sleep_ms(1)
