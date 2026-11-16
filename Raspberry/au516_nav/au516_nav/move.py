import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


HELP = """
Keyboard control of the robot (requires encoder_reader to be running)
---------------------------
   z / up arrow    : faster forward (or slower backward)
   s / down arrow  : slower forward (or faster backward)
   q / left arrow  : steer left
   d / right arrow : steer right
   c               : center the steering
   space           : STOP
   CTRL+C          : quit (stops the robot)
"""

SPEED_STEP = 5      # pulses per 10 ms
SPEED_MIN = -10     # same limits as MotorDriverI2C.clamp() on the Pico
SPEED_MAX = 50

STEER_CENTER = 1500  # servo pulse in us
STEER_STEP = 100
STEER_MIN = 1000
STEER_MAX = 2000

# Arrow keys are sent by the terminal as escape sequences
ARROWS = {"\x1b[A": "z", "\x1b[B": "s", "\x1b[D": "q", "\x1b[C": "d"}


class Move(Node):
    """
    Reads the keyboard and publishes [speed, steering_pulse_us] on /pico_cmd.
    The command is re-sent every 0.1 s because the Pico stops the robot after 0.5 s without command.
    """
    def __init__(self):
        super().__init__("move")
        self.publisher = self.create_publisher(Int32MultiArray, "/pico_cmd", 10)
        self.speed = 0
        self.steering = STEER_CENTER

    def handle_key(self, key):
        key = ARROWS.get(key, key.lower())
        if key == "z":
            self.speed = min(self.speed + SPEED_STEP, SPEED_MAX)
        elif key == "s":
            self.speed = max(self.speed - SPEED_STEP, SPEED_MIN)
        elif key == "q":
            self.steering = min(self.steering + STEER_STEP, STEER_MAX)
        elif key == "d":
            self.steering = max(self.steering - STEER_STEP, STEER_MIN)
        elif key == "c":
            self.steering = STEER_CENTER
        elif key == " ":
            self.speed = 0
        else:
            return
        print(f"\rspeed: {self.speed:4d}   steering: {self.steering} us    ", end="", flush=True)

    def publish(self):
        msg = Int32MultiArray()
        msg.data = [self.speed, self.steering]
        self.publisher.publish(msg)


def get_key(timeout):
    """ Return the key pressed during the timeout ('' if none), arrows included """
    if not select.select([sys.stdin], [], [], timeout)[0]:
        return ""
    key = sys.stdin.read(1)
    if key == "\x1b":
        key += sys.stdin.read(2)
    return key


def main():
    rclpy.init()
    node = Move()
    settings = termios.tcgetattr(sys.stdin)
    print(HELP)

    try:
        tty.setcbreak(sys.stdin.fileno())  # read keys one by one, without pressing Enter
        while rclpy.ok():
            key = get_key(0.1)
            if key == "\x03":  # CTRL+C
                break
            if key:
                node.handle_key(key)
            node.publish()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.speed, node.steering = 0, STEER_CENTER
        node.publish()
        print("\nRobot stopped")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
