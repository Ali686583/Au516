import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32MultiArray


HELP = """
Keyboard control of the robot (requires serial_bridge to be running)
---------------------------
   z / up arrow    : faster forward (or slower backward)
   s / down arrow  : slower forward (or faster backward)
   q / left arrow  : steer left
   d / right arrow : steer right
   c               : center the steering
   space           : STOP
   n               : start the autonomous navigation (robot_launch.py),
                     then any other key above stops it and takes the control back
   CTRL+C          : quit (stops the robot)
"""

SPEED_STEP = 5      # pulses per 10 ms
SPEED_MIN = -10     # same limits as MotorDriverI2C.clamp() on the Pico
SPEED_MAX = 50

STEER_CENTER = 1500  # servo pulse in us
STEER_STEP = 100
STEER_MIN = 1000     # 20 deg to the left
STEER_MAX = 2000     # 20 deg to the right

# Arrow keys are sent by the terminal as escape sequences
ARROWS = {"\x1b[A": "z", "\x1b[B": "s", "\x1b[D": "q", "\x1b[C": "d"}
DRIVE_KEYS = ("z", "s", "q", "d", "c", " ")


class Move(Node):
    """
    Reads the keyboard and publishes [speed, steering_pulse_us] on /cmd_drive (read by serial_bridge).
    The command is re-sent every 0.1 s because serial_bridge stops the robot after 0.3 s without command.

    With robot_launch.py, drive_mux only forwards these commands while the autonomous navigation
    is off. "n" starts the navigation (/nav/enable); while it drives, any driving key stops it
    first (manual takeover), then applies the key from a stopped car.
    """
    def __init__(self):
        super().__init__("move")
        self.publisher = self.create_publisher(Int32MultiArray, "/cmd_drive", 10)
        self.speed = 0
        self.steering = STEER_CENTER

        # State of local_navigator (latched topic: received even if move starts later)
        self.nav_active = False
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.nav_enable_pub = self.create_publisher(Bool, "/nav/enable", 10)
        self.create_subscription(Bool, "/nav/enabled", self.nav_state_cb, latched)

    def nav_state_cb(self, msg):
        if msg.data == self.nav_active:
            return
        self.nav_active = msg.data
        # When the control comes back, start again from a stopped car
        self.speed, self.steering = 0, STEER_CENTER
        if msg.data:
            state = "ON (any driving key takes the control back)"
        else:
            state = "OFF, manual control"
        print(f"\rAutonomous navigation {state}          ", end="", flush=True)

    def set_navigation(self, enabled):
        self.speed, self.steering = 0, STEER_CENTER
        self.nav_enable_pub.publish(Bool(data=enabled))

    def handle_key(self, key):
        key = ARROWS.get(key, key.lower())
        if key == "n":
            self.set_navigation(True)
            print("\rAutonomous navigation requested          ", end="", flush=True)
            return
        if key in DRIVE_KEYS and self.nav_active:
            # Manual takeover: the navigation is stopped before applying the key
            self.set_navigation(False)
            self.nav_active = False
        if key == "z":
            self.speed = min(self.speed + SPEED_STEP, SPEED_MAX)
        elif key == "s":
            self.speed = max(self.speed - SPEED_STEP, SPEED_MIN)
        elif key == "q":
            self.steering = max(self.steering - STEER_STEP, STEER_MIN)
        elif key == "d":
            self.steering = min(self.steering + STEER_STEP, STEER_MAX)
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
            rclpy.spin_once(node, timeout_sec=0.0)  # receive /nav/enabled
            node.publish()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        if node.nav_active:
            node.set_navigation(False)
        node.speed, node.steering = 0, STEER_CENTER
        node.publish()
        print("\nRobot stopped")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
