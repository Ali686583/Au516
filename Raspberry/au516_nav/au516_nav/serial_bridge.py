import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray


SERVO_CENTER = 1500
SERVO_MIN = 1000
SERVO_MAX = 2000

# Same limits as MotorDriverI2C.clamp on the Pico
SPEED_MIN = -10
SPEED_MAX = 50


class SerialBridge(Node):
    """
    Only node that owns the serial link with the Pico (one port cannot be opened twice).

    Pico -> Pi : "E <enc_L> <enc_R>" lines, published on /encoders as [enc_L, enc_R]
                 (total ticks, ~1260 per revolution).
    Pi -> Pico : "C <speed> <steering_pwm>" lines, built from /cmd_drive [speed, steering_pwm].
                 The steering really applied is published on /steering_pwm for the odometry.
    """
    def __init__(self):
        super().__init__("serial_bridge")

        # FTDI cable wired to the Pico UART (stable name, unlike /dev/ttyUSBx)
        port = self.declare_parameter("port", "/dev/serial/by-id/usb-FTDI_TTL232R_FTBG33U7-if00-port0").value
        baudrate = self.declare_parameter("baudrate", 115200).value
        # Without a new /cmd_drive for this long, the car is stopped
        self.cmd_timeout = self.declare_parameter("cmd_timeout", 0.3).value

        self.ser = serial.Serial(port, baudrate, timeout=0)
        self.buffer = b""

        self.encoder_pub = self.create_publisher(Int32MultiArray, "/encoders", 10)
        self.steering_pub = self.create_publisher(Int32, "/steering_pwm", 10)
        self.create_subscription(Int32MultiArray, "/cmd_drive", self.cmd_callback, 10)

        self.speed = 0
        self.steering = SERVO_CENTER
        self.last_cmd_time = None
        self.command_active = False

        self.create_timer(0.005, self.read_serial)
        # 20 Hz, well below the 500 ms watchdog of the Pico
        self.create_timer(0.05, self.send_command)

        # Report every 2 s how many bytes/messages came in, to debug the link
        self.nb_bytes = 0
        self.nb_msgs = 0
        self.create_timer(2.0, self.report)

        self.get_logger().info(f"Talking to the Pico on {port}")

    def cmd_callback(self, msg):
        if len(msg.data) != 2:
            self.get_logger().warn("/cmd_drive expects [speed, steering_pwm]")
            return

        self.speed = max(SPEED_MIN, min(SPEED_MAX, msg.data[0]))
        self.steering = max(SERVO_MIN, min(SERVO_MAX, msg.data[1]))
        self.last_cmd_time = self.get_clock().now()

    def send_command(self):
        fresh = (
            self.last_cmd_time is not None
            and (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9 < self.cmd_timeout
        )

        if fresh:
            self.ser.write(f"C {self.speed} {self.steering}\n".encode())
            self.command_active = True
        else:
            if self.command_active:
                # Stop right away instead of waiting for the Pico watchdog
                self.ser.write(f"C 0 {SERVO_CENTER}\n".encode())
                self.command_active = False
                self.get_logger().warn("No /cmd_drive anymore, car stopped")
            # The Pico recenters the servo when it stops
            self.steering = SERVO_CENTER

        self.steering_pub.publish(Int32(data=self.steering))

    def report(self):
        if self.nb_msgs == 0:
            self.get_logger().warn(f"No encoder message yet ({self.nb_bytes} bytes received)")
        else:
            self.get_logger().info(
                f"{self.nb_msgs} messages received, last: {self.last} | "
                f"cmd: speed={self.speed if self.command_active else 0} steering={self.steering}"
            )

    def read_serial(self):
        data = self.ser.read(self.ser.in_waiting or 1)
        self.nb_bytes += len(data)
        self.buffer += data

        # Only parse complete lines, keep the partial one for the next call
        *lines, self.buffer = self.buffer.split(b"\n")
        for line in lines:
            parts = line.decode(errors="ignore").split()
            if len(parts) != 3 or parts[0] != "E":
                continue
            try:
                enc_L, enc_R = int(parts[1]), int(parts[2])
            except ValueError:
                continue

            msg = Int32MultiArray()
            msg.data = [enc_L, enc_R]
            self.encoder_pub.publish(msg)
            self.nb_msgs += 1
            self.last = (enc_L, enc_R)

    def destroy_node(self):
        try:
            self.ser.write(f"C 0 {SERVO_CENTER}\n".encode())
        except serial.SerialException:
            pass
        self.ser.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = SerialBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
