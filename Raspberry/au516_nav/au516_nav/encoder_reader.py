import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class EncoderReader(Node):
    """
    Reads the encoder lines sent by the Pico over USB serial ("E <enc_L> <enc_R>")
    and publishes them on /encoders as [enc_L, enc_R] (total ticks, ~1260 per revolution).
    """
    def __init__(self):
        super().__init__("encoder_reader")

        # FTDI cable wired to the Pico UART (stable name, unlike /dev/ttyUSBx)
        port = self.declare_parameter("port", "/dev/serial/by-id/usb-FTDI_TTL232R_FTBG33U7-if00-port0").value
        baudrate = self.declare_parameter("baudrate", 115200).value

        self.ser = serial.Serial(port, baudrate, timeout=0)
        self.buffer = b""

        self.publisher = self.create_publisher(Int32MultiArray, "/encoders", 10)
        self.create_timer(0.005, self.read_serial)

        # Report every 2 s how many bytes/messages came in, to debug the link
        self.nb_bytes = 0
        self.nb_msgs = 0
        self.create_timer(2.0, self.report)

        self.get_logger().info(f"Listening to the Pico on {port}")

    def report(self):
        if self.nb_msgs == 0:
            self.get_logger().warn(f"No encoder message yet ({self.nb_bytes} bytes received)")
        else:
            self.get_logger().info(f"{self.nb_msgs} messages received, last: {self.last}")

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
            self.publisher.publish(msg)
            self.nb_msgs += 1
            self.last = (enc_L, enc_R)

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = EncoderReader()

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
