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

        port = self.declare_parameter("port", "/dev/ttyACM0").value
        baudrate = self.declare_parameter("baudrate", 115200).value

        self.ser = serial.Serial(port, baudrate, timeout=0)
        self.buffer = b""

        self.publisher = self.create_publisher(Int32MultiArray, "/encoders", 10)
        self.create_timer(0.005, self.read_serial)

        self.get_logger().info(f"Listening to the Pico on {port}")

    def read_serial(self):
        self.buffer += self.ser.read(self.ser.in_waiting or 1)

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
