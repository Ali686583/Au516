"""AU516 M1 : passerelle lecture seule Pico W -> ROS 2.

Lit les trames UART/serie ASCII 'ENC,M1,M3' emises par le Pico W.
Publie les compteurs bruts sur /wheel_encoder_ticks (M1 puis M3).
AUCUNE commande moteur, pas d'odom publiable a ce stade.
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray

from au516_nav.encoder_protocol import parse_encoder_line

try:
    import serial
except ImportError:
    serial = None


class EncoderBridge(Node):
    def __init__(self):
        super().__init__('encoder_bridge')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('watchdog_s', 2.0)
        self.port = str(self.get_parameter('port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.watchdog_s = float(self.get_parameter('watchdog_s').value)
        self.publisher = self.create_publisher(Int64MultiArray, '/wheel_encoder_ticks', 10)
        self.link = None
        self.buffer = bytearray()
        self.last_rx = time.monotonic()
        self.last_attempt = 0.0
        self.last_warning = 0.0
        self.count_frames = 0

        if serial is None:
            raise RuntimeError('Module pyserial absent : sudo apt install python3-serial')
        self.timer = self.create_timer(0.05, self.poll)  # 20 Hz maximum de verification
        self.get_logger().info(
            f'Passerelle encodeurs active sur {self.port} a {self.baudrate} bauds (lecture seule)'
        )

    def _warn_throttled(self, message):
        now = time.monotonic()
        if now - self.last_warning > 2.0:
            self.last_warning = now
            self.get_logger().warning(message)

    def _connect(self):
        now = time.monotonic()
        if now - self.last_attempt < 2.0:
            return
        self.last_attempt = now
        try:
            self.link = serial.Serial(self.port, self.baudrate, timeout=0)
            self.buffer.clear()
            self.last_rx = now
            self.get_logger().info('Port serie connecte')
        except (serial.SerialException, OSError) as exc:
            self._warn_throttled(f'Port serie indisponible ({self.port}) : {exc}')

    def poll(self):
        if self.link is None:
            self._connect()
            return
        try:
            available = self.link.in_waiting
            if available:
                # Le Pico emet des trames de quelques dizaines d'octets.
                # On limite la lecture pour borner le traitement du callback.
                chunk = self.link.read(min(available, 4096))
                self.buffer.extend(chunk)
                if len(self.buffer) > 8192:
                    self.buffer.clear()
                    self._warn_throttled('Trames trop longues : buffer remis a zero')
                    return
                while b'\n' in self.buffer:
                    line, _, remainder = self.buffer.partition(b'\n')
                    self.buffer = bytearray(remainder)
                    ticks = parse_encoder_line(line)
                    if ticks is None:
                        continue
                    self.last_rx = time.monotonic()
                    message = Int64MultiArray()
                    message.data = [ticks[0], ticks[1]]  # M1, M3 (pas encore gauche, droite)
                    self.publisher.publish(message)
                    self.count_frames += 1
                    if self.count_frames % 10 == 0:
                        self.get_logger().info(f'Encodeurs M1={ticks[0]} M3={ticks[1]}')
            if time.monotonic() - self.last_rx > self.watchdog_s:
                self._warn_throttled('Aucune trame ENCODEUR recente (liaison active ?)')
        except (serial.SerialException, OSError) as exc:
            self._warn_throttled(f'Lecture serie interrompue : {exc}')
            try:
                self.link.close()
            except Exception:
                pass
            self.link = None
            self.buffer.clear()

    def destroy_node(self):
        if self.link is not None:
            self.link.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = EncoderBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
