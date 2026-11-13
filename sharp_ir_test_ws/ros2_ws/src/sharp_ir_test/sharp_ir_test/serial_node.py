"""Read Arduino JSON serial lines and publish Sharp IR ROS 2 topics."""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from .protocol import parse_sensor_line

try:
    import serial
except ImportError:  # Give a useful error if pyserial is missing.
    serial = None


class SharpIRSerialNode(Node):
    def __init__(self):
        super().__init__('sharp_ir_serial_node')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 9600)
        self.declare_parameter('serial_timeout_sec', 0.02)
        self.declare_parameter('reconnect_interval_sec', 2.0)
        self.declare_parameter('no_data_timeout_sec', 1.5)

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.serial_timeout = float(self.get_parameter('serial_timeout_sec').value)
        self.reconnect_interval = float(self.get_parameter('reconnect_interval_sec').value)
        self.no_data_timeout = float(self.get_parameter('no_data_timeout_sec').value)

        self.distance_pub = self.create_publisher(
            Float32, '/lab/sensors/sharp/distance_cm', 10)
        self.valid_pub = self.create_publisher(
            Bool, '/lab/sensors/sharp/valid', 10)
        self.status_pub = self.create_publisher(
            String, '/lab/sensors/sharp/status', 10)
        self.raw_pub = self.create_publisher(
            String, '/lab/sensors/sharp/raw_json', 10)

        self.serial_handle = None
        self.last_open_attempt = 0.0
        self.last_message_time = 0.0
        self.disconnect_reported = False

        if serial is None:
            raise RuntimeError(
                'pyserial is not installed. Install it with: '
                'sudo apt install python3-serial')

        self.timer = self.create_timer(0.02, self.poll_serial)
        self.get_logger().info(
            f'Sharp IR bridge starting: port={self.port}, baud={self.baud}')

    def open_serial_if_needed(self):
        if self.serial_handle is not None and self.serial_handle.is_open:
            return True

        now = time.monotonic()
        if now - self.last_open_attempt < self.reconnect_interval:
            return False
        self.last_open_attempt = now

        try:
            self.serial_handle = serial.Serial(
                self.port, self.baud, timeout=self.serial_timeout)
            self.last_message_time = now
            self.disconnect_reported = False
            self.get_logger().info(f'Connected to Arduino on {self.port}')
            return True
        except (serial.SerialException, OSError) as exc:
            self.get_logger().warning(f'Cannot open {self.port}: {exc}')
            self.serial_handle = None
            return False

    def publish_invalid(self, status):
        self.distance_pub.publish(Float32(data=float('nan')))
        self.valid_pub.publish(Bool(data=False))
        self.status_pub.publish(String(data=status))

    def poll_serial(self):
        if not self.open_serial_if_needed():
            return

        try:
            line = self.serial_handle.readline().decode('utf-8', errors='replace').strip()
        except (serial.SerialException, OSError) as exc:
            self.get_logger().warning(f'Serial read failed: {exc}')
            self.close_serial()
            self.publish_invalid('SERIAL_DISCONNECTED')
            return

        if line:
            self.last_message_time = time.monotonic()
            parsed = parse_sensor_line(line)
            if parsed is None:
                self.get_logger().warning(f'Ignoring non-JSON serial line: {line[:120]}')
                return

            self.raw_pub.publish(String(data=parsed['raw_json']))
            self.valid_pub.publish(Bool(data=parsed['valid']))
            self.status_pub.publish(String(data=parsed['status']))

            if parsed['valid']:
                self.distance_pub.publish(
                    Float32(data=float(parsed['distance_cm'])))
            else:
                self.distance_pub.publish(Float32(data=float('nan')))

        elif (not self.disconnect_reported and self.last_message_time > 0.0
              and time.monotonic() - self.last_message_time > self.no_data_timeout):
            self.publish_invalid('NO_SERIAL_DATA')
            self.disconnect_reported = True

    def close_serial(self):
        if self.serial_handle is not None:
            try:
                self.serial_handle.close()
            except Exception:
                pass
        self.serial_handle = None

    def destroy_node(self):
        self.close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SharpIRSerialNode()
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
