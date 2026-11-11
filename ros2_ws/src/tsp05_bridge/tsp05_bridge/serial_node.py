"""ROS 2 bridge for the TSP05 Arduino (sensors + motors over USB serial).

Reads the Arduino's JSON state stream and publishes every sensor / motor
value on its own std_msgs topic (easy to plot in Foxglove), and forwards
command topics (mode, servo, DC, stepper, safety threshold) to the Arduino.
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float32, Int32, String

from . import protocol

try:
    import serial
except ImportError:  # Give a useful error if pyserial is missing.
    serial = None


# (state field or derived key, topic suffix, message type)
STATE_TOPICS = (
    ('mode', 'mode', String),
    ('safety', 'safety/active', Bool),
    ('safety_cm', 'safety/threshold_cm', Float32),

    ('flex_raw', 'sensors/flex/adc_raw', Float32),
    ('flex_adc', 'sensors/flex/adc_filtered', Float32),
    ('flex_kohm', 'sensors/flex/resistance_kohm', Float32),
    ('flex_deg', 'sensors/flex/bend_deg', Float32),
    ('flex_force', 'sensors/flex/force_pct', Float32),
    ('flex_flat_k', 'sensors/flex/cal_flat_kohm', Float32),
    ('flex_bent_k', 'sensors/flex/cal_bent_kohm', Float32),
    ('flex_angle', 'sensors/flex/angle_deg', Float32),

    ('pot_raw', 'sensors/pot/adc_raw', Float32),
    ('pot_v', 'sensors/pot/voltage', Float32),
    ('pot_pct', 'sensors/pot/position_pct', Float32),
    ('pot_deg', 'sensors/pot/angle_deg', Float32),

    ('tof_ok', 'sensors/tof/ok', Bool),
    ('tof_cm', 'sensors/tof/distance_cm', Float32),

    ('sharp_v', 'sensors/sharp/voltage', Float32),
    ('sharp_dist_cm', 'sensors/sharp/distance_cm', Float32),
    ('sharp_valid', 'sensors/sharp/valid', Bool),
    ('sharp_st', 'sensors/sharp/status', String),

    ('btn', 'button/pressed', Bool),
    ('btn_n', 'button/count', Int32),

    ('servo_deg', 'motors/servo/angle_deg', Float32),
    ('step_deg', 'motors/stepper/angle_deg', Float32),
    ('step_tgt', 'motors/stepper/target_deg', Float32),
    ('dc_on', 'motors/dc/connected', Bool),
    ('dc_tgt', 'motors/dc/target_rpm', Float32),
    ('dc_rpm', 'motors/dc/rpm', Float32),
    ('dc_pwm', 'motors/dc/pwm', Float32),
    ('dc_pol', 'motors/dc/polarity', Int32),
    ('dc_stall', 'motors/dc/stalled', Bool),
    ('dc_deg', 'motors/dc/angle_deg', Float32),
    ('dc_deg_tgt', 'motors/dc/target_deg', Float32),
)


# Time to wait after opening the port before sending settings
SETTLE_SEC = 0.5


def to_msg(msg_type, value):
    if msg_type is Bool:
        return Bool(data=bool(value))
    if msg_type is Int32:
        return Int32(data=int(value))
    if msg_type is Float32:
        return Float32(data=float(value) if value is not None else math.nan)
    return String(data=str(value))


class TSP05SerialNode(Node):
    def __init__(self):
        super().__init__('tsp05_serial_node')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('topic_prefix', '/tsp05')
        self.declare_parameter('reconnect_interval_sec', 2.0)
        self.declare_parameter('no_data_timeout_sec', 1.5)
        # < 0 keeps the firmware default (20 cm); 0 disables the interlock
        self.declare_parameter('safety_threshold_cm', -1.0)
        # Mode to request after each Arduino boot: '' keeps firmware default (GUI)
        self.declare_parameter('initial_mode', '')

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        prefix = self.get_parameter('topic_prefix').value.rstrip('/')
        self.reconnect_interval = float(
            self.get_parameter('reconnect_interval_sec').value)
        self.no_data_timeout = float(
            self.get_parameter('no_data_timeout_sec').value)
        self.safety_threshold = float(
            self.get_parameter('safety_threshold_cm').value)
        self.initial_mode = str(self.get_parameter('initial_mode').value)

        if serial is None:
            raise RuntimeError(
                'pyserial is not installed (apt install python3-serial)')

        # Publishers
        self.state_pubs = {
            key: (self.create_publisher(msg_type, f'{prefix}/{suffix}', 10),
                  msg_type)
            for key, suffix, msg_type in STATE_TOPICS
        }
        self.connected_pub = self.create_publisher(
            Bool, f'{prefix}/connected', 10)
        self.state_json_pub = self.create_publisher(
            String, f'{prefix}/state_json', 10)
        self.events_pub = self.create_publisher(
            String, f'{prefix}/events', 10)
        # Force-sensor transfer function in one message, so Foxglove can plot
        # y against x: x = filtered ADC, y = force %, z = servo target deg
        self.transfer_pub = self.create_publisher(
            Point, f'{prefix}/sensors/flex/transfer', 10)
        # ToF -> DC: x = ToF distance cm, y = DC target rpm, z = measured rpm
        self.dc_transfer_pub = self.create_publisher(
            Point, f'{prefix}/motors/dc/transfer', 10)

        # Command subscribers
        self.create_subscription(
            String, f'{prefix}/cmd/mode',
            lambda m: self.send(protocol.format_mode, m.data), 10)
        self.create_subscription(
            Float32, f'{prefix}/cmd/servo_angle',
            lambda m: self.send(protocol.format_servo, m.data), 10)
        self.create_subscription(
            Float32, f'{prefix}/cmd/dc_rpm',
            lambda m: self.send(protocol.format_dc, m.data), 10)
        self.create_subscription(
            Float32, f'{prefix}/cmd/dc_deg',
            lambda m: self.send(protocol.format_dc_deg, m.data), 10)
        self.create_subscription(
            Float32, f'{prefix}/cmd/dc_pwm',
            lambda m: self.send(protocol.format_dc_pwm, m.data), 10)
        self.create_subscription(
            String, f'{prefix}/cmd/dc_polarity',
            lambda m: self.send(protocol.format_dc_polarity, m.data), 10)
        self.create_subscription(
            Float32, f'{prefix}/cmd/stepper_delta',
            lambda m: self.send(protocol.format_step, m.data), 10)
        self.create_subscription(
            String, f'{prefix}/cmd/flex_calibrate',
            lambda m: self.send(protocol.format_calibration, m.data), 10)
        self.create_subscription(
            Float32, f'{prefix}/cmd/safety_threshold',
            lambda m: self.send(protocol.format_safety, m.data), 10)

        self.ser = None
        self.rx_buffer = b''
        self.last_open_attempt = 0.0
        self.last_rx_time = 0.0
        self.connected = False
        self.last_mode = None
        self.opened_at = 0.0
        self.settings_pending = False

        self.create_timer(0.01, self.poll_serial)
        self.create_timer(0.5, self.publish_connection)
        self.get_logger().info(
            f'TSP05 bridge: port={self.port} baud={self.baud} prefix={prefix}')

    # ------------------------------------------------------------------
    # Serial connection
    # ------------------------------------------------------------------

    def open_serial(self):
        now = time.monotonic()
        if now - self.last_open_attempt < self.reconnect_interval:
            return False
        self.last_open_attempt = now

        try:
            # Opening the port may reset the Uno (it then sends a "boot" line)
            # exclusive: fail instead of sharing the port with another reader
            self.ser = serial.Serial(
                self.port, self.baud, timeout=0, exclusive=True)
            self.ser.reset_input_buffer()
        except (serial.SerialException, OSError) as exc:
            self.get_logger().warning(
                f'Cannot open {self.port}: {exc}', throttle_duration_sec=10.0)
            self.ser = None
            return False

        self.rx_buffer = b''
        self.last_rx_time = now
        self.opened_at = now
        # The Uno may not reset when the port opens (no boot line), so the
        # configured settings are applied on every (re)connection.
        self.settings_pending = True
        self.get_logger().info(f'Opened {self.port}, waiting for Arduino')
        return True

    def close_serial(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.set_connected(False)

    def set_connected(self, connected):
        if connected != self.connected:
            self.connected = connected
            if connected:
                self.get_logger().info('Receiving data from Arduino')
            else:
                self.get_logger().warning('No data from Arduino')
            self.publish_connection()

    def publish_connection(self):
        self.connected_pub.publish(Bool(data=self.connected))

    def send(self, formatter, value):
        try:
            line = formatter(value)
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(f'Rejected command: {exc}')
            return
        if self.ser is None or not self.connected:
            self.get_logger().warning(
                f'Arduino not connected, dropped command {line.strip()!r}')
            return
        try:
            # Leading newline ends any partial line on the Arduino (e.g. bytes
            # echoed back by the tty before pyserial switched it to raw mode);
            # the firmware ignores empty lines.
            self.ser.write(b'\n' + line.encode('ascii'))
        except (serial.SerialException, OSError) as exc:
            self.get_logger().warning(f'Serial write failed: {exc}')
            self.close_serial()
            return
        self.get_logger().info(f'Sent {line.strip()!r}')

    # ------------------------------------------------------------------
    # Receiving
    # ------------------------------------------------------------------

    def poll_serial(self):
        if self.ser is None and not self.open_serial():
            return

        try:
            data = self.ser.read(self.ser.in_waiting or 1)
        except (serial.SerialException, OSError) as exc:
            self.get_logger().warning(f'Serial read failed: {exc}')
            self.close_serial()
            return

        now = time.monotonic()
        if data:
            self.rx_buffer += data
            *lines, self.rx_buffer = self.rx_buffer.split(b'\n')
            for line in lines:
                self.handle_line(line, now)
            # Guard against a stream with no newlines
            if len(self.rx_buffer) > 4096:
                self.rx_buffer = b''
        elif self.connected and now - self.last_rx_time > self.no_data_timeout:
            self.set_connected(False)

    def handle_line(self, line, now):
        msg = protocol.parse_line(line)
        if msg is None:
            text = line.decode('utf-8', errors='replace').strip()
            if text:
                self.get_logger().debug(f'Ignoring line: {text[:120]}')
            return

        self.last_rx_time = now
        self.set_connected(True)
        kind = msg['t']

        if kind == 'state':
            self.publish_state(msg, line)
            # Wait a moment after opening: bytes the tty echoes back while the
            # port is being opened can corrupt a command sent immediately.
            if self.settings_pending and now - self.opened_at >= SETTLE_SEC:
                self.settings_pending = False
                self.apply_settings()
        elif kind == 'boot':
            self.get_logger().info(
                f'Arduino booted: mode={msg.get("mode")} '
                f'tof_ok={msg.get("tof_ok")} dc_connected={msg.get("dc_on")}')
            self.events_pub.publish(String(data=line.decode().strip()))
            self.settings_pending = True  # firmware restarted with defaults
        else:  # ack / event
            text = line.decode('utf-8', errors='replace').strip()
            self.events_pub.publish(String(data=text))
            if kind == 'ack' and not msg.get('ok', False):
                self.get_logger().warning(
                    f'Arduino rejected {msg.get("cmd")}: {msg.get("err")}')
            elif kind == 'event':
                self.get_logger().info(
                    f'Event: {msg.get("event")} (mode={msg.get("mode")})')

    def apply_settings(self):
        if self.safety_threshold >= 0.0:
            self.send(protocol.format_safety, self.safety_threshold)
        if self.initial_mode:
            self.send(protocol.format_mode, self.initial_mode)

    def publish_state(self, msg, line):
        for key, (pub, msg_type) in self.state_pubs.items():
            pub.publish(to_msg(msg_type, msg.get(key)))
        self.transfer_pub.publish(Point(
            x=float(msg['flex_angle']), y=float(msg['flex_adc']),
            z=float(msg['flex_kohm'])))
        self.dc_transfer_pub.publish(Point(
            x=float(msg['tof_cm']), y=float(msg['dc_tgt']),
            z=float(msg['dc_rpm'])))
        self.state_json_pub.publish(
            String(data=line.decode('utf-8', errors='replace').strip()))

        if msg['mode'] != self.last_mode:
            self.last_mode = msg['mode']
            self.get_logger().info(f'Mode: {self.last_mode}')

    def destroy_node(self):
        self.close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TSP05SerialNode()
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
