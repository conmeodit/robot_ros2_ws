"""ROS 2 node that bridges Serial USB ↔ Arduino Mega.

Responsibilities:
- Subscribe ``/cmd_vel`` (Twist) → send ``CMD_VEL,v,w`` to Arduino.
- Read serial telemetry (``$TELE,...``) → publish ``/base/status`` (String)
  and ``/imu/data_raw`` (sensor_msgs/Imu).
- Safety: if no ``/cmd_vel`` for ``cmd_vel_timeout_sec`` → send ``S`` (stop).
"""

import glob
import os
import re
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from robot_serial_bridge.packet_builder import cmd_vel as build_cmd_vel, stop as build_stop
from robot_serial_bridge.packet_parser import parse_telemetry, TelemetryFrame

try:
    import serial
except ImportError:
    serial = None


# ---------------------------------------------------------------------------
# Port auto-detection helpers
# ---------------------------------------------------------------------------

def _port_score(port_path: str) -> int:
    path = port_path.lower()
    score = 0
    if '/dev/serial/by-id/' in path:
        score += 100
    if '/dev/serial/by-path/' in path:
        score += 80
    if '/dev/ttyusb' in path:
        score += 20
    if '/dev/ttyacm' in path:
        score += 10
    arduino_hints = ('arduino', 'mega', 'ch340', '1a86', 'wch')
    for hint in arduino_hints:
        if hint in path:
            score += 30
    lidar_hints = ('lidar', 'rplidar', 'slamtec', 'cp210', 'silicon_labs', 'sllidar')
    for hint in lidar_hints:
        if hint in path:
            score -= 60
    return score


def _detect_mega_port() -> Optional[str]:
    candidates = []
    candidates.extend(sorted(glob.glob('/dev/serial/by-id/*')))
    candidates.extend(sorted(glob.glob('/dev/serial/by-path/*')))
    candidates.extend(sorted(glob.glob('/dev/ttyUSB*')))
    candidates.extend(sorted(glob.glob('/dev/ttyACM*')))
    seen = set()
    unique = []
    for c in candidates:
        real = os.path.realpath(c)
        if real not in seen:
            seen.add(real)
            unique.append(c)
    if not unique:
        return None
    ranked = sorted(unique, key=lambda p: (_port_score(p), p), reverse=True)
    return ranked[0]


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class MegaBridgeNode(Node):
    def __init__(self):
        super().__init__('mega_bridge_node')

        self.declare_parameter('port', 'auto')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('status_topic', '/base/status')
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('write_rate_hz', 20.0)
        self.declare_parameter('cmd_vel_timeout_sec', 0.5)
        self.declare_parameter('read_enabled', True)
        self.declare_parameter('imu_frame_id', 'base_link')
        self.declare_parameter('serial_startup_delay_sec', 2.0)

        port_param = str(self.get_parameter('port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.status_topic = str(self.get_parameter('status_topic').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.write_rate_hz = max(1.0, float(self.get_parameter('write_rate_hz').value))
        self.cmd_vel_timeout_sec = max(0.1, float(self.get_parameter('cmd_vel_timeout_sec').value))
        self.read_enabled = bool(self.get_parameter('read_enabled').value)
        self.imu_frame_id = str(self.get_parameter('imu_frame_id').value)
        self.serial_startup_delay_sec = max(
            0.0, float(self.get_parameter('serial_startup_delay_sec').value)
        )

        # Resolve port
        if port_param == 'auto':
            self.port = _detect_mega_port()
            if self.port:
                self.get_logger().info(f'Auto-detected Arduino port: {self.port}')
            else:
                self.get_logger().error('No Arduino serial port detected!')
                self.port = '/dev/ttyUSB0'
        else:
            self.port = port_param

        # Serial connection
        self.ser: Optional['serial.Serial'] = None
        self._open_serial()

        # State
        self._cmd_linear = 0.0
        self._cmd_angular = 0.0
        self._last_cmd_ns = self.get_clock().now().nanoseconds
        self._lock = threading.Lock()
        self._rx_buffer = bytearray()
        self._telemetry_count = 0

        # Publishers
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.imu_pub = self.create_publisher(Imu, self.imu_topic, 10)

        # Subscriber
        self.cmd_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self._cmd_vel_cb, 10
        )

        # Timer: write commands at fixed rate
        self._write_timer = self.create_timer(1.0 / self.write_rate_hz, self._write_cb)

        # Timer: read serial at ~100 Hz
        if self.read_enabled:
            self._read_timer = self.create_timer(0.01, self._read_cb)

        self.get_logger().info(
            f'MegaBridgeNode ready: port={self.port}, baud={self.baudrate}, '
            f'cmd_vel={self.cmd_vel_topic}'
        )

    def _open_serial(self):
        if serial is None:
            self.get_logger().error('pyserial not installed! Run: pip install pyserial')
            return
        try:
            self.ser = serial.Serial(
                self.port, self.baudrate,
                timeout=0.0,
                write_timeout=0.1,
            )
            self.get_logger().info(f'Serial port opened: {self.port}')
            if self.serial_startup_delay_sec > 0.0:
                self.get_logger().info(
                    f'Waiting {self.serial_startup_delay_sec:.1f}s for Arduino reset...'
                )
                time.sleep(self.serial_startup_delay_sec)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except Exception as e:
            self.get_logger().error(f'Failed to open {self.port}: {e}')
            self.ser = None

    def _cmd_vel_cb(self, msg: Twist):
        with self._lock:
            self._cmd_linear = float(msg.linear.x)
            self._cmd_angular = float(msg.angular.z)
            self._last_cmd_ns = self.get_clock().now().nanoseconds

    def _write_cb(self):
        if self.ser is None or not self.ser.is_open:
            return

        now_ns = self.get_clock().now().nanoseconds
        with self._lock:
            age_sec = (now_ns - self._last_cmd_ns) * 1e-9
            if age_sec > self.cmd_vel_timeout_sec:
                data = build_stop()
            else:
                data = build_cmd_vel(self._cmd_linear, self._cmd_angular)

        try:
            self.ser.write(data.encode('ascii'))
        except Exception as e:
            self.get_logger().warn(f'Serial write error: {e}')

    def _read_cb(self):
        if self.ser is None or not self.ser.is_open:
            return

        try:
            waiting = self.ser.in_waiting
            if waiting <= 0:
                return

            chunk = self.ser.read(waiting)
            if not chunk:
                return

            self._rx_buffer.extend(chunk)
            if len(self._rx_buffer) > 4096:
                self.get_logger().warn('Serial RX buffer overflow; dropping stale bytes')
                del self._rx_buffer[:-512]

            while b'\n' in self._rx_buffer:
                raw_line, _, rest = self._rx_buffer.partition(b'\n')
                self._rx_buffer = bytearray(rest)
                line = raw_line.decode('ascii', errors='replace').strip()
                if line:
                    self._handle_serial_line(line)

        except Exception as e:
            self.get_logger().warn(f'Serial read error: {e}')

    def _handle_serial_line(self, line: str):
        status_msg = String()
        status_msg.data = line
        self.status_pub.publish(status_msg)

        frame = parse_telemetry(line)
        if frame is None:
            return

        self._telemetry_count += 1
        if self._telemetry_count == 1:
            self.get_logger().info(f'First telemetry frame received: {line}')
        self._publish_imu(frame)

    def _publish_imu(self, frame: TelemetryFrame):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.imu_frame_id

        # No orientation from gyro+accel alone
        imu_msg.orientation_covariance[0] = -1.0

        # Linear acceleration
        imu_msg.linear_acceleration.x = frame.accel_x
        imu_msg.linear_acceleration.y = frame.accel_y
        imu_msg.linear_acceleration.z = frame.accel_z
        imu_msg.linear_acceleration_covariance[0] = 0.5
        imu_msg.linear_acceleration_covariance[4] = 0.5
        imu_msg.linear_acceleration_covariance[8] = 0.5

        # Angular velocity
        imu_msg.angular_velocity.x = frame.gyro_x
        imu_msg.angular_velocity.y = frame.gyro_y
        imu_msg.angular_velocity.z = frame.gyro_z
        imu_msg.angular_velocity_covariance[0] = 0.02
        imu_msg.angular_velocity_covariance[4] = 0.02
        imu_msg.angular_velocity_covariance[8] = 0.02

        self.imu_pub.publish(imu_msg)

    def destroy_node(self):
        # Send stop before shutting down
        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.write(build_stop().encode('ascii'))
                self.ser.flush()
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MegaBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
