"""ROS 2 serial bridge for the Arduino Mega base controller.

This node is kept as a compatibility bridge. The main real robot path should
use ``vacuum_driver real_driver`` so only one node owns the Arduino serial link.
"""

import glob
import math
import os
import re
import threading
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

try:
    import serial
except ImportError:
    serial = None


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


def _normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _quat_from_yaw(yaw):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _stamp_from_ns(now_ns):
    stamp = TimeMsg()
    stamp.sec = int(now_ns // 1_000_000_000)
    stamp.nanosec = int(now_ns % 1_000_000_000)
    return stamp


def _parse_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'on', 'yes', 'enabled')


def _port_score(port_path: str) -> int:
    path = port_path.lower()
    score = 0
    if '/dev/rfcomm' in path:
        score += 120
    if '/dev/serial/by-id/' in path:
        score += 100
    if '/dev/serial/by-path/' in path:
        score += 80
    if '/dev/ttyacm' in path:
        score += 35
    if '/dev/ttyusb' in path:
        score += 20
    for hint in ('arduino', 'mega', 'ch340', '1a86', 'wch', 'usb-serial'):
        if hint in path:
            score += 45
    for hint in ('lidar', 'rplidar', 'slamtec', 'cp210', 'silicon_labs', 'sllidar'):
        if hint in path:
            score -= 60
    return score


def _detect_mega_port() -> Optional[str]:
    candidates = []
    for pattern in (
        '/dev/rfcomm*',
        '/dev/serial/by-id/*',
        '/dev/serial/by-path/*',
        '/dev/ttyACM*',
        '/dev/ttyUSB*',
    ):
        candidates.extend(sorted(glob.glob(pattern)))

    unique = []
    seen = set()
    for candidate in candidates:
        real = os.path.realpath(candidate)
        if real not in seen:
            seen.add(real)
            unique.append(candidate)

    if not unique:
        return None
    return max(unique, key=lambda item: (_port_score(item), item))


@dataclass
class MegaTelemetry:
    left_ticks: Optional[int] = None
    right_ticks: Optional[int] = None
    accel_x_raw: Optional[int] = None
    accel_y_raw: Optional[int] = None
    accel_z_raw: Optional[int] = None
    gyro_x_raw: Optional[int] = None
    gyro_y_raw: Optional[int] = None
    gyro_z_raw: Optional[int] = None


class MegaBridgeNode(Node):
    """Compatibility bridge for Arduino Mega command and telemetry."""

    def __init__(self):
        super().__init__('mega_bridge_node')

        self.declare_parameter('port', '/dev/rfcomm0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('status_topic', '/base/status')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_raw_topic', '/odom/raw')
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('imu_raw_topic', '')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('imu_frame_id', 'base_link')
        self.declare_parameter('publish_odom_tf', True)
        self.declare_parameter('wheel_radius', 0.0425)
        self.declare_parameter('wheel_separation', 0.42)
        self.declare_parameter('encoder_ticks_per_rev', 1320.0)
        self.declare_parameter('reverse_left_ticks', False)
        self.declare_parameter('reverse_right_ticks', False)
        self.declare_parameter('auto_detect_encoder_direction', True)
        self.declare_parameter('encoder_direction_min_delta_ticks', 3)
        self.declare_parameter('write_rate_hz', 20.0)
        self.declare_parameter('read_rate_hz', 50.0)
        self.declare_parameter('cmd_vel_timeout_sec', 0.8)
        self.declare_parameter('serial_reconnect_sec', 3.0)
        self.declare_parameter('serial_exclusive', True)
        self.declare_parameter('serial_startup_delay_sec', 0.0)
        self.declare_parameter('read_enabled', True)
        self.declare_parameter('max_linear_speed_mps', 0.16)
        self.declare_parameter('max_angular_speed_radps', 0.85)
        self.declare_parameter('reject_encoder_jump', True)
        self.declare_parameter('max_tick_delta', 5000)

        port_param = str(self.get_parameter('port').value).strip()
        if port_param == 'auto':
            detected = _detect_mega_port()
            self.port = detected if detected else '/dev/rfcomm0'
            if detected:
                self.get_logger().info(f'Auto-detected Arduino port: {detected}')
            else:
                self.get_logger().warn('No Arduino serial port detected; using /dev/rfcomm0')
        else:
            self.port = port_param

        self.baudrate = int(self.get_parameter('baudrate').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.status_topic = str(self.get_parameter('status_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.odom_raw_topic = str(self.get_parameter('odom_raw_topic').value)
        imu_raw_topic = str(self.get_parameter('imu_raw_topic').value).strip()
        imu_topic = str(self.get_parameter('imu_topic').value).strip()
        self.imu_raw_topic = imu_raw_topic if imu_raw_topic else imu_topic
        self.odom_frame_id = str(self.get_parameter('odom_frame_id').value)
        self.base_frame_id = str(self.get_parameter('base_frame_id').value)
        self.imu_frame_id = str(self.get_parameter('imu_frame_id').value)
        self.publish_odom_tf = bool(self.get_parameter('publish_odom_tf').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.ticks_per_rev = float(self.get_parameter('encoder_ticks_per_rev').value)
        self.reverse_left_ticks = bool(self.get_parameter('reverse_left_ticks').value)
        self.reverse_right_ticks = bool(self.get_parameter('reverse_right_ticks').value)
        self.auto_detect_encoder_direction = bool(
            self.get_parameter('auto_detect_encoder_direction').value
        )
        self.encoder_direction_min_delta_ticks = max(
            1, int(self.get_parameter('encoder_direction_min_delta_ticks').value)
        )
        self.meters_per_tick = (2.0 * math.pi * self.wheel_radius) / max(
            self.ticks_per_rev, 1e-6
        )
        self.write_rate_hz = max(1.0, float(self.get_parameter('write_rate_hz').value))
        self.read_rate_hz = max(1.0, float(self.get_parameter('read_rate_hz').value))
        self.cmd_vel_timeout_sec = max(
            0.1, float(self.get_parameter('cmd_vel_timeout_sec').value)
        )
        self.serial_reconnect_sec = max(
            1.0, float(self.get_parameter('serial_reconnect_sec').value)
        )
        self.serial_exclusive = bool(self.get_parameter('serial_exclusive').value)
        self.serial_startup_delay_sec = max(
            0.0, float(self.get_parameter('serial_startup_delay_sec').value)
        )
        self.read_enabled = bool(self.get_parameter('read_enabled').value)
        self.max_linear = float(self.get_parameter('max_linear_speed_mps').value)
        self.max_angular = float(self.get_parameter('max_angular_speed_radps').value)
        self.reject_encoder_jump = bool(self.get_parameter('reject_encoder_jump').value)
        self.max_tick_delta = max(1, int(self.get_parameter('max_tick_delta').value))

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.x_raw = 0.0
        self.y_raw = 0.0
        self.yaw_raw = 0.0
        self.prev_left_ticks: Optional[int] = None
        self.prev_right_ticks: Optional[int] = None
        self.last_odom_time_ns: Optional[int] = None
        self.left_tick_sign = -1 if self.reverse_left_ticks else 1
        self.right_tick_sign = -1 if self.reverse_right_ticks else 1
        self.encoder_direction_locked = not self.auto_detect_encoder_direction
        self.last_accel = (0.0, 0.0, 9.81)
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.last_cmd_ns = int(self.get_clock().now().nanoseconds)
        self.ser: Optional['serial.Serial'] = None
        self.serial_lock = threading.Lock()
        self.read_buffer = ''

        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.odom_raw_pub = self.create_publisher(Odometry, self.odom_raw_topic, 10)
        self.imu_raw_pub = self.create_publisher(Imu, self.imu_raw_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self._cmd_vel_cb, 10
        )
        self.write_timer = self.create_timer(
            1.0 / self.write_rate_hz, self._write_cmd_tick
        )
        if self.read_enabled:
            self.read_timer = self.create_timer(1.0 / self.read_rate_hz, self._read_serial_tick)
        self.reconnect_timer = self.create_timer(
            self.serial_reconnect_sec, self._try_connect
        )
        self._try_connect()

        self.get_logger().info(
            f'MegaBridgeNode ready: port={self.port}, baud={self.baudrate}, '
            f'wheel_radius={self.wheel_radius}, wheel_separation={self.wheel_separation}, '
            f'ticks_per_rev={self.ticks_per_rev:.0f}'
        )

    def _try_connect(self):
        if self.ser is not None and self.ser.is_open:
            return
        if serial is None:
            self.get_logger().error('pyserial not installed. Install python3-serial.')
            return
        try:
            serial_kwargs = {
                'port': self.port,
                'baudrate': self.baudrate,
                'timeout': 0.01,
                'write_timeout': 0.1,
            }
            if self.serial_exclusive:
                serial_kwargs['exclusive'] = True
            try:
                self.ser = serial.Serial(**serial_kwargs)
            except TypeError:
                serial_kwargs.pop('exclusive', None)
                self.ser = serial.Serial(**serial_kwargs)
                if self.serial_exclusive:
                    self.get_logger().warn(
                        'pyserial does not support exclusive serial locking on this system.'
                    )
            if self.serial_startup_delay_sec > 0.0:
                time.sleep(self.serial_startup_delay_sec)
            try:
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
            except Exception:
                pass
            self.get_logger().info(f'Serial connected: {self.port}')
        except Exception as exc:
            self.ser = None
            self.get_logger().warn(f'Serial connect failed ({self.port}): {exc}')

    def _serial_write(self, data: str):
        with self.serial_lock:
            if self.ser is None or not self.ser.is_open:
                return
            try:
                self.ser.write((data + '\n').encode('ascii'))
            except Exception as exc:
                self.get_logger().warn(f'Serial write error: {exc}')
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None

    def _cmd_vel_cb(self, msg: Twist):
        self.cmd_linear = _clamp(float(msg.linear.x), -self.max_linear, self.max_linear)
        self.cmd_angular = _clamp(
            float(msg.angular.z), -self.max_angular, self.max_angular
        )
        self.last_cmd_ns = int(self.get_clock().now().nanoseconds)

    def _write_cmd_tick(self):
        now_ns = int(self.get_clock().now().nanoseconds)
        age_sec = (now_ns - self.last_cmd_ns) * 1e-9
        if age_sec > self.cmd_vel_timeout_sec:
            v = 0.0
            w = 0.0
        else:
            v = self.cmd_linear
            w = self.cmd_angular
        self._serial_write(f'CMD_VEL,{v:.4f},{w:.4f}')

    def _read_serial_tick(self):
        if self.ser is None or not self.ser.is_open:
            return
        try:
            raw = self.ser.read(self.ser.in_waiting or 1)
            if not raw:
                return
            self.read_buffer += raw.decode('ascii', errors='replace')
        except Exception as exc:
            self.get_logger().warn(f'Serial read error: {exc}')
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            return

        while '\n' in self.read_buffer:
            line, self.read_buffer = self.read_buffer.split('\n', 1)
            line = line.strip()
            if line:
                self._handle_serial_line(line)

    _RE_ENCODER = re.compile(
        r'Encoder\s*:\s*T=(-?\d+)\s*\|\s*P=(-?\d+)', re.IGNORECASE
    )
    _RE_ACCEL = re.compile(
        r'Gia toc\s*:\s*X=(-?\d+)\s*\|\s*Y=(-?\d+)\s*\|\s*Z=(-?\d+)',
        re.IGNORECASE,
    )
    _RE_GYRO = re.compile(
        r'Goc ngieng\s*:\s*X=(-?\d+)\s*\|\s*Y=(-?\d+)\s*\|\s*Z=(-?\d+)',
        re.IGNORECASE,
    )

    def _handle_serial_line(self, line: str):
        self.status_pub.publish(String(data=line))
        telemetry = self._parse_telemetry_line(line)
        if telemetry is None:
            return

        if telemetry.left_ticks is not None and telemetry.right_ticks is not None:
            self._update_odometry(telemetry.left_ticks, telemetry.right_ticks)
        if (
            telemetry.accel_x_raw is not None
            and telemetry.accel_y_raw is not None
            and telemetry.accel_z_raw is not None
        ):
            self.last_accel = (
                telemetry.accel_x_raw / 16384.0 * 9.81,
                telemetry.accel_y_raw / 16384.0 * 9.81,
                telemetry.accel_z_raw / 16384.0 * 9.81,
            )
        if (
            telemetry.gyro_x_raw is not None
            and telemetry.gyro_y_raw is not None
            and telemetry.gyro_z_raw is not None
        ):
            gx = telemetry.gyro_x_raw / 131.0 * (math.pi / 180.0)
            gy = telemetry.gyro_y_raw / 131.0 * (math.pi / 180.0)
            gz = telemetry.gyro_z_raw / 131.0 * (math.pi / 180.0)
            self._publish_imu_raw(gx, gy, gz)

    def _parse_telemetry_line(self, line: str) -> Optional[MegaTelemetry]:
        if line.startswith('STAT,'):
            return self._parse_stat_line(line)
        if line.startswith('$TELE,'):
            return self._parse_tele_line(line)
        match = self._RE_ENCODER.search(line)
        if match:
            return MegaTelemetry(left_ticks=int(match.group(1)), right_ticks=int(match.group(2)))
        match = self._RE_ACCEL.search(line)
        if match:
            return MegaTelemetry(
                accel_x_raw=int(match.group(1)),
                accel_y_raw=int(match.group(2)),
                accel_z_raw=int(match.group(3)),
            )
        match = self._RE_GYRO.search(line)
        if match:
            return MegaTelemetry(
                gyro_x_raw=int(match.group(1)),
                gyro_y_raw=int(match.group(2)),
                gyro_z_raw=int(match.group(3)),
            )
        return None

    def _parse_stat_line(self, line: str) -> Optional[MegaTelemetry]:
        parts = [part.strip() for part in line.split(',')]
        if len(parts) < 6:
            return None
        try:
            telemetry = MegaTelemetry(
                left_ticks=int(parts[4]),
                right_ticks=int(parts[5]),
            )
            if len(parts) >= 12:
                telemetry.accel_x_raw = int(parts[6])
                telemetry.accel_y_raw = int(parts[7])
                telemetry.accel_z_raw = int(parts[8])
                telemetry.gyro_x_raw = int(parts[9])
                telemetry.gyro_y_raw = int(parts[10])
                telemetry.gyro_z_raw = int(parts[11])
            return telemetry
        except ValueError:
            self.get_logger().warn(f'Bad STAT telemetry: {line}')
            return None

    def _parse_tele_line(self, line: str) -> Optional[MegaTelemetry]:
        parts = [part.strip() for part in line[6:].split(',')]
        if len(parts) != 8:
            return None
        try:
            return MegaTelemetry(
                left_ticks=int(parts[0]),
                right_ticks=int(parts[1]),
                accel_x_raw=int(parts[2]),
                accel_y_raw=int(parts[3]),
                accel_z_raw=int(parts[4]),
                gyro_x_raw=int(parts[5]),
                gyro_y_raw=int(parts[6]),
                gyro_z_raw=int(parts[7]),
            )
        except ValueError:
            self.get_logger().warn(f'Bad $TELE telemetry: {line}')
            return None

    def _update_odometry(self, left_ticks: int, right_ticks: int):
        now_ns = int(self.get_clock().now().nanoseconds)
        if self.prev_left_ticks is None or self.prev_right_ticks is None:
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            self.last_odom_time_ns = now_ns
            return

        dt = (now_ns - self.last_odom_time_ns) * 1e-9 if self.last_odom_time_ns else 0.02
        if dt <= 0.0 or dt > 2.0:
            dt = 0.02
        self.last_odom_time_ns = now_ns
        delta_left = left_ticks - self.prev_left_ticks
        delta_right = right_ticks - self.prev_right_ticks
        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks

        if not self.encoder_direction_locked:
            straight_forward_cmd = self.cmd_linear > 0.04 and abs(self.cmd_angular) < 0.12
            enough_motion = (
                abs(delta_left) >= self.encoder_direction_min_delta_ticks
                and abs(delta_right) >= self.encoder_direction_min_delta_ticks
            )
            if straight_forward_cmd and enough_motion:
                self.left_tick_sign = 1 if delta_left > 0 else -1
                self.right_tick_sign = 1 if delta_right > 0 else -1
                self.encoder_direction_locked = True
                self.get_logger().info(
                    'Encoder direction detected: '
                    f'left_sign={self.left_tick_sign}, right_sign={self.right_tick_sign}'
                )

        delta_left *= self.left_tick_sign
        delta_right *= self.right_tick_sign

        if self.reject_encoder_jump and (
            abs(delta_left) > self.max_tick_delta or abs(delta_right) > self.max_tick_delta
        ):
            self.get_logger().warn(f'Encoder jump rejected: dL={delta_left}, dR={delta_right}')
            return

        dist_left = delta_left * self.meters_per_tick
        dist_right = delta_right * self.meters_per_tick
        delta_s = 0.5 * (dist_left + dist_right)
        delta_yaw = (dist_right - dist_left) / max(self.wheel_separation, 1e-6)
        linear_vel = delta_s / max(dt, 1e-6)
        angular_vel = delta_yaw / max(dt, 1e-6)
        stamp = _stamp_from_ns(now_ns)

        self.x_raw += delta_s * math.cos(self.yaw_raw + 0.5 * delta_yaw)
        self.y_raw += delta_s * math.sin(self.yaw_raw + 0.5 * delta_yaw)
        self.yaw_raw = _normalize_angle(self.yaw_raw + delta_yaw)
        self._publish_odom(
            self.odom_raw_pub,
            self.x_raw,
            self.y_raw,
            self.yaw_raw,
            linear_vel,
            angular_vel,
            stamp,
            publish_tf=False,
        )

        self.x += delta_s * math.cos(self.yaw + 0.5 * delta_yaw)
        self.y += delta_s * math.sin(self.yaw + 0.5 * delta_yaw)
        self.yaw = _normalize_angle(self.yaw + delta_yaw)
        self._publish_odom(
            self.odom_pub,
            self.x,
            self.y,
            self.yaw,
            linear_vel,
            angular_vel,
            stamp,
            publish_tf=self.publish_odom_tf,
        )

    def _publish_odom(
        self,
        publisher,
        x,
        y,
        yaw,
        linear_vel,
        angular_vel,
        stamp,
        publish_tf,
    ):
        qx, qy, qz, qw = _quat_from_yaw(yaw)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.angular.z = angular_vel
        pose_cov = [0.0] * 36
        twist_cov = [0.0] * 36
        pose_cov[0] = 0.05
        pose_cov[7] = 0.05
        pose_cov[14] = 99999.0
        pose_cov[21] = 99999.0
        pose_cov[28] = 99999.0
        pose_cov[35] = 0.10
        twist_cov[0] = 0.05
        twist_cov[7] = 0.05
        twist_cov[14] = 99999.0
        twist_cov[21] = 99999.0
        twist_cov[28] = 99999.0
        twist_cov[35] = 0.10
        odom.pose.covariance = pose_cov
        odom.twist.covariance = twist_cov
        publisher.publish(odom)

        if publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.odom_frame_id
            tf_msg.child_frame_id = self.base_frame_id
            tf_msg.transform.translation.x = x
            tf_msg.transform.translation.y = y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf_msg)

    def _publish_imu_raw(self, gx: float, gy: float, gz: float):
        now_ns = int(self.get_clock().now().nanoseconds)
        msg = Imu()
        msg.header.stamp = _stamp_from_ns(now_ns)
        msg.header.frame_id = self.imu_frame_id
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.angular_velocity_covariance[0] = 0.02
        msg.angular_velocity_covariance[4] = 0.02
        msg.angular_velocity_covariance[8] = 0.02
        ax, ay, az = self.last_accel
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.linear_acceleration_covariance[0] = 0.5
        msg.linear_acceleration_covariance[4] = 0.5
        msg.linear_acceleration_covariance[8] = 0.5
        self.imu_raw_pub.publish(msg)

    def destroy_node(self):
        try:
            self._serial_write('CMD_VEL,0.0000,0.0000')
        except Exception:
            pass
        if self.ser is not None:
            try:
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
