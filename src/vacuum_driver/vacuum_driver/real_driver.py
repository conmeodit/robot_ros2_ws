import glob
import math
import threading
from dataclasses import dataclass
from typing import Optional

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Int16MultiArray, Int64MultiArray, String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

try:
    import serial
except ImportError:
    serial = None


DEFAULT_WHEEL_RADIUS_M = 0.0425
DEFAULT_WHEEL_SEPARATION_M = 0.42
DEFAULT_LIDAR_OFFSET_X_M = -0.1
DEFAULT_LIDAR_OFFSET_Y_M = 0.0
DEFAULT_LIDAR_OFFSET_Z_M = 0.081


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quat_from_yaw(yaw):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def time_msg_from_ns(now_ns):
    stamp = TimeMsg()
    stamp.sec = int(now_ns // 1_000_000_000)
    stamp.nanosec = int(now_ns % 1_000_000_000)
    return stamp


def parse_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'on', 'yes', 'enabled')


@dataclass
class MegaTelemetry:
    mode: str = 'UNKNOWN'
    estop: bool = False
    motor_enabled: bool = False
    left_ticks: Optional[int] = None
    right_ticks: Optional[int] = None
    accel_x_raw: Optional[int] = None
    accel_y_raw: Optional[int] = None
    accel_z_raw: Optional[int] = None
    gyro_x_raw: Optional[int] = None
    gyro_y_raw: Optional[int] = None
    gyro_z_raw: Optional[int] = None
    left_pwm: Optional[int] = None
    right_pwm: Optional[int] = None


class RealHardwareDriver(Node):
    """Bridge Arduino Mega telemetry/control into the vacuum_driver topic contract."""

    def __init__(self):
        super().__init__('real_hardware_driver')

        self.declare_parameter('port', 'auto')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_raw_topic', '/odom/raw')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('imu_raw_topic', '/imu/data_raw')
        self.declare_parameter('telemetry_topic', '/hardware/mega/telemetry')
        self.declare_parameter('encoder_ticks_topic', '/hardware/encoder_ticks')
        self.declare_parameter('motor_pwm_topic', '/hardware/motor_pwm')
        self.declare_parameter('estop_topic', '/hardware/mega/estop')
        self.declare_parameter('motor_enable_topic', '/hardware/mega/motor_enable')
        self.declare_parameter('servo_topic', '/hardware/mega/servo')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('imu_frame_id', 'base_link')
        self.declare_parameter('lidar_frame_id', 'laser')
        self.declare_parameter('publish_odom_tf', True)
        self.declare_parameter('publish_lidar_tf', False)
        self.declare_parameter('lidar_offset_x', DEFAULT_LIDAR_OFFSET_X_M)
        self.declare_parameter('lidar_offset_y', DEFAULT_LIDAR_OFFSET_Y_M)
        self.declare_parameter('lidar_offset_z', DEFAULT_LIDAR_OFFSET_Z_M)
        self.declare_parameter('wheel_radius', DEFAULT_WHEEL_RADIUS_M)
        self.declare_parameter('wheel_separation', DEFAULT_WHEEL_SEPARATION_M)
        self.declare_parameter('encoder_ticks_per_rev', 1320.0)
        self.declare_parameter('reverse_left_ticks', False)
        self.declare_parameter('reverse_right_ticks', False)
        self.declare_parameter('auto_detect_encoder_direction', True)
        self.declare_parameter('encoder_direction_min_delta_ticks', 3)
        self.declare_parameter('write_rate_hz', 20.0)
        self.declare_parameter('read_rate_hz', 50.0)
        self.declare_parameter('odom_publish_rate_hz', 30.0)
        self.declare_parameter('cmd_vel_timeout_sec', 0.8)
        self.declare_parameter('serial_reconnect_sec', 3.0)
        self.declare_parameter('serial_exclusive', True)
        self.declare_parameter('max_linear_speed_mps', 0.16)
        self.declare_parameter('max_angular_speed_radps', 0.85)
        self.declare_parameter('reject_encoder_jump', True)
        self.declare_parameter('max_tick_delta', 5000)

        requested_port = str(self.get_parameter('port').value).strip()
        self.port = self._resolve_port(requested_port)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.odom_raw_topic = str(self.get_parameter('odom_raw_topic').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.imu_raw_topic = str(self.get_parameter('imu_raw_topic').value)
        self.telemetry_topic = str(self.get_parameter('telemetry_topic').value)
        self.encoder_ticks_topic = str(self.get_parameter('encoder_ticks_topic').value)
        self.motor_pwm_topic = str(self.get_parameter('motor_pwm_topic').value)
        self.estop_topic = str(self.get_parameter('estop_topic').value)
        self.motor_enable_topic = str(self.get_parameter('motor_enable_topic').value)
        self.servo_topic = str(self.get_parameter('servo_topic').value)
        self.odom_frame_id = str(self.get_parameter('odom_frame_id').value)
        self.base_frame_id = str(self.get_parameter('base_frame_id').value)
        self.imu_frame_id = str(self.get_parameter('imu_frame_id').value)
        self.lidar_frame_id = str(self.get_parameter('lidar_frame_id').value)
        self.publish_odom_tf = bool(self.get_parameter('publish_odom_tf').value)
        self.publish_lidar_tf = bool(self.get_parameter('publish_lidar_tf').value)
        self.lidar_offset_x = float(self.get_parameter('lidar_offset_x').value)
        self.lidar_offset_y = float(self.get_parameter('lidar_offset_y').value)
        self.lidar_offset_z = float(self.get_parameter('lidar_offset_z').value)
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
        self.odom_publish_rate_hz = max(
            1.0, float(self.get_parameter('odom_publish_rate_hz').value)
        )
        self.cmd_vel_timeout_sec = max(
            0.1, float(self.get_parameter('cmd_vel_timeout_sec').value)
        )
        self.serial_reconnect_sec = max(
            1.0, float(self.get_parameter('serial_reconnect_sec').value)
        )
        self.serial_exclusive = bool(self.get_parameter('serial_exclusive').value)
        self.max_linear = max(
            0.01, float(self.get_parameter('max_linear_speed_mps').value)
        )
        self.max_angular = max(
            0.05, float(self.get_parameter('max_angular_speed_radps').value)
        )
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
        self.last_linear_vel = 0.0
        self.last_angular_vel = 0.0
        self.left_tick_sign = -1 if self.reverse_left_ticks else 1
        self.right_tick_sign = -1 if self.reverse_right_ticks else 1
        self.encoder_direction_locked = not self.auto_detect_encoder_direction
        self.last_accel_mps2 = (0.0, 0.0, 9.81)
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.last_cmd_ns = int(self.get_clock().now().nanoseconds)
        self.last_cmd_log_ns = 0
        self.last_telemetry_log_ns = 0
        self.last_serial_rx_ns = 0
        self.serial_connected_ns = 0
        self.no_telemetry_warned = False
        self.unsupported_telemetry_warned = False
        self.ser: Optional['serial.Serial'] = None
        self.serial_lock = threading.Lock()
        self.read_buffer = ''

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.odom_raw_pub = self.create_publisher(Odometry, self.odom_raw_topic, 10)
        self.imu_pub = self.create_publisher(Imu, self.imu_topic, 10)
        self.imu_raw_pub = self.create_publisher(Imu, self.imu_raw_topic, 10)
        self.telemetry_pub = self.create_publisher(String, self.telemetry_topic, 10)
        self.encoder_ticks_pub = self.create_publisher(
            Int64MultiArray, self.encoder_ticks_topic, 10
        )
        self.motor_pwm_pub = self.create_publisher(Int16MultiArray, self.motor_pwm_topic, 10)
        self.cmd_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self._cmd_vel_cb, 10
        )
        self.estop_sub = self.create_subscription(
            Bool, self.estop_topic, self._estop_cb, 10
        )
        self.motor_enable_sub = self.create_subscription(
            Bool, self.motor_enable_topic, self._motor_enable_cb, 10
        )
        self.servo_sub = self.create_subscription(
            Int16MultiArray, self.servo_topic, self._servo_cb, 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        if self.publish_lidar_tf:
            self._publish_lidar_static_tf()

        self.write_timer = self.create_timer(1.0 / self.write_rate_hz, self._write_cmd_tick)
        self.read_timer = self.create_timer(1.0 / self.read_rate_hz, self._read_serial_tick)
        self.odom_timer = self.create_timer(
            1.0 / self.odom_publish_rate_hz, self._publish_current_odometry_tick
        )
        self.reconnect_timer = self.create_timer(
            self.serial_reconnect_sec, self._try_connect
        )
        self._try_connect()
        self.get_logger().info(
            'real_driver ready: '
            f'port={self.port}, baud={self.baudrate}, '
            f'wheel_r={self.wheel_radius:.4f}, wheel_sep={self.wheel_separation:.3f}, '
            f'ticks/rev={self.ticks_per_rev:.0f}'
        )

    def _resolve_port(self, requested_port: str) -> str:
        if requested_port and requested_port != 'auto':
            return requested_port

        candidates = []
        for pattern in (
            '/dev/rfcomm*',
            '/dev/serial/by-id/*',
            '/dev/serial/by-path/*',
            '/dev/ttyACM*',
            '/dev/ttyUSB*',
        ):
            candidates.extend(sorted(glob.glob(pattern)))

        unique_candidates = []
        seen = set()
        for candidate in candidates:
            if candidate not in seen:
                unique_candidates.append(candidate)
                seen.add(candidate)
        if not unique_candidates:
            return '/dev/rfcomm0'

        def score(path):
            lowered = path.lower()
            value = 0
            if '/dev/rfcomm' in lowered:
                value += 120
            if '/dev/serial/by-id/' in lowered:
                value += 80
            if '/dev/ttyacm' in lowered:
                value += 35
            if '/dev/ttyusb' in lowered:
                value += 20
            for hint in ('arduino', 'mega', 'ch340', 'wch', '1a86', 'usb-serial', 'usb_serial'):
                if hint in lowered:
                    value += 45
            for hint in ('lidar', 'rplidar', 'sllidar', 'slamtec', 'cp210'):
                if hint in lowered:
                    value -= 60
            return value

        selected = max(unique_candidates, key=lambda item: (score(item), item))
        self.get_logger().info(
            f'Auto-detected Arduino port: {selected} '
            f'(candidates: {", ".join(unique_candidates)})'
        )
        return selected

    def _try_connect(self):
        if self.ser is not None and self.ser.is_open:
            return
        if serial is None:
            self.get_logger().error('pyserial is missing. Install python3-serial.')
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
            self.get_logger().info(f'Arduino serial connected: {self.port}')
            self.serial_connected_ns = int(self.get_clock().now().nanoseconds)
            self.no_telemetry_warned = False
            self._serial_write('MOTOR,1')
            self._serial_write('ESTOP,0')
        except Exception as exc:
            self.ser = None
            self.get_logger().warn(f'Arduino serial connect failed ({self.port}): {exc}')

    def _serial_write(self, data: str):
        with self.serial_lock:
            if self.ser is None or not self.ser.is_open:
                return
            try:
                self.ser.write((data + '\n').encode('ascii'))
            except Exception as exc:
                self.get_logger().warn(f'Arduino serial write error: {exc}')
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None

    def _cmd_vel_cb(self, msg: Twist):
        self.cmd_linear = clamp(float(msg.linear.x), -self.max_linear, self.max_linear)
        self.cmd_angular = clamp(
            float(msg.angular.z), -self.max_angular, self.max_angular
        )
        self.last_cmd_ns = int(self.get_clock().now().nanoseconds)

    def _estop_cb(self, msg: Bool):
        self._serial_write(f'ESTOP,{1 if msg.data else 0}')

    def _motor_enable_cb(self, msg: Bool):
        self._serial_write(f'MOTOR,{1 if msg.data else 0}')

    def _servo_cb(self, msg: Int16MultiArray):
        if len(msg.data) < 2:
            self.get_logger().warn('Servo command requires [id, degrees].')
            return
        servo_id = int(msg.data[0])
        degrees = int(clamp(int(msg.data[1]), 0, 180))
        self._serial_write(f'SERVO,{servo_id},{degrees}')

    def _write_cmd_tick(self):
        now_ns = int(self.get_clock().now().nanoseconds)
        age_sec = (now_ns - self.last_cmd_ns) * 1e-9
        if age_sec > self.cmd_vel_timeout_sec:
            linear = 0.0
            angular = 0.0
        else:
            linear = self.cmd_linear
            angular = self.cmd_angular
        self._serial_write(f'CMD_VEL,{linear:.4f},{angular:.4f}')
        if (
            self.ser is not None
            and self.ser.is_open
            and self.last_serial_rx_ns == 0
            and not self.no_telemetry_warned
            and self.serial_connected_ns > 0
            and (now_ns - self.serial_connected_ns) > 3_000_000_000
        ):
            self.no_telemetry_warned = True
            self.get_logger().warn(
                'Arduino serial is open but no telemetry line has been received. '
                'Stop launch and test the USB port directly; ROS requires '
                'firmware/arduino_mega_base to print STAT,... on USB Serial at 115200.'
            )
        if abs(linear) > 0.01 or abs(angular) > 0.05:
            if (now_ns - self.last_cmd_log_ns) > 1_000_000_000:
                self.last_cmd_log_ns = now_ns
                self.get_logger().info(
                    f'cmd_vel -> Arduino: linear={linear:.3f}, angular={angular:.3f}'
                )

    def _read_serial_tick(self):
        if self.ser is None or not self.ser.is_open:
            return
        try:
            raw = self.ser.read(self.ser.in_waiting or 1)
            if not raw:
                return
            self.read_buffer += raw.decode('ascii', errors='replace')
        except Exception as exc:
            self.get_logger().warn(f'Arduino serial read error: {exc}')
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
                self._handle_line(line)

    def _handle_line(self, line: str):
        self.last_serial_rx_ns = int(self.get_clock().now().nanoseconds)
        self.telemetry_pub.publish(String(data=line))
        telemetry = self._parse_line(line)
        if telemetry is None:
            return

        now_ns = int(self.get_clock().now().nanoseconds)
        stamp = time_msg_from_ns(now_ns)
        if telemetry.left_ticks is not None and telemetry.right_ticks is not None:
            self._publish_ticks(telemetry.left_ticks, telemetry.right_ticks)
            self._update_odometry(telemetry.left_ticks, telemetry.right_ticks, stamp, now_ns)
        if telemetry.left_pwm is not None and telemetry.right_pwm is not None:
            self._publish_motor_pwm(telemetry.left_pwm, telemetry.right_pwm)
            if (
                abs(telemetry.left_pwm) > 0
                or abs(telemetry.right_pwm) > 0
                or not telemetry.motor_enabled
                or telemetry.estop
            ):
                if (now_ns - self.last_telemetry_log_ns) > 1_000_000_000:
                    self.last_telemetry_log_ns = now_ns
                    self.get_logger().info(
                        'Arduino telemetry: '
                        f'mode={telemetry.mode}, estop={int(telemetry.estop)}, '
                        f'motor={int(telemetry.motor_enabled)}, '
                        f'ticks=({telemetry.left_ticks},{telemetry.right_ticks}), '
                        f'pwm=({telemetry.left_pwm},{telemetry.right_pwm})'
                    )
        if (
            telemetry.accel_x_raw is not None
            and telemetry.accel_y_raw is not None
            and telemetry.accel_z_raw is not None
        ):
            self.last_accel_mps2 = (
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
            self._publish_imu(gx, gy, gz, stamp)

    def _parse_line(self, line: str) -> Optional[MegaTelemetry]:
        if line.startswith('STAT,'):
            return self._parse_stat_line(line)
        if line.startswith('Encoder'):
            return self._parse_encoder_line(line)
        if line.startswith('Gia toc'):
            return self._parse_accel_line(line)
        if line.startswith('Goc ngieng'):
            return self._parse_gyro_line(line)
        return None

    def _parse_stat_line(self, line: str) -> Optional[MegaTelemetry]:
        parts = [part.strip() for part in line.split(',')]
        if len(parts) < 6:
            if not self.unsupported_telemetry_warned:
                self.unsupported_telemetry_warned = True
                self.get_logger().warn(
                    'Unsupported Arduino STAT telemetry format. '
                    'If this is from firmware/mega_motor_test, upload '
                    'firmware/arduino_mega_base with ./upload_mega.sh; '
                    'the ROS driver sends CMD_VEL and requires encoder telemetry.'
                )
            return None
        telemetry = MegaTelemetry()
        try:
            telemetry.mode = parts[1]
            telemetry.estop = parse_bool(parts[2])
            telemetry.motor_enabled = parse_bool(parts[3])
            telemetry.left_ticks = int(parts[4])
            telemetry.right_ticks = int(parts[5])
            if len(parts) >= 12:
                telemetry.accel_x_raw = int(parts[6])
                telemetry.accel_y_raw = int(parts[7])
                telemetry.accel_z_raw = int(parts[8])
                telemetry.gyro_x_raw = int(parts[9])
                telemetry.gyro_y_raw = int(parts[10])
                telemetry.gyro_z_raw = int(parts[11])
            if len(parts) >= 14:
                telemetry.left_pwm = int(parts[12])
                telemetry.right_pwm = int(parts[13])
        except ValueError:
            self.get_logger().warn(f'Bad STAT telemetry: {line}')
            return None
        return telemetry

    def _parse_encoder_line(self, line: str) -> Optional[MegaTelemetry]:
        try:
            _, values = line.split(':', 1)
            left_part, right_part = values.split('|', 1)
            left_ticks = int(left_part.split('=')[1].strip())
            right_ticks = int(right_part.split('=')[1].strip())
            return MegaTelemetry(left_ticks=left_ticks, right_ticks=right_ticks)
        except Exception:
            return None

    def _parse_accel_line(self, line: str) -> Optional[MegaTelemetry]:
        values = self._parse_xyz_line(line)
        if values is None:
            return None
        return MegaTelemetry(accel_x_raw=values[0], accel_y_raw=values[1], accel_z_raw=values[2])

    def _parse_gyro_line(self, line: str) -> Optional[MegaTelemetry]:
        values = self._parse_xyz_line(line)
        if values is None:
            return None
        return MegaTelemetry(gyro_x_raw=values[0], gyro_y_raw=values[1], gyro_z_raw=values[2])

    def _parse_xyz_line(self, line: str):
        try:
            _, values = line.split(':', 1)
            parts = values.split('|')
            x = int(parts[0].split('=')[1].strip())
            y = int(parts[1].split('=')[1].strip())
            z = int(parts[2].split('=')[1].strip())
            return x, y, z
        except Exception:
            return None

    def _publish_ticks(self, left_ticks: int, right_ticks: int):
        msg = Int64MultiArray()
        msg.data = [int(left_ticks), int(right_ticks)]
        self.encoder_ticks_pub.publish(msg)

    def _publish_motor_pwm(self, left_pwm: int, right_pwm: int):
        msg = Int16MultiArray()
        msg.data = [int(left_pwm), int(right_pwm)]
        self.motor_pwm_pub.publish(msg)

    def _publish_current_odometry_tick(self):
        now_ns = int(self.get_clock().now().nanoseconds)
        stamp = time_msg_from_ns(now_ns)
        odom_age_sec = (
            (now_ns - self.last_odom_time_ns) * 1e-9
            if self.last_odom_time_ns is not None
            else float('inf')
        )
        if odom_age_sec > 0.5:
            linear_vel = 0.0
            angular_vel = 0.0
        else:
            linear_vel = self.last_linear_vel
            angular_vel = self.last_angular_vel
        self._publish_odometry_state(stamp, linear_vel, angular_vel)

    def _update_odometry(self, left_ticks: int, right_ticks: int, stamp: TimeMsg, now_ns: int):
        if self.prev_left_ticks is None or self.prev_right_ticks is None:
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            self.last_odom_time_ns = now_ns
            self.last_linear_vel = 0.0
            self.last_angular_vel = 0.0
            self._publish_odometry_state(stamp, 0.0, 0.0)
            return

        dt = (now_ns - self.last_odom_time_ns) * 1e-9 if self.last_odom_time_ns else 0.1
        if dt <= 0.0 or dt > 2.0:
            dt = 0.1
        self.last_odom_time_ns = now_ns
        delta_left_ticks = left_ticks - self.prev_left_ticks
        delta_right_ticks = right_ticks - self.prev_right_ticks
        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks

        if not self.encoder_direction_locked:
            straight_forward_cmd = (
                self.cmd_linear > 0.04 and abs(self.cmd_angular) < 0.12
            )
            enough_motion = (
                abs(delta_left_ticks) >= self.encoder_direction_min_delta_ticks
                and abs(delta_right_ticks) >= self.encoder_direction_min_delta_ticks
            )
            if straight_forward_cmd and enough_motion:
                self.left_tick_sign = 1 if delta_left_ticks > 0 else -1
                self.right_tick_sign = 1 if delta_right_ticks > 0 else -1
                self.encoder_direction_locked = True
                self.get_logger().info(
                    'Encoder direction detected: '
                    f'left_sign={self.left_tick_sign}, right_sign={self.right_tick_sign}'
                )

        delta_left_ticks *= self.left_tick_sign
        delta_right_ticks *= self.right_tick_sign

        if self.reject_encoder_jump and (
            abs(delta_left_ticks) > self.max_tick_delta
            or abs(delta_right_ticks) > self.max_tick_delta
        ):
            self.get_logger().warn(
                f'Encoder jump rejected: dL={delta_left_ticks}, dR={delta_right_ticks}'
            )
            return

        dist_left = delta_left_ticks * self.meters_per_tick
        dist_right = delta_right_ticks * self.meters_per_tick
        delta_s = 0.5 * (dist_left + dist_right)
        delta_yaw = (dist_right - dist_left) / max(self.wheel_separation, 1e-6)
        linear_vel = delta_s / max(dt, 1e-6)
        angular_vel = delta_yaw / max(dt, 1e-6)
        self.last_linear_vel = linear_vel
        self.last_angular_vel = angular_vel

        self.x_raw += delta_s * math.cos(self.yaw_raw + 0.5 * delta_yaw)
        self.y_raw += delta_s * math.sin(self.yaw_raw + 0.5 * delta_yaw)
        self.yaw_raw = normalize_angle(self.yaw_raw + delta_yaw)
        self.x += delta_s * math.cos(self.yaw + 0.5 * delta_yaw)
        self.y += delta_s * math.sin(self.yaw + 0.5 * delta_yaw)
        self.yaw = normalize_angle(self.yaw + delta_yaw)
        self._publish_odometry_state(stamp, linear_vel, angular_vel)

    def _publish_odometry_state(self, stamp: TimeMsg, linear_vel: float, angular_vel: float):
        self._publish_odom_msg(
            self.odom_raw_pub,
            self.x_raw,
            self.y_raw,
            self.yaw_raw,
            linear_vel,
            angular_vel,
            stamp,
            publish_tf=False,
        )
        self._publish_odom_msg(
            self.odom_pub,
            self.x,
            self.y,
            self.yaw,
            linear_vel,
            angular_vel,
            stamp,
            publish_tf=self.publish_odom_tf,
        )

    def _publish_odom_msg(
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
        qx, qy, qz, qw = quat_from_yaw(yaw)
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

    def _publish_imu(self, gyro_x, gyro_y, gyro_z, stamp):
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self.imu_frame_id
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity.x = gyro_x
        msg.angular_velocity.y = gyro_y
        msg.angular_velocity.z = gyro_z
        msg.angular_velocity_covariance[0] = 0.02
        msg.angular_velocity_covariance[4] = 0.02
        msg.angular_velocity_covariance[8] = 0.02
        ax, ay, az = self.last_accel_mps2
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.linear_acceleration_covariance[0] = 0.5
        msg.linear_acceleration_covariance[4] = 0.5
        msg.linear_acceleration_covariance[8] = 0.5
        self.imu_raw_pub.publish(msg)
        self.imu_pub.publish(msg)

    def _publish_lidar_static_tf(self):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.base_frame_id
        tf_msg.child_frame_id = self.lidar_frame_id
        tf_msg.transform.translation.x = self.lidar_offset_x
        tf_msg.transform.translation.y = self.lidar_offset_y
        tf_msg.transform.translation.z = self.lidar_offset_z
        tf_msg.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(tf_msg)

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
    node = RealHardwareDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
