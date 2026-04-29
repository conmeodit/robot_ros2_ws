"""ROS 2 serial bridge for the Arduino Mega base controller."""

import math
import re
import threading
from typing import Optional

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
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


class MegaBridgeNode(Node):
    """Compatibility bridge for Arduino Mega command and telemetry."""

    def __init__(self):
        super().__init__('mega_bridge_node')

        self.declare_parameter('port', '/dev/rfcomm0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('imu_raw_topic', '/imu/data_raw')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('imu_frame_id', 'base_link')
        self.declare_parameter('publish_odom_tf', True)
        self.declare_parameter('wheel_radius', 0.0425)
        self.declare_parameter('wheel_separation', 0.42)
        self.declare_parameter('encoder_ticks_per_rev', 1320.0)
        self.declare_parameter('write_rate_hz', 20.0)
        self.declare_parameter('cmd_vel_timeout_sec', 0.8)
        self.declare_parameter('serial_reconnect_sec', 3.0)
        self.declare_parameter('serial_exclusive', True)
        self.declare_parameter('max_linear_speed_mps', 0.16)
        self.declare_parameter('max_angular_speed_radps', 0.85)

        self.port = str(self.get_parameter('port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.imu_raw_topic = str(self.get_parameter('imu_raw_topic').value)
        self.odom_frame_id = str(self.get_parameter('odom_frame_id').value)
        self.base_frame_id = str(self.get_parameter('base_frame_id').value)
        self.imu_frame_id = str(self.get_parameter('imu_frame_id').value)
        self.publish_odom_tf = bool(self.get_parameter('publish_odom_tf').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.ticks_per_rev = float(self.get_parameter('encoder_ticks_per_rev').value)
        self.meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
        self.write_rate_hz = max(1.0, float(self.get_parameter('write_rate_hz').value))
        self.cmd_vel_timeout_sec = max(
            0.1, float(self.get_parameter('cmd_vel_timeout_sec').value)
        )
        self.serial_reconnect_sec = max(
            1.0, float(self.get_parameter('serial_reconnect_sec').value)
        )
        self.serial_exclusive = bool(self.get_parameter('serial_exclusive').value)
        self.max_linear = float(self.get_parameter('max_linear_speed_mps').value)
        self.max_angular = float(self.get_parameter('max_angular_speed_radps').value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.prev_left_ticks: Optional[int] = None
        self.prev_right_ticks: Optional[int] = None
        self.last_odom_time_ns: Optional[int] = None
        self.last_accel = (0.0, 0.0, 9.81)
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.last_cmd_ns = int(self.get_clock().now().nanoseconds)
        self.ser: Optional['serial.Serial'] = None
        self.serial_lock = threading.Lock()
        self.read_buffer = ''

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.imu_raw_pub = self.create_publisher(Imu, self.imu_raw_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self._cmd_vel_cb, 10
        )
        self.write_timer = self.create_timer(
            1.0 / self.write_rate_hz, self._write_cmd_tick
        )
        self.read_timer = self.create_timer(0.02, self._read_serial_tick)
        self.reconnect_timer = self.create_timer(
            self.serial_reconnect_sec, self._try_connect
        )
        self._try_connect()

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
                self._parse_telemetry_line(line)

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

    def _parse_telemetry_line(self, line: str):
        if line.startswith('STAT,'):
            parts = [part.strip() for part in line.split(',')]
            if len(parts) < 6:
                return
            try:
                left_ticks = int(parts[4])
                right_ticks = int(parts[5])
                self._update_odometry(left_ticks, right_ticks)
                if len(parts) >= 12:
                    self.last_accel = (
                        int(parts[6]) / 16384.0 * 9.81,
                        int(parts[7]) / 16384.0 * 9.81,
                        int(parts[8]) / 16384.0 * 9.81,
                    )
                    gx = int(parts[9]) / 131.0 * (math.pi / 180.0)
                    gy = int(parts[10]) / 131.0 * (math.pi / 180.0)
                    gz = int(parts[11]) / 131.0 * (math.pi / 180.0)
                    self._publish_imu_raw(gx, gy, gz)
            except ValueError:
                self.get_logger().warn(f'Bad STAT telemetry: {line}')
            return

        match = self._RE_ENCODER.search(line)
        if match:
            self._update_odometry(int(match.group(1)), int(match.group(2)))
            return
        match = self._RE_ACCEL.search(line)
        if match:
            self.last_accel = (
                int(match.group(1)) / 16384.0 * 9.81,
                int(match.group(2)) / 16384.0 * 9.81,
                int(match.group(3)) / 16384.0 * 9.81,
            )
            return
        match = self._RE_GYRO.search(line)
        if match:
            gx = int(match.group(1)) / 131.0 * (math.pi / 180.0)
            gy = int(match.group(2)) / 131.0 * (math.pi / 180.0)
            gz = int(match.group(3)) / 131.0 * (math.pi / 180.0)
            self._publish_imu_raw(gx, gy, gz)

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
        if abs(delta_left) > 5000 or abs(delta_right) > 5000:
            self.get_logger().warn(f'Encoder jump rejected: dL={delta_left}, dR={delta_right}')
            return

        dist_left = delta_left * self.meters_per_tick
        dist_right = delta_right * self.meters_per_tick
        delta_s = 0.5 * (dist_left + dist_right)
        delta_yaw = (dist_right - dist_left) / max(self.wheel_separation, 1e-6)
        self.x += delta_s * math.cos(self.yaw + 0.5 * delta_yaw)
        self.y += delta_s * math.sin(self.yaw + 0.5 * delta_yaw)
        self.yaw = _normalize_angle(self.yaw + delta_yaw)
        linear_vel = delta_s / dt
        angular_vel = delta_yaw / dt
        stamp = _stamp_from_ns(now_ns)
        qx, qy, qz, qw = _quat_from_yaw(self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
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
        self.odom_pub.publish(odom)

        if self.publish_odom_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.odom_frame_id
            tf_msg.child_frame_id = self.base_frame_id
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
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
