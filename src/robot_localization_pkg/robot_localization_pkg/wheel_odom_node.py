"""ROS 2 node: encoder ticks to ``/odom`` + TF ``odom -> base_link``.

This legacy node subscribes to ``/base/status`` and accepts both the current
``STAT,...`` firmware telemetry and the older ``$TELE,...`` format.
"""

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


def _parse_telemetry(line: str):
    """Return ``(left_ticks, right_ticks)`` from Arduino telemetry."""
    line = line.strip()
    try:
        if line.startswith('STAT,'):
            parts = [part.strip() for part in line.split(',')]
            if len(parts) < 6:
                return None
            return int(parts[4]), int(parts[5])
        if line.startswith('$TELE,'):
            parts = [part.strip() for part in line[6:].split(',')]
            if len(parts) != 8:
                return None
            return int(parts[0]), int(parts[1])
    except ValueError:
        return None
    return None


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _quat_from_yaw(yaw: float):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


class WheelOdomNode(Node):
    def __init__(self):
        super().__init__('wheel_odom_node')

        self.declare_parameter('status_topic', '/base/status')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_raw_topic', '/odom/raw')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('wheel_radius_m', 0.0425)
        self.declare_parameter('wheel_base_m', 0.42)
        self.declare_parameter('ticks_per_rev', 1320)
        self.declare_parameter('max_tick_delta', 500)
        self.declare_parameter('publish_tf', True)

        self.status_topic = str(self.get_parameter('status_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.odom_raw_topic = str(self.get_parameter('odom_raw_topic').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius_m').value)
        self.wheel_base = float(self.get_parameter('wheel_base_m').value)
        self.ticks_per_rev = int(self.get_parameter('ticks_per_rev').value)
        self.max_tick_delta = int(self.get_parameter('max_tick_delta').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.meters_per_tick = (2.0 * math.pi * self.wheel_radius) / max(
            self.ticks_per_rev, 1
        )

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.prev_left_ticks: Optional[int] = None
        self.prev_right_ticks: Optional[int] = None
        self.prev_time_ns: Optional[int] = None

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.odom_raw_pub = self.create_publisher(Odometry, self.odom_raw_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.status_sub = self.create_subscription(
            String, self.status_topic, self._status_cb, 20
        )

        self.get_logger().info(
            f'WheelOdomNode ready: wheel_radius={self.wheel_radius}m, '
            f'wheel_base={self.wheel_base}m, ticks_per_rev={self.ticks_per_rev}, '
            f'm/tick={self.meters_per_tick:.6f}, odom_raw_topic={self.odom_raw_topic}'
        )

    def _status_cb(self, msg: String):
        result = _parse_telemetry(msg.data)
        if result is None:
            return

        left_ticks, right_ticks = result
        now_ns = self.get_clock().now().nanoseconds

        if self.prev_left_ticks is None:
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            self.prev_time_ns = now_ns
            return

        d_left = left_ticks - self.prev_left_ticks
        d_right = right_ticks - self.prev_right_ticks

        if abs(d_left) > self.max_tick_delta or abs(d_right) > self.max_tick_delta:
            self.get_logger().warn(
                f'Encoder jump rejected: dL={d_left}, dR={d_right}'
            )
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            self.prev_time_ns = now_ns
            return

        dt = (now_ns - self.prev_time_ns) * 1e-9 if self.prev_time_ns else 0.05
        if dt <= 0.0 or dt > 1.0:
            dt = 0.05

        dist_left = d_left * self.meters_per_tick
        dist_right = d_right * self.meters_per_tick
        delta_s = 0.5 * (dist_left + dist_right)
        delta_yaw = (dist_right - dist_left) / max(self.wheel_base, 1e-6)

        self.x += delta_s * math.cos(self.yaw + 0.5 * delta_yaw)
        self.y += delta_s * math.sin(self.yaw + 0.5 * delta_yaw)
        self.yaw = _normalize_angle(self.yaw + delta_yaw)

        linear_vel = delta_s / max(dt, 1e-6)
        angular_vel = delta_yaw / max(dt, 1e-6)

        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks
        self.prev_time_ns = now_ns

        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = _quat_from_yaw(self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
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

        self.odom_pub.publish(odom)
        self.odom_raw_pub.publish(odom)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
