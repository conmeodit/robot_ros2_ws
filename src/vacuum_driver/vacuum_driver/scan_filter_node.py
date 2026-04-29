import math
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def median(values):
    if not values:
        return None
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2 == 1:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


class ScanFilterNode(Node):
    """Preprocess real lidar scans before SLAM and autonomy consume /scan."""

    def __init__(self):
        super().__init__('scan_filter_node')

        self.declare_parameter('input_topic', '/scan/raw')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('reverse_scan', False)
        self.declare_parameter('scan_use_inf_for_max_range', True)
        self.declare_parameter('scan_max_range_margin', 0.02)
        self.declare_parameter('scan_filter_enabled', True)
        self.declare_parameter('scan_filter_window', 5)
        self.declare_parameter('scan_filter_min_valid_neighbors', 3)
        self.declare_parameter('scan_filter_outlier_threshold_m', 0.12)
        self.declare_parameter('scan_filter_temporal_alpha', 0.60)
        self.declare_parameter('scan_filter_temporal_jump_threshold_m', 0.20)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.reverse_scan = bool(self.get_parameter('reverse_scan').value)
        self.scan_use_inf_for_max_range = bool(
            self.get_parameter('scan_use_inf_for_max_range').value
        )
        self.scan_max_range_margin = max(
            0.0, float(self.get_parameter('scan_max_range_margin').value)
        )
        self.scan_filter_enabled = bool(
            self.get_parameter('scan_filter_enabled').value
        )
        self.scan_filter_window = max(
            1, int(self.get_parameter('scan_filter_window').value)
        )
        if self.scan_filter_window % 2 == 0:
            self.scan_filter_window += 1
        self.scan_filter_min_valid_neighbors = max(
            1, int(self.get_parameter('scan_filter_min_valid_neighbors').value)
        )
        self.scan_filter_outlier_threshold_m = max(
            0.01,
            float(self.get_parameter('scan_filter_outlier_threshold_m').value),
        )
        self.scan_filter_temporal_alpha = clamp(
            float(self.get_parameter('scan_filter_temporal_alpha').value), 0.0, 1.0
        )
        self.scan_filter_temporal_jump_threshold_m = max(
            0.02,
            float(self.get_parameter('scan_filter_temporal_jump_threshold_m').value),
        )

        self.prev_filtered_ranges: Optional[List[float]] = None
        self.scan_pub = self.create_publisher(LaserScan, self.output_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, self.input_topic, self.scan_cb, 20
        )

        self.get_logger().info(
            f'scan_filter ready: {self.input_topic} -> {self.output_topic}, '
            f'filter={"on" if self.scan_filter_enabled else "off"}'
        )

    def scan_cb(self, msg: LaserScan):
        ranges = list(msg.ranges)
        if self.reverse_scan:
            ranges.reverse()

        normalized = [self._normalize_scan_range(value, msg) for value in ranges]
        clean_ranges = self._filter_scan_ranges(normalized)

        filtered = LaserScan()
        filtered.header = msg.header
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max
        filtered.ranges = clean_ranges
        filtered.intensities = list(msg.intensities)
        if self.reverse_scan and filtered.intensities:
            filtered.intensities.reverse()
        self.scan_pub.publish(filtered)

    def _normalize_scan_range(self, value, msg: LaserScan):
        try:
            rng = float(value)
        except (TypeError, ValueError):
            return float('inf')
        if not math.isfinite(rng):
            return float('inf')
        if rng < msg.range_min:
            return float('inf')
        if rng > msg.range_max:
            return float('inf')
        if (
            self.scan_use_inf_for_max_range
            and rng >= (msg.range_max - self.scan_max_range_margin)
        ):
            return float('inf')
        return rng

    def _median_filter_scan(self, ranges):
        if self.scan_filter_window <= 1:
            return list(ranges)

        radius = self.scan_filter_window // 2
        filtered = list(ranges)
        total = len(ranges)

        for index, current in enumerate(ranges):
            if not math.isfinite(current):
                continue

            neighbors = []
            for offset in range(-radius, radius + 1):
                sample_index = index + offset
                if sample_index < 0 or sample_index >= total:
                    continue
                sample = ranges[sample_index]
                if math.isfinite(sample):
                    neighbors.append(sample)

            if len(neighbors) < self.scan_filter_min_valid_neighbors:
                continue

            local_median = median(neighbors)
            if local_median is None:
                continue
            if abs(current - local_median) > self.scan_filter_outlier_threshold_m:
                filtered[index] = local_median

        return filtered

    def _temporal_filter_scan(self, ranges):
        if (
            self.prev_filtered_ranges is None
            or len(self.prev_filtered_ranges) != len(ranges)
        ):
            self.prev_filtered_ranges = list(ranges)
            return list(ranges)

        alpha = self.scan_filter_temporal_alpha
        jump_threshold = self.scan_filter_temporal_jump_threshold_m
        filtered = list(ranges)

        for index, current in enumerate(ranges):
            previous = self.prev_filtered_ranges[index]
            if not math.isfinite(current):
                filtered[index] = float('inf')
                continue
            if not math.isfinite(previous):
                filtered[index] = current
                continue
            if abs(current - previous) > jump_threshold:
                filtered[index] = current
                continue
            filtered[index] = alpha * current + (1.0 - alpha) * previous

        self.prev_filtered_ranges = list(filtered)
        return filtered

    def _filter_scan_ranges(self, normalized):
        if not self.scan_filter_enabled:
            self.prev_filtered_ranges = list(normalized)
            return list(normalized)

        spatially_filtered = self._median_filter_scan(normalized)
        return self._temporal_filter_scan(spatially_filtered)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
