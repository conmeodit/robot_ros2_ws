#!/usr/bin/env python3

import os
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from slam_toolbox.srv import SaveMap


class MapAutoSaver(Node):
    def __init__(self) -> None:
        super().__init__('map_auto_saver')

        self.declare_parameter('save_path', '/home/linh-pham/robot_maps/live_map')
        self.declare_parameter('interval_sec', 10.0)

        self.save_path = str(self.get_parameter('save_path').value)
        self.interval_sec = float(self.get_parameter('interval_sec').value)

        if self.interval_sec < 1.0:
            self.get_logger().warn('interval_sec < 1.0 is too aggressive, using 1.0 sec.')
            self.interval_sec = 1.0

        save_dir = Path(self.save_path).parent
        save_dir.mkdir(parents=True, exist_ok=True)

        self.client = self.create_client(SaveMap, '/slam_toolbox/save_map')
        self.pending: Future | None = None

        self.get_logger().info(
            f'Auto-save enabled. Path={self.save_path}, interval={self.interval_sec:.1f}s'
        )

        self.timer = self.create_timer(self.interval_sec, self._on_timer)

    def _on_timer(self) -> None:
        if not self.client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn('Waiting for /slam_toolbox/save_map service...')
            return

        if self.pending is not None and not self.pending.done():
            self.get_logger().warn('Previous save request still running, skipping this cycle.')
            return

        req = SaveMap.Request()
        req.name.data = self.save_path
        self.pending = self.client.call_async(req)
        self.pending.add_done_callback(self._on_save_done)

    def _on_save_done(self, future: Future) -> None:
        try:
            _ = future.result()
            self.get_logger().info(f'Map snapshot saved to {self.save_path}.yaml/.pgm')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Failed to save map snapshot: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapAutoSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
