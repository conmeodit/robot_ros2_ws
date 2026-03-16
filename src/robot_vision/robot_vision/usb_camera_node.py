#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class UsbCameraNode(Node):
	def __init__(self) -> None:
		super().__init__('usb_camera_node')

		self.declare_parameter('camera_index', 0)
		self.declare_parameter('frame_id', 'camera_link')
		self.declare_parameter('width', 640)
		self.declare_parameter('height', 480)
		self.declare_parameter('fps', 30.0)
		self.declare_parameter('image_topic', '/camera/image_raw')

		self.camera_index = int(self.get_parameter('camera_index').value)
		self.frame_id = str(self.get_parameter('frame_id').value)
		self.width = int(self.get_parameter('width').value)
		self.height = int(self.get_parameter('height').value)
		self.fps = float(self.get_parameter('fps').value)
		self.image_topic = str(self.get_parameter('image_topic').value)

		self.bridge = CvBridge()
		self.image_pub = self.create_publisher(Image, self.image_topic, 10)

		self.cap = None
		self._reopen_camera()

		timer_period = 1.0 / max(self.fps, 1.0)
		self.timer = self.create_timer(timer_period, self._on_timer)

		self.get_logger().info(
			f'USB camera started: /dev/video{self.camera_index}, '
			f'{self.width}x{self.height}@{self.fps:.1f} -> {self.image_topic}'
		)

	def _reopen_camera(self) -> None:
		if self.cap is not None:
			self.cap.release()

		self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
		if not self.cap.isOpened():
			self.get_logger().warn('OpenCV V4L2 failed, fallback to default backend.')
			self.cap = cv2.VideoCapture(self.camera_index)

		if not self.cap.isOpened():
			self.get_logger().error(f'Cannot open camera index {self.camera_index}')
			return

		self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
		self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
		self.cap.set(cv2.CAP_PROP_FPS, self.fps)

	def _on_timer(self) -> None:
		if self.cap is None or not self.cap.isOpened():
			self._reopen_camera()
			return

		ok, frame = self.cap.read()
		if not ok:
			self.get_logger().warn('Failed to read frame. Re-opening camera...')
			self._reopen_camera()
			return

		msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
		msg.header.stamp = self.get_clock().now().to_msg()
		msg.header.frame_id = self.frame_id
		self.image_pub.publish(msg)

	def destroy_node(self):
		if self.cap is not None:
			self.cap.release()
		super().destroy_node()


def main(args=None) -> None:
	rclpy.init(args=args)
	node = UsbCameraNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()

