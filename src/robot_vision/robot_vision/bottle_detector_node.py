#!/usr/bin/env python3

import os
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from robot_vision.utils.yolo_wrapper import YoloWrapper


class BottleDetectorNode(Node):
	def __init__(self) -> None:
		super().__init__('bottle_detector_node')

		self.declare_parameter('image_topic', '/camera/image_raw')
		self.declare_parameter('debug_image_topic', '/camera/detections')
		self.declare_parameter('model_path', '')
		self.declare_parameter('conf_threshold', 0.35)
		self.declare_parameter('iou_threshold', 0.45)
		self.declare_parameter('device', 'cpu')

		image_topic = str(self.get_parameter('image_topic').value)
		self.debug_image_topic = str(self.get_parameter('debug_image_topic').value)
		model_path = str(self.get_parameter('model_path').value)
		conf_threshold = float(self.get_parameter('conf_threshold').value)
		iou_threshold = float(self.get_parameter('iou_threshold').value)
		device = str(self.get_parameter('device').value)

		if not model_path:
			self.get_logger().warn('Parameter model_path is empty, detector disabled.')
			self.detector = None
		elif not os.path.exists(model_path):
			self.get_logger().error(f'Model file not found: {model_path}')
			self.detector = None
		else:
			self.detector = YoloWrapper(
				model_path=model_path,
				conf_threshold=conf_threshold,
				iou_threshold=iou_threshold,
				device=device,
			)
			self.get_logger().info(f'Loaded model: {model_path}')

		self.bridge = CvBridge()
		self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10)
		self.image_sub = self.create_subscription(Image, image_topic, self._on_image, 10)

		self.last_log_time = self.get_clock().now()

	def _on_image(self, msg: Image) -> None:
		frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

		if self.detector is None:
			self.debug_pub.publish(msg)
			return

		boxes = self.detector.infer(frame)

		for box in boxes:
			x1, y1, x2, y2 = box['x1'], box['y1'], box['x2'], box['y2']
			label = box['label']
			score = box['score']

			cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
			cv2.putText(
				frame,
				f'{label} {score:.2f}',
				(x1, max(0, y1 - 8)),
				cv2.FONT_HERSHEY_SIMPLEX,
				0.5,
				(0, 255, 0),
				2,
			)

		now = self.get_clock().now()
		if (now - self.last_log_time).nanoseconds > 1_000_000_000:
			self.get_logger().info(f'Detected objects: {len(boxes)}')
			self.last_log_time = now

		dbg_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
		dbg_msg.header = msg.header
		self.debug_pub.publish(dbg_msg)


def main(args=None) -> None:
	rclpy.init(args=args)
	node = BottleDetectorNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()

