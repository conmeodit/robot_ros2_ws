#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import cv2
from robot_vision.utils.yolo_wrapper import YOLOWrapper

class BottleDetectorNode(Node):
    def __init__(self):
        super().__init__('bottle_detector_node')

        self.declare_parameter('model_path', '')
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/detection/image_boxes')
        self.declare_parameter('confidence', 0.5)

        model_path = self.get_parameter('model_path').value
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        confidence = self.get_parameter('confidence').value

        # If model_path is empty, look for best.pt in package directory
        if not model_path:
            package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            model_path = os.path.join(package_dir, 'models', 'best.pt')

        self.get_logger().info(f"Loading model from: {model_path}")

        if not os.path.exists(model_path):
            self.get_logger().error(f"Model not found: {model_path}")
            raise FileNotFoundError(f"Model not found: {model_path}")

        self.detector = YOLOWrapper(model_path)
        self.detector.set_confidence(confidence)

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(Image, self.output_topic, 10)

        self.get_logger().info(f"Detector initialized. Input: {self.input_topic}, Output: {self.output_topic}")

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Run detection
            results = self.detector.detect(cv_image)

            # Draw boxes
            output_image = self.detector.draw_boxes(cv_image, results)

            # Convert back to ROS message
            output_msg = self.bridge.cv2_to_imgmsg(output_image, encoding='bgr8')
            output_msg.header = msg.header

            # Publish
            self.publisher.publish(output_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BottleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
