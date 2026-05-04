import math
import os
import unicodedata
from typing import List, Optional, Sequence, Set, Tuple

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import Pose, PoseArray
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from robot_vision.detection_tracker import Detection, DetectionTracker
from robot_vision.ground_projection import GroundProjector, transform_xy, yaw_from_quaternion


def normalize_name(value: str) -> str:
    normalized = unicodedata.normalize('NFD', value)
    ascii_value = normalized.encode('ascii', 'ignore').decode('ascii')
    return ascii_value.strip().lower()


class RobotVisionNode(Node):
    """Detect floor trash with YOLO and publish map-frame dynamic obstacles."""

    def __init__(self):
        super().__init__('robot_vision_node')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('debug_image_topic', '/vision/debug_image')
        self.declare_parameter('trash_obstacles_topic', '/vision/trash_obstacles')
        self.declare_parameter('trash_markers_topic', '/vision/trash_markers')
        self.declare_parameter('model_path', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('confidence_threshold', 0.45)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('image_size', 640)
        self.declare_parameter('device', '')
        self.declare_parameter('detection_rate_hz', 8.0)
        self.declare_parameter('bbox_anchor', 'center')
        self.declare_parameter('projection_method', 'homography')
        self.declare_parameter('camera_height_m', 1.0)
        self.declare_parameter('camera_offset_x_m', 0.0)
        self.declare_parameter('camera_offset_y_m', 0.0)
        self.declare_parameter('camera_yaw_rad', 0.0)
        self.declare_parameter('homography', [0.0] * 9)
        self.declare_parameter('trash_class_ids', [0])
        self.declare_parameter('trash_class_names', ['trash', 'rac', 'garbage', 'waste'])
        self.declare_parameter('tracker_min_hits', 2)
        self.declare_parameter('tracker_max_missed', 3)
        self.declare_parameter('tracker_iou_threshold', 0.25)
        self.declare_parameter('tf_lookup_timeout_sec', 0.12)
        self.declare_parameter('marker_scale_m', 0.16)
        self.declare_parameter('publish_debug_image', True)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self.debug_image_topic = str(self.get_parameter('debug_image_topic').value)
        self.trash_obstacles_topic = str(self.get_parameter('trash_obstacles_topic').value)
        self.trash_markers_topic = str(self.get_parameter('trash_markers_topic').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.iou_threshold = float(self.get_parameter('iou_threshold').value)
        self.image_size = max(64, int(self.get_parameter('image_size').value))
        self.device = str(self.get_parameter('device').value).strip()
        self.detection_period_sec = 1.0 / max(
            0.1, float(self.get_parameter('detection_rate_hz').value)
        )
        self.bbox_anchor = str(self.get_parameter('bbox_anchor').value).strip().lower()
        self.tf_lookup_timeout_sec = max(
            0.01, float(self.get_parameter('tf_lookup_timeout_sec').value)
        )
        self.marker_scale_m = max(0.03, float(self.get_parameter('marker_scale_m').value))
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)

        self.trash_class_ids = self._read_int_set('trash_class_ids', default={0})
        self.trash_class_names = self._read_name_set(
            'trash_class_names',
            default={'trash', 'rac', 'garbage', 'waste'},
        )

        self.projector = GroundProjector(
            projection_method=str(self.get_parameter('projection_method').value),
            camera_height_m=float(self.get_parameter('camera_height_m').value),
            camera_offset_x_m=float(self.get_parameter('camera_offset_x_m').value),
            camera_offset_y_m=float(self.get_parameter('camera_offset_y_m').value),
            camera_yaw_rad=float(self.get_parameter('camera_yaw_rad').value),
            homography=list(self.get_parameter('homography').value or []),
        )
        if self.projector.projection_method == 'homography' and self.projector.homography is None:
            self.get_logger().warn(
                'Vision homography is not calibrated. BBoxes can be shown, but no map '
                'obstacle will be published until config/vision.yaml has a valid homography.'
            )

        self.tracker = DetectionTracker(
            min_hits=int(self.get_parameter('tracker_min_hits').value),
            max_missed=int(self.get_parameter('tracker_max_missed').value),
            iou_threshold=float(self.get_parameter('tracker_iou_threshold').value),
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_inference_time = 0.0
        self.last_warning_time = 0.0

        self.bridge = None
        self.cv2 = None
        self.model = None
        self.model_ready = self._load_runtime_dependencies_and_model()

        self.obstacles_pub = self.create_publisher(PoseArray, self.trash_obstacles_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.trash_markers_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10)

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_cb,
            10,
        )
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_cb,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'robot_vision ready: image={self.image_topic}, '
            f'obstacles={self.trash_obstacles_topic}, debug={self.debug_image_topic}, '
            f'model={"loaded" if self.model_ready else "not loaded"}'
        )

    def _read_int_set(self, name: str, default: Set[int]) -> Set[int]:
        values = self.get_parameter(name).value
        if values is None:
            return set(default)
        return {int(value) for value in values}

    def _read_name_set(self, name: str, default: Set[str]) -> Set[str]:
        values = self.get_parameter(name).value
        if values is None:
            return set(default)
        names = {normalize_name(str(value)) for value in values if str(value).strip()}
        return names if names else set(default)

    def _load_runtime_dependencies_and_model(self) -> bool:
        try:
            import cv2  # pylint: disable=import-outside-toplevel
            from cv_bridge import CvBridge  # pylint: disable=import-outside-toplevel
            from ultralytics import YOLO  # pylint: disable=import-outside-toplevel
        except Exception as exc:
            self.get_logger().error(
                'robot_vision dependencies are missing. Install cv_bridge, OpenCV, '
                f'and ultralytics. Detail: {exc}'
            )
            return False

        model_path = self._resolve_model_path()
        if not model_path or not os.path.exists(model_path):
            self.get_logger().error(
                'YOLO model not found. Put best.pt in src/robot_vision/models/ '
                'or pass vision_model_path:=/path/to/best.pt.'
            )
            self.cv2 = cv2
            self.bridge = CvBridge()
            return False

        try:
            self.model = YOLO(model_path)
        except Exception as exc:
            self.get_logger().error(f'Failed to load YOLO model {model_path}: {exc}')
            self.cv2 = cv2
            self.bridge = CvBridge()
            return False

        self.cv2 = cv2
        self.bridge = CvBridge()
        self.get_logger().info(f'Loaded YOLO model: {model_path}')
        return True

    def _resolve_model_path(self) -> str:
        configured = str(self.get_parameter('model_path').value).strip()
        if configured:
            return os.path.expanduser(configured)

        candidates = []
        try:
            pkg_share = get_package_share_directory('robot_vision')
            candidates.append(os.path.join(pkg_share, 'models', 'best.pt'))
        except PackageNotFoundError:
            pass

        source_candidate = os.path.abspath(
            os.path.join(os.path.dirname(__file__), '..', 'models', 'best.pt')
        )
        candidates.append(source_candidate)

        for candidate in candidates:
            if os.path.exists(candidate):
                return candidate
        return candidates[0] if candidates else ''

    def camera_info_cb(self, msg: CameraInfo):
        self.projector.set_camera_info(msg.k)

    def image_cb(self, msg: Image):
        now = self.now_sec()
        if (now - self.last_inference_time) < self.detection_period_sec:
            return
        self.last_inference_time = now

        if self.bridge is None or self.cv2 is None:
            return

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self._warn_throttled(f'Failed to convert camera image: {exc}')
            return

        detections: List[Detection] = []
        if self.model_ready and self.model is not None:
            detections = self._detect_trash(msg, image)
        stable_detections = self.tracker.update(detections)

        self._publish_obstacles(msg.header.stamp, stable_detections)
        self._publish_markers(msg.header.stamp, stable_detections)
        if self.publish_debug_image:
            self._publish_debug_image(msg, image, detections, stable_detections)

    def _detect_trash(self, msg: Image, image) -> List[Detection]:
        predict_kwargs = {
            'source': image,
            'imgsz': self.image_size,
            'conf': self.confidence_threshold,
            'iou': self.iou_threshold,
            'verbose': False,
        }
        if self.device:
            predict_kwargs['device'] = self.device

        try:
            results = self.model.predict(**predict_kwargs)
        except Exception as exc:
            self._warn_throttled(f'YOLO inference failed: {exc}')
            return []

        if not results:
            return []

        transform = self._lookup_base_to_map(msg.header.stamp)
        detections: List[Detection] = []
        result = results[0]
        boxes = getattr(result, 'boxes', None)
        if boxes is None:
            return detections

        for box in boxes:
            class_id = int(box.cls[0])
            class_name = self._class_name(class_id)
            if not self._is_trash_class(class_id, class_name):
                continue

            confidence = float(box.conf[0])
            x0, y0, x1, y1 = [float(value) for value in box.xyxy[0].tolist()]
            anchor = self._bbox_anchor((x0, y0, x1, y1))
            base_xy = self.projector.project_pixel(anchor[0], anchor[1])
            map_xy = None
            if base_xy is not None and transform is not None:
                map_xy = self._transform_base_to_map(base_xy[0], base_xy[1], transform)

            detections.append(
                Detection(
                    bbox=(x0, y0, x1, y1),
                    confidence=confidence,
                    class_id=class_id,
                    class_name=class_name,
                    anchor_pixel=anchor,
                    base_xy=base_xy,
                    map_xy=map_xy,
                )
            )
        return detections

    def _class_name(self, class_id: int) -> str:
        names = getattr(self.model, 'names', {})
        if isinstance(names, dict):
            return str(names.get(class_id, class_id))
        if isinstance(names, (list, tuple)) and 0 <= class_id < len(names):
            return str(names[class_id])
        return str(class_id)

    def _is_trash_class(self, class_id: int, class_name: str) -> bool:
        if class_id in self.trash_class_ids:
            return True
        return normalize_name(class_name) in self.trash_class_names

    def _bbox_anchor(self, bbox: Sequence[float]) -> Tuple[float, float]:
        x0, y0, x1, y1 = bbox
        center_x = 0.5 * (x0 + x1)
        if self.bbox_anchor in ('bottom', 'bottom_center', 'feet'):
            return center_x, y1
        return center_x, 0.5 * (y0 + y1)

    def _lookup_base_to_map(self, stamp_msg):
        stamp = Time.from_msg(stamp_msg)
        if stamp.nanoseconds == 0:
            stamp = Time()
        try:
            return self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                stamp,
                timeout=Duration(seconds=self.tf_lookup_timeout_sec),
            )
        except TransformException as exc:
            self._warn_throttled(f'Waiting for TF {self.map_frame}->{self.base_frame}: {exc}')
            return None

    def _transform_base_to_map(self, x: float, y: float, transform) -> Tuple[float, float]:
        trans = transform.transform.translation
        rot = transform.transform.rotation
        yaw = yaw_from_quaternion(rot.x, rot.y, rot.z, rot.w)
        return transform_xy(x, y, float(trans.x), float(trans.y), yaw)

    def _publish_obstacles(self, stamp_msg, detections: Sequence[Detection]):
        pose_array = PoseArray()
        pose_array.header.frame_id = self.map_frame
        pose_array.header.stamp = stamp_msg

        for detection in detections:
            if detection.map_xy is None:
                continue
            pose = Pose()
            pose.position.x = float(detection.map_xy[0])
            pose.position.y = float(detection.map_xy[1])
            pose.position.z = float(detection.confidence)
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

        self.obstacles_pub.publish(pose_array)

    def _publish_markers(self, stamp_msg, detections: Sequence[Detection]):
        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.header.frame_id = self.map_frame
        delete_marker.header.stamp = stamp_msg
        delete_marker.ns = 'vision_trash'
        delete_marker.id = 0
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        marker_id = 1
        for detection in detections:
            if detection.map_xy is None:
                continue

            obstacle = self._base_marker(marker_id, Marker.SPHERE, stamp_msg)
            marker_id += 1
            obstacle.pose.position.x = float(detection.map_xy[0])
            obstacle.pose.position.y = float(detection.map_xy[1])
            obstacle.pose.position.z = 0.06
            obstacle.scale.x = self.marker_scale_m
            obstacle.scale.y = self.marker_scale_m
            obstacle.scale.z = 0.08
            obstacle.color.r = 1.0
            obstacle.color.g = 0.12
            obstacle.color.b = 0.08
            obstacle.color.a = 0.90
            marker_array.markers.append(obstacle)

            label = self._base_marker(marker_id, Marker.TEXT_VIEW_FACING, stamp_msg)
            marker_id += 1
            label.pose.position.x = float(detection.map_xy[0])
            label.pose.position.y = float(detection.map_xy[1])
            label.pose.position.z = 0.22
            label.scale.z = 0.12
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 0.95
            label.text = f'{detection.class_name} {detection.confidence:.2f}'
            marker_array.markers.append(label)

        self.marker_pub.publish(marker_array)

    def _base_marker(self, marker_id: int, marker_type: int, stamp_msg) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = stamp_msg
        marker.ns = 'vision_trash'
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime.sec = 1
        return marker

    def _publish_debug_image(
        self,
        msg: Image,
        image,
        detections: Sequence[Detection],
        stable_detections: Sequence[Detection],
    ):
        debug = image.copy()
        stable_boxes = {tuple(detection.bbox) for detection in stable_detections}

        for detection in detections:
            x0, y0, x1, y1 = [int(round(value)) for value in detection.bbox]
            stable = tuple(detection.bbox) in stable_boxes
            color = (0, 255, 0) if stable else (0, 180, 255)
            self.cv2.rectangle(debug, (x0, y0), (x1, y1), color, 2)
            ax, ay = [int(round(value)) for value in detection.anchor_pixel]
            self.cv2.circle(debug, (ax, ay), 4, color, -1)
            label = f'{detection.class_name} {detection.confidence:.2f}'
            if detection.map_xy is None:
                label += ' no-map'
            self.cv2.putText(
                debug,
                label,
                (x0, max(18, y0 - 6)),
                self.cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                self.cv2.LINE_AA,
            )

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        except Exception as exc:
            self._warn_throttled(f'Failed to publish debug image: {exc}')
            return
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)

    def _warn_throttled(self, message: str):
        now = self.now_sec()
        if (now - self.last_warning_time) < 2.0:
            return
        self.last_warning_time = now
        self.get_logger().warn(message)

    def now_sec(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = RobotVisionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
