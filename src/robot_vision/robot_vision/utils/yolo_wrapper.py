import cv2
from ultralytics import YOLO

class YOLOWrapper:
    def __init__(self, model_path):
        """
        Initialize YOLO model
        Args:
            model_path: Path to best.pt model
        """
        self.model = YOLO(model_path)
        self.confidence_threshold = 0.5

    def detect(self, frame):
        """
        Run inference on frame
        Args:
            frame: OpenCV image (BGR format)
        Returns:
            detections: List of detection results
        """
        results = self.model(frame, conf=self.confidence_threshold, verbose=False)
        return results

    def draw_boxes(self, frame, results):
        """
        Draw bounding boxes on frame
        Args:
            frame: OpenCV image (BGR format)
            results: YOLO detection results
        Returns:
            frame: Image with boxes drawn
        """
        for result in results:
            boxes = result.boxes
            for box in boxes:
                # Get coordinates
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = box.conf[0]
                cls_id = int(box.cls[0])

                # Get class name
                class_name = self.model.names.get(cls_id, f"Class {cls_id}")

                # Draw rectangle
                color = (0, 255, 0)  # Green
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

                # Draw label
                label = f"{class_name} {conf:.2f}"
                label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(frame, (x1, y1 - label_size[1] - 5),
                            (x1 + label_size[0], y1), color, -1)
                cv2.putText(frame, label, (x1, y1 - 5),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        return frame

    def set_confidence(self, confidence):
        """Set confidence threshold (0.0 to 1.0)"""
        self.confidence_threshold = max(0.0, min(1.0, confidence))
