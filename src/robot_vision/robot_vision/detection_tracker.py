from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple


BBox = Tuple[float, float, float, float]


@dataclass
class Detection:
    bbox: BBox
    confidence: float
    class_id: int
    class_name: str
    anchor_pixel: Tuple[float, float]
    base_xy: Optional[Tuple[float, float]]
    map_xy: Optional[Tuple[float, float]]


@dataclass
class Track:
    detection: Detection
    hits: int = 1
    missed: int = 0


def iou(a: BBox, b: BBox) -> float:
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b
    inter_x0 = max(ax0, bx0)
    inter_y0 = max(ay0, by0)
    inter_x1 = min(ax1, bx1)
    inter_y1 = min(ay1, by1)
    inter_w = max(0.0, inter_x1 - inter_x0)
    inter_h = max(0.0, inter_y1 - inter_y0)
    inter_area = inter_w * inter_h
    if inter_area <= 0.0:
        return 0.0

    area_a = max(0.0, ax1 - ax0) * max(0.0, ay1 - ay0)
    area_b = max(0.0, bx1 - bx0) * max(0.0, by1 - by0)
    union = area_a + area_b - inter_area
    if union <= 0.0:
        return 0.0
    return inter_area / union


class DetectionTracker:
    def __init__(
        self,
        *,
        min_hits: int = 2,
        max_missed: int = 3,
        iou_threshold: float = 0.25,
    ):
        self.min_hits = max(1, int(min_hits))
        self.max_missed = max(0, int(max_missed))
        self.iou_threshold = max(0.0, min(1.0, float(iou_threshold)))
        self.tracks: List[Track] = []

    def update(self, detections: Sequence[Detection]) -> List[Detection]:
        detections = list(detections)
        if not detections:
            self._mark_all_missed()
            return self.stable_detections()

        assigned_tracks = set()
        assigned_detections = set()

        matches = []
        for track_index, track in enumerate(self.tracks):
            for detection_index, detection in enumerate(detections):
                score = iou(track.detection.bbox, detection.bbox)
                if score >= self.iou_threshold:
                    matches.append((score, track_index, detection_index))

        matches.sort(reverse=True)
        for _, track_index, detection_index in matches:
            if track_index in assigned_tracks or detection_index in assigned_detections:
                continue
            track = self.tracks[track_index]
            track.detection = self._merge_detection(track.detection, detections[detection_index])
            track.hits += 1
            track.missed = 0
            assigned_tracks.add(track_index)
            assigned_detections.add(detection_index)

        for track_index, track in enumerate(self.tracks):
            if track_index not in assigned_tracks:
                track.missed += 1

        for detection_index, detection in enumerate(detections):
            if detection_index not in assigned_detections:
                self.tracks.append(Track(detection=detection))

        self.tracks = [track for track in self.tracks if track.missed <= self.max_missed]
        return self.stable_detections()

    def stable_detections(self) -> List[Detection]:
        return [
            track.detection
            for track in self.tracks
            if track.hits >= self.min_hits and track.missed <= self.max_missed
        ]

    def _mark_all_missed(self):
        for track in self.tracks:
            track.missed += 1
        self.tracks = [track for track in self.tracks if track.missed <= self.max_missed]

    @staticmethod
    def _merge_detection(previous: Detection, current: Detection) -> Detection:
        return Detection(
            bbox=current.bbox,
            confidence=current.confidence,
            class_id=current.class_id,
            class_name=current.class_name,
            anchor_pixel=current.anchor_pixel,
            base_xy=current.base_xy if current.base_xy is not None else previous.base_xy,
            map_xy=current.map_xy if current.map_xy is not None else previous.map_xy,
        )
