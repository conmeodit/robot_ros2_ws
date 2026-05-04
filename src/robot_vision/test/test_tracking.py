from robot_vision.detection_tracker import Detection, DetectionTracker, iou


def make_detection(bbox):
    return Detection(
        bbox=bbox,
        confidence=0.9,
        class_id=0,
        class_name='trash',
        anchor_pixel=(0.0, 0.0),
        base_xy=(0.0, 0.0),
        map_xy=(1.0, 1.0),
    )


def test_iou_handles_overlap_and_disjoint_boxes():
    assert iou((0.0, 0.0, 2.0, 2.0), (1.0, 1.0, 3.0, 3.0)) == 1.0 / 7.0
    assert iou((0.0, 0.0, 1.0, 1.0), (2.0, 2.0, 3.0, 3.0)) == 0.0


def test_tracker_marks_detection_stable_after_min_hits():
    tracker = DetectionTracker(min_hits=2, max_missed=1, iou_threshold=0.2)

    assert tracker.update([make_detection((0.0, 0.0, 1.0, 1.0))]) == []
    stable = tracker.update([make_detection((0.05, 0.0, 1.05, 1.0))])

    assert len(stable) == 1
    assert stable[0].class_name == 'trash'


def test_tracker_expires_after_missed_frame_budget():
    tracker = DetectionTracker(min_hits=1, max_missed=1, iou_threshold=0.2)

    assert len(tracker.update([make_detection((0.0, 0.0, 1.0, 1.0))])) == 1
    assert tracker.update([]) == []
    assert tracker.update([]) == []
    assert tracker.tracks == []
