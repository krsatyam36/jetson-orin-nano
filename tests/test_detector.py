"""
Tests for vision/detector.py
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
from vision.detector import Detector, Detection, draw_detections


def test_detection_namedtuple():
    d = Detection(track_id=1, bbox=[10, 20, 100, 200], confidence=0.85, class_name="person", class_id=0)
    assert d.track_id == 1
    assert d.bbox == [10, 20, 100, 200]
    assert d.confidence == 0.85
    assert d.class_name == "person"
    assert d.class_id == 0


def test_draw_detections_empty():
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    draw_detections(frame, [])
    # Should not crash


def test_draw_detections_with_items():
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    dets = [
        Detection(track_id=1, bbox=[10, 20, 100, 200], confidence=0.85, class_name="person", class_id=0),
        Detection(track_id=None, bbox=[200, 100, 300, 300], confidence=0.72, class_name="car", class_id=2),
    ]
    draw_detections(frame, dets, label=True)
    # Should not crash — visual check omitted
