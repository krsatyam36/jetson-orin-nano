"""
Shared YOLO detection engine — supports both PyTorch (.pt) and TensorRT (.engine) backends.

Usage:
    from vision.detector import Detector

    det = Detector("models/yolov8n.pt", conf_threshold=0.4)
    results = det.detect(frame)
    for d in results:
        print(d.track_id, d.bbox, d.confidence, d.class_name)

Automatic backend selection:
  - .engine files → TensorRT (faster on Jetson)
  - .pt files      → PyTorch (fallback)
  - .onnx files    → ONNX Runtime (if available)
"""

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None

from collections import namedtuple
from pathlib import Path

Detection = namedtuple(
    "Detection", ["track_id", "bbox", "confidence", "class_name", "class_id"]
)


class Detector:
    """YOLO object detector with pluggable backends.

    Automatically selects the inference backend based on file extension.
    Provides a consistent interface so vision scripts never need to touch
    Ultralytics / TensorRT APIs directly.
    """

    def __init__(self, model_path: str, conf_threshold: float = 0.4):
        self.model_path = Path(model_path)
        self.conf_threshold = conf_threshold
        self._backend = None
        self._model = None
        self._class_names = None
        self._load()

    # ── Public API ──────────────────────────────────────────────────────

    def detect(self, frame: np.ndarray) -> list[Detection]:
        """Run inference on a single frame.

        Returns a list of Detection namedtuples. Each detection includes
        track_id (always None for single-frame inference; populated when
        track() is used downstream).
        """
        return self._run_inference(frame)

    def track(self, frame: np.ndarray, persist: bool = True) -> list[Detection]:
        """Run inference with cross-frame tracking (ByteTrack).

        Returns detections with stable track_id across consecutive frames.
        Falls back to detect() if the backend doesn't support tracking.
        """
        return self._run_inference(frame, track=True, persist=persist)

    @property
    def class_names(self) -> list[str]:
        return self._class_names

    @property
    def backend(self) -> str:
        return self._backend

    # ── Backend loading ─────────────────────────────────────────────────

    def _load(self):
        ext = self.model_path.suffix.lower()
        if ext == ".engine":
            self._load_tensorrt()
        elif ext == ".onnx":
            self._load_onnx()
        else:
            self._load_ultralytics()

    def _load_ultralytics(self):
        try:
            from ultralytics import YOLO
        except ImportError:
            raise ImportError("Ultralytics not installed. Run: pip install ultralytics")

        self._model = YOLO(str(self.model_path))
        self._class_names = list(self._model.names.values())
        self._backend = "ultralytics"
        print(f"[detector] Loaded {self.model_path.name} (backend: ultralytics)")

    def _load_tensorrt(self):
        try:
            import tensorrt as trt  # noqa: F401
            import pycuda.driver as cuda  # noqa: F401
            import pycuda.autoinit  # noqa: F401
        except ImportError:
            print(
                "[detector] TensorRT/pycuda not available, falling back to ultralytics"
            )
            self._load_ultralytics()
            return

        try:
            from ultralytics import YOLO

            self._model = YOLO(str(self.model_path))
            self._class_names = list(self._model.names.values())
            self._backend = "tensorrt"
            print(f"[detector] Loaded {self.model_path.name} (backend: TensorRT)")
        except Exception:
            self._load_ultralytics()

    def _load_onnx(self):
        try:
            import onnxruntime as ort  # noqa: F401
        except ImportError:
            print("[detector] ONNX Runtime not available, falling back to ultralytics")
            self._load_ultralytics()
            return

        try:
            from ultralytics import YOLO

            self._model = YOLO(str(self.model_path))
            self._class_names = list(self._model.names.values())
            self._backend = "onnx"
            print(f"[detector] Loaded {self.model_path.name} (backend: ONNX)")
        except Exception:
            self._load_ultralytics()

    # ── Inference ────────────────────────────────────────────────────────

    def _run_inference(
        self, frame: np.ndarray, track: bool = False, persist: bool = True
    ) -> list[Detection]:
        if self._model is None:
            return []

        if track:
            results = self._model.track(frame, persist=persist, verbose=False)
        else:
            results = self._model(frame, verbose=False)

        detections = []
        if len(results) == 0:
            return detections

        r = results[0]
        if r.boxes is None:
            return detections

        boxes_xyxy = (
            r.boxes.xyxy.int().cpu().tolist()
            if hasattr(r.boxes.xyxy, "cpu")
            else r.boxes.xyxy.int().tolist()
        )
        class_ids = (
            r.boxes.cls.int().cpu().tolist()
            if hasattr(r.boxes.cls, "cpu")
            else r.boxes.cls.int().tolist()
        )
        confs = (
            r.boxes.conf.cpu().tolist()
            if hasattr(r.boxes.conf, "cpu")
            else r.boxes.conf.tolist()
        )

        track_ids = None
        if track and r.boxes.id is not None:
            tid = (
                r.boxes.id.int().cpu().tolist()
                if hasattr(r.boxes.id, "cpu")
                else r.boxes.id.int().tolist()
            )
            track_ids = tid

        for i in range(len(boxes_xyxy)):
            conf = confs[i]
            if conf < self.conf_threshold:
                continue

            class_id = class_ids[i]
            class_name = (
                self._class_names[class_id]
                if class_id < len(self._class_names)
                else "unknown"
            )
            tid = track_ids[i] if track_ids else None

            detections.append(
                Detection(
                    track_id=tid,
                    bbox=boxes_xyxy[i],
                    confidence=conf,
                    class_name=class_name,
                    class_id=class_id,
                )
            )

        return detections


def draw_detections(
    frame: np.ndarray, detections: list[Detection], label: bool = True
) -> None:
    """Draw bounding boxes and labels on a frame (in-place)."""
    for d in detections:
        if d.track_id is not None:
            color = (0, 0, 255)  # red for tracked
        else:
            color = (0, 255, 0)  # green for untracked

        x1, y1, x2, y2 = d.bbox
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

        if label:
            text = d.class_name
            if d.track_id is not None:
                text += f" #{d.track_id}"
            text += f" {d.confidence:.2f}"
            cv2.putText(
                frame, text, (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2
            )
