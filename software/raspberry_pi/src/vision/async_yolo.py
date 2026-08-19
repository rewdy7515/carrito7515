"""Ejecución no bloqueante de YOLO para mantener fluida la cámara."""

from __future__ import annotations

import threading
from typing import Optional

import numpy as np

from .types import Detection
from .yolo_ncnn import YoloNcnnDetector


YoloDetections = tuple[list[Detection], list[Detection]]


class AsyncYoloDetector:
    """Procesa solo el frame pendiente más reciente en un hilo dedicado."""

    def __init__(self, detector: YoloNcnnDetector) -> None:
        self.detector = detector
        self.condition = threading.Condition()
        self.pending: Optional[tuple[np.ndarray, dict]] = None
        self.latest: YoloDetections = ([], [])
        self.last_error: Optional[RuntimeError] = None
        self.running = True
        self.thread = threading.Thread(target=self._run, name="yolo-ncnn", daemon=True)
        self.thread.start()

    def submit(self, frame: np.ndarray, tuning: dict) -> None:
        """Reemplaza el trabajo pendiente: nunca acumula frames antiguos."""
        with self.condition:
            self.pending = (frame.copy(), tuning)
            self.condition.notify()

    def snapshot(self) -> YoloDetections:
        with self.condition:
            return self.latest

    def close(self) -> None:
        with self.condition:
            self.running = False
            self.condition.notify()
        self.thread.join(timeout=2.0)

    def _run(self) -> None:
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.pending is not None or not self.running)
                if not self.running:
                    return
                frame, tuning = self.pending
                self.pending = None
            try:
                detections = self.detector.detect(frame, tuning)
            except RuntimeError as error:
                with self.condition:
                    self.last_error = error
                continue
            with self.condition:
                self.latest = detections
                self.last_error = None
