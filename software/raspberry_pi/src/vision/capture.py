"""Captura concurrente que conserva el frame más reciente."""

from __future__ import annotations

import threading
import time
from typing import Optional
import cv2
import numpy as np


class LatestCameraFrame:
    def __init__(
        self,
        source: int | str,
        width: int,
        height: int,
        *,
        focus_value: float | None = None,
        zoom_value: float | None = None,
    ) -> None:
        self.capture = cv2.VideoCapture(source)
        if not self.capture.isOpened(): raise RuntimeError(f"No se pudo abrir la cámara {source}")
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width); self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height); self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.optics_status = self._freeze_optics(focus_value, zoom_value)
        self.width, self.height, self.lock, self.latest, self.running = width, height, threading.Lock(), None, True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True); self.thread.start()

    def _freeze_optics(
        self, focus_value: float | None, zoom_value: float | None
    ) -> dict[str, float | str | None]:
        """Desactiva autofocus y vuelve a aplicar valores UVC una sola vez.

        ``None`` conserva el valor que la webcam tenía al iniciarse. No todas
        las cámaras exponen foco o zoom mediante OpenCV; esos casos se reportan
        sin impedir la captura.
        """
        status: dict[str, float | str | None] = {"focus": None, "zoom": None}
        try:
            autofocus_disabled = self.capture.set(cv2.CAP_PROP_AUTOFOCUS, 0)
            requested_focus = (
                float(focus_value)
                if focus_value is not None
                else self.capture.get(cv2.CAP_PROP_FOCUS)
            )
            if requested_focus >= 0 and self.capture.set(
                cv2.CAP_PROP_FOCUS, requested_focus
            ):
                actual_focus = self.capture.get(cv2.CAP_PROP_FOCUS)
                status["focus"] = (
                    requested_focus if actual_focus < 0 else float(actual_focus)
                )
            else:
                status["focus"] = "unsupported"
            if not autofocus_disabled and status["focus"] != "unsupported":
                status["focus"] = "manual value set; autofocus control unsupported"
        except (AttributeError, TypeError, ValueError, cv2.error):
            status["focus"] = "unsupported"

        try:
            requested_zoom = (
                float(zoom_value)
                if zoom_value is not None
                else self.capture.get(cv2.CAP_PROP_ZOOM)
            )
            if requested_zoom >= 0 and self.capture.set(
                cv2.CAP_PROP_ZOOM, requested_zoom
            ):
                actual_zoom = self.capture.get(cv2.CAP_PROP_ZOOM)
                status["zoom"] = (
                    requested_zoom if actual_zoom < 0 else float(actual_zoom)
                )
            else:
                status["zoom"] = "unsupported"
        except (AttributeError, TypeError, ValueError, cv2.error):
            status["zoom"] = "unsupported"
        return status

    def _capture_loop(self) -> None:
        while self.running:
            # ``read()`` puede bloquear hasta el próximo frame. No retener el
            # lock durante esa espera permite que el stream lea el último frame
            # disponible sin quedar detrás de la cámara USB.
            success, frame = self.capture.read()
            if not success: time.sleep(.01); continue
            if frame.shape[1] != self.width or frame.shape[0] != self.height: frame = cv2.resize(frame, (self.width, self.height), interpolation=cv2.INTER_AREA)
            with self.lock: self.latest = frame

    def read_latest(self) -> Optional[np.ndarray]:
        with self.lock: return None if self.latest is None else self.latest.copy()

    def close(self) -> None:
        self.running = False; self.thread.join(timeout=2)
        with self.lock: self.capture.release()
