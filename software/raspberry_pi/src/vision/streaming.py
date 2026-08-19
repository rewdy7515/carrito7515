"""Servidor HTTP y almacenamiento de las vistas de visión."""

from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

import cv2
import numpy as np

from .config import TuningState
from .types import WallGroundPoint


class FrameStore:
    """Guarda las cuatro vistas de cámara disponibles en el navegador."""

    def __init__(self) -> None:
        self.condition = threading.Condition()
        self.raw_jpeg: Optional[bytes] = None
        self.vision_jpeg: Optional[bytes] = None
        self.wall_mask_jpeg: Optional[bytes] = None
        self.line_mask_jpeg: Optional[bytes] = None
        self.wall_points: list[dict] = []
        self.version = 0

    def update(self, raw_frame: np.ndarray, vision_frame: np.ndarray, wall_mask_frame: np.ndarray, line_mask_frame: np.ndarray, wall_points: list[WallGroundPoint] | None = None) -> None:
        parameters = [cv2.IMWRITE_JPEG_QUALITY, 60]
        encoded = [cv2.imencode(".jpg", item, parameters) for item in (raw_frame, vision_frame, wall_mask_frame, line_mask_frame)]
        if not all(success for success, _ in encoded):
            return
        with self.condition:
            self.raw_jpeg, self.vision_jpeg, self.wall_mask_jpeg, self.line_mask_jpeg = (item.tobytes() for _, item in encoded)
            self.wall_points = [
                {"index": point.index, "pixel": point.pixel, "x_cm": round(point.x_cm, 3), "y_cm": round(point.y_cm, 3)}
                for point in (wall_points or [])
            ]
            self.version += 1
            self.condition.notify_all()

    def wait_for_new(self, last_version: int, stream_name: str) -> tuple[int, bytes]:
        with self.condition:
            self.condition.wait_for(lambda: self.version > last_version)
            image = {"raw": self.raw_jpeg, "vision": self.vision_jpeg, "wall_mask": self.wall_mask_jpeg, "line_mask": self.line_mask_jpeg}[stream_name]
            return self.version, image  # type: ignore[return-value]

    def wall_points_snapshot(self) -> list[dict]:
        with self.condition:
            return list(self.wall_points)


class CameraStreamHandler(BaseHTTPRequestHandler):
    store: FrameStore
    tuning_state: TuningState

    def _send_body(self, body: bytes, content_type: str) -> None:
        self.send_response(200); self.send_header("Content-Type", content_type); self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body)

    def _send_page(self) -> None:
        body = b'''<!doctype html><html><head><meta name="viewport" content="width=device-width, initial-scale=1"><title>Vision del robot</title><style>body{background:#202124;color:#eee;font-family:Arial;text-align:center}.views{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px;margin:auto;width:96vw}.view{background:#111}.view img{width:100%;height:40vh;object-fit:contain}@media(max-width:900px){.views{grid-template-columns:1fr}}</style></head><body><h1>Vision del robot</h1><p>YOLO detecta la forma; cada recorte de obstaculo se clasifica como rojo o verde.</p><main class="views"><section class="view"><h2>Camara original</h2><img src="/stream/raw"></section><section class="view"><h2>Vision procesada</h2><img src="/stream/vision"></section><section class="view"><h2>Mask muro / suelo</h2><img src="/stream/wall-mask"></section><section class="view"><h2>Mask de lineas</h2><img src="/stream/line-mask"></section></main></body></html>'''
        self._send_body(body, "text/html; charset=utf-8")

    def do_GET(self) -> None:  # noqa: N802
        if self.path == "/": self._send_page(); return
        if self.path == "/api/tuning": self._send_body(json.dumps(self.tuning_state.snapshot()).encode(), "application/json; charset=utf-8"); return
        if self.path == "/api/wall-points": self._send_body(json.dumps(self.store.wall_points_snapshot()).encode(), "application/json; charset=utf-8"); return
        stream = {"/stream/raw": "raw", "/stream/vision": "vision", "/stream/wall-mask": "wall_mask", "/stream/line-mask": "line_mask"}.get(self.path)
        if stream is None: self.send_error(404); return
        self.send_response(200); self.send_header("Cache-Control", "no-cache, no-store, must-revalidate"); self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame"); self.end_headers()
        version = 0
        try:
            while True:
                version, jpeg = self.store.wait_for_new(version, stream)
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n" + f"Content-Length: {len(jpeg)}\r\n\r\n".encode() + jpeg + b"\r\n")
        except (BrokenPipeError, ConnectionResetError): pass

    def do_POST(self) -> None:  # noqa: N802
        if self.path != "/api/tuning": self.send_error(404); return
        try:
            values = json.loads(self.rfile.read(int(self.headers.get("Content-Length", "0"))))
            self._send_body(json.dumps(self.tuning_state.update(values)).encode(), "application/json; charset=utf-8")
        except (ValueError, TypeError, json.JSONDecodeError, OSError): self.send_error(400, "JSON de ajustes invalido")

    def log_message(self, format: str, *args: object) -> None: return


def start_stream_server(host: str, port: int, store: FrameStore, tuning_state: TuningState) -> ThreadingHTTPServer:
    handler = type("BoundCameraStreamHandler", (CameraStreamHandler,), {"store": store, "tuning_state": tuning_state})
    server = ThreadingHTTPServer((host, port), handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server
