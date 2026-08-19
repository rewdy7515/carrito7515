"""Componentes reutilizables de visión para la cámara del robot.

Este paquete es una extracción gradual de ``camera.py``.  Aún no sustituye
sus imports: mantener ambos permite migrar consumidores sin alterar el
comportamiento de la cámara que ya está en uso.
"""

from .capture import LatestCameraFrame
from .config import CAMERA_CALIBRATION_PATH, DEFAULT_TUNING, MODEL_PATH, TUNING_PATH, TuningState
from .ground_projection import GroundProjection, load_ground_projection
from .pipeline import analyze_frame
from .rendering import draw_result
from .streaming import FrameStore, start_stream_server
from .types import Detection, LineGeometry, VisionConfig, VisionResult, WallGroundPoint
from .yolo_ncnn import YoloNcnnDetector

__all__ = [
    "CAMERA_CALIBRATION_PATH", "DEFAULT_TUNING", "MODEL_PATH", "TUNING_PATH", "Detection", "FrameStore", "GroundProjection",
    "LatestCameraFrame", "LineGeometry", "TuningState", "VisionConfig", "WallGroundPoint",
    "VisionResult", "YoloNcnnDetector", "analyze_frame", "draw_result", "load_ground_projection", "start_stream_server",
]
