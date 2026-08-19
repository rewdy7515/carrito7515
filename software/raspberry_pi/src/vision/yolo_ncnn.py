"""Inferencia YOLO/NCNN directa y clasificación de color por recorte."""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

from .colors import GREEN_RGB, MAGENTA_RGB, RED_RGB, clean_mask, color_mask
from .types import Detection


class YoloNcnnDetector:
    """Detecta forma con YOLO; el color se decide solo dentro del recorte."""

    def __init__(
        self,
        model_path: Path,
        confidence: float = 0.35,
        iou: float = 0.45,
        image_size: int = 256,
        threads: int = 3,
    ) -> None:
        if not (model_path / "model.ncnn.param").is_file() or not (model_path / "model.ncnn.bin").is_file():
            raise RuntimeError(f"Modelo NCNN incompleto: {model_path}")
        try:
            import ncnn
        except ImportError as error:
            raise RuntimeError(
                "Falta NCNN. Ejecuta: ./.venv/bin/python -m pip install ncnn"
            ) from error
        self.ncnn = ncnn
        self.network = ncnn.Net()
        self.network.opt.num_threads = max(1, threads)
        if self.network.load_param(str(model_path / "model.ncnn.param")) not in (None, 0):
            raise RuntimeError("NCNN no pudo cargar model.ncnn.param")
        if self.network.load_model(str(model_path / "model.ncnn.bin")) not in (None, 0):
            raise RuntimeError("NCNN no pudo cargar model.ncnn.bin")
        self.confidence = confidence
        self.iou = iou
        self.image_size = image_size

    def detect(self, frame: np.ndarray, tuning: dict) -> tuple[list[Detection], list[Detection]]:
        """Devuelve ``(obstacles, parking_walls)`` del frame BGR actual."""
        image, scale, pad_x, pad_y = _letterbox(frame, self.image_size)
        chw_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB).transpose(2, 0, 1)
        input_tensor = np.ascontiguousarray(chw_rgb, dtype=np.float32) / 255.0
        with self.network.create_extractor() as extractor:
            if extractor.input("in0", self.ncnn.Mat(input_tensor).clone()) not in (None, 0):
                raise RuntimeError("NCNN no pudo recibir el frame de cámara")
            status, output = extractor.extract("out0")
        if status != 0:
            raise RuntimeError("NCNN no pudo ejecutar la inferencia")

        predictions = _normalize_predictions(np.asarray(output))
        candidates = []
        for prediction in predictions:
            class_id = int(np.argmax(prediction[4:]))
            confidence = float(prediction[4 + class_id])
            if confidence < self.confidence or class_id not in (0, 1):
                continue
            center_x, center_y, width, height = (float(value) for value in prediction[:4])
            candidates.append((
                class_id,
                confidence,
                (center_x - width / 2 - pad_x) / scale,
                (center_y - height / 2 - pad_y) / scale,
                (center_x + width / 2 - pad_x) / scale,
                (center_y + height / 2 - pad_y) / scale,
            ))
        obstacles: list[Detection] = []
        parking_walls: list[Detection] = []
        for class_id, confidence, left, top, right, bottom in _class_aware_nms(candidates, self.iou):
            x1, y1 = max(0, round(left)), max(0, round(top))
            x2, y2 = min(frame.shape[1], round(right)), min(frame.shape[0], round(bottom))
            if x2 <= x1 or y2 <= y1:
                continue
            # La forma de YOLO puede confundir un pilar rectangular con el
            # parking wall. El color físico define la clase final: los únicos
            # obstáculos válidos son rojo/verde y el parking wall es magenta.
            color = classify_detected_color(frame[y1:y2, x1:x2], tuning)
            if color in ("red", "green"):
                class_name = "obstacle"
            elif color == "magenta":
                class_name = "parking_wall"
            else:
                continue
            width, height = x2 - x1, y2 - y1
            detection = Detection(
                kind=class_name,
                color=color,
                bounding_box=(x1, y1, width, height),
                center=(x1 + width // 2, y1 + height // 2),
                area=float(width * height),
                confidence=confidence,
            )
            if class_name == "obstacle":
                obstacles.append(detection)
            elif class_name == "parking_wall":
                parking_walls.append(detection)
        return obstacles, parking_walls


def _letterbox(frame: np.ndarray, image_size: int) -> tuple[np.ndarray, float, int, int]:
    """Escala sin deformar y rellena como YOLO antes de pasar el tensor a NCNN."""
    height, width = frame.shape[:2]
    scale = min(image_size / width, image_size / height)
    resized_width, resized_height = round(width * scale), round(height * scale)
    resized = cv2.resize(frame, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
    pad_x, pad_y = (image_size - resized_width) // 2, (image_size - resized_height) // 2
    image = np.full((image_size, image_size, 3), 114, dtype=np.uint8)
    image[pad_y:pad_y + resized_height, pad_x:pad_x + resized_width] = resized
    return image, scale, pad_x, pad_y


def _normalize_predictions(output: np.ndarray) -> np.ndarray:
    """Normaliza ``out0`` de NCNN a una fila ``cx, cy, w, h, clase...``."""
    values = np.squeeze(output).astype(np.float32, copy=False)
    if values.ndim != 2:
        raise RuntimeError(f"Salida NCNN inesperada: shape={values.shape}")
    # El modelo exportado concatena 4 coordenadas y 2 clases: (6, 1344).
    if values.shape[0] == 6:
        return values.T
    if values.shape[1] == 6:
        return values
    raise RuntimeError(f"Salida NCNN inesperada: shape={values.shape}; se esperaba 6xN")


def _class_aware_nms(candidates: list[tuple[int, float, float, float, float, float]], iou_threshold: float) -> list[tuple[int, float, float, float, float, float]]:
    kept = []
    for class_id in (0, 1):
        pending = sorted((item for item in candidates if item[0] == class_id), key=lambda item: item[1], reverse=True)
        while pending:
            selected = pending.pop(0)
            kept.append(selected)
            pending = [item for item in pending if _iou(selected[2:], item[2:]) < iou_threshold]
    return kept


def _iou(first: tuple[float, float, float, float], second: tuple[float, float, float, float]) -> float:
    left, top = max(first[0], second[0]), max(first[1], second[1])
    right, bottom = min(first[2], second[2]), min(first[3], second[3])
    intersection = max(0.0, right - left) * max(0.0, bottom - top)
    first_area = max(0.0, first[2] - first[0]) * max(0.0, first[3] - first[1])
    second_area = max(0.0, second[2] - second[0]) * max(0.0, second[3] - second[1])
    return intersection / max(first_area + second_area - intersection, 1e-6)


def classify_detected_color(crop_bgr: np.ndarray, tuning: dict) -> str:
    """Clasifica el recorte YOLO como rojo, verde, magenta o desconocido."""
    if crop_bgr.size == 0:
        return "unknown"
    hsv = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2HSV)
    red_count = int(np.count_nonzero(clean_mask(color_mask(hsv, RED_RGB, **tuning["colors"]["red"]))))
    green_count = int(np.count_nonzero(clean_mask(color_mask(hsv, GREEN_RGB, **tuning["colors"]["green"]))))
    magenta_count = int(np.count_nonzero(clean_mask(color_mask(hsv, MAGENTA_RGB, **tuning["colors"]["magenta"]))))
    required_pixels = max(25, round(crop_bgr.shape[0] * crop_bgr.shape[1] * 0.01))
    color, count = max(
        (("red", red_count), ("green", green_count), ("magenta", magenta_count)),
        key=lambda item: item[1],
    )
    if count < required_pixels:
        return "unknown"
    return color
