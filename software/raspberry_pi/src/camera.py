"""Captura y análisis visual inicial con OpenCV.

Este módulo detecta, en cada imagen:

* líneas azules y naranjas de la pista;
* regiones oscuras que pueden corresponder a muros negros;
* obstáculos rojos y verdes;
* delimitadores magenta del cajón de estacionamiento.
* marcas grises de arranque y referencias de la pista.

La franja verde es una zona de seguridad proyectada sobre la imagen. Su
distancia inicial es 340 mm, pero la relación entre píxeles y milímetros aún
debe calibrarse con la altura y el ángulo vertical reales de la cámara.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import threading
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional

import cv2
import numpy as np


TRACK_MAT_SIZE_MM = (3200, 3200)
TRACK_INTERIOR_SIZE_MM = (3000, 3000)
OUTER_AND_INNER_WALL_HEIGHT_MM = 100
TRACK_LINE_WIDTH_MM = 20
START_ZONE_SIZE_MM = (200, 500)
SIGN_SEAT_SIZE_MM = (50, 50)
OBSTACLE_DIMENSIONS_MM = (50, 50, 100)
MAX_OBSTACLES_PER_COLOR = 7
PARKING_DELIMITER_DIMENSIONS_MM = (200, 20, 100)
TUNING_PATH = Path(__file__).resolve().parents[3] / "config" / "vision_tuning.json"

DEFAULT_TUNING = {
    "safe_distance_mm": 200,
    "safe_zone_top_ratio": 0.68,
    "traffic_sign_minimum_area": 250,
    "traffic_sign_top_ratio": 0.32,
    "traffic_sign_bottom_ratio": 0.92,
    "black_wall_threshold": 65,
    "colors": {
        "red": {"hue_tolerance": 10, "minimum_saturation": 80, "minimum_value": 80},
        # El pilar puede verse verde oscuro y desaturado por la cámara y la
        # iluminación. La geometría de detect_traffic_signs evita aceptar
        # manchas pequeñas aunque esta máscara sea más tolerante.
        "green": {"hue_tolerance": 35, "minimum_saturation": 20, "minimum_value": 15},
        "blue": {"hue_tolerance": 18, "minimum_saturation": 60, "minimum_value": 50},
        "orange": {"hue_tolerance": 15, "minimum_saturation": 70, "minimum_value": 50},
        "magenta": {"hue_tolerance": 15, "minimum_saturation": 80, "minimum_value": 70},
    },
}


def _copy_tuning(value: dict) -> dict:
    return json.loads(json.dumps(value))


class TuningState:
    """Valores ajustables desde el panel web, persistidos en JSON."""

    def __init__(self, path: Path) -> None:
        self.path = path
        self.lock = threading.RLock()
        self.values = _copy_tuning(DEFAULT_TUNING)
        self._load()

    def _load(self) -> None:
        try:
            with self.path.open(encoding="utf-8") as file:
                loaded = json.load(file)
            self._merge(loaded)
        except (FileNotFoundError, json.JSONDecodeError, OSError, TypeError):
            return

    def _merge(self, loaded: dict) -> None:
        if not isinstance(loaded, dict):
            return
        for key in DEFAULT_TUNING:
            if key == "colors":
                continue
            if key in loaded and isinstance(loaded[key], (int, float)):
                self.values[key] = loaded[key]
        if isinstance(loaded.get("colors"), dict):
            for color, defaults in DEFAULT_TUNING["colors"].items():
                supplied = loaded["colors"].get(color)
                if isinstance(supplied, dict):
                    for key in defaults:
                        if isinstance(supplied.get(key), (int, float)):
                            self.values["colors"][color][key] = supplied[key]
        self.values = self._clamp(self.values)

    @staticmethod
    def _clamp(values: dict) -> dict:
        result = _copy_tuning(DEFAULT_TUNING)
        for key in ("safe_distance_mm", "traffic_sign_minimum_area", "black_wall_threshold"):
            result[key] = int(max(1, min(5000, values.get(key, result[key]))))
        for key in ("safe_zone_top_ratio", "traffic_sign_top_ratio", "traffic_sign_bottom_ratio"):
            result[key] = max(0.05, min(0.98, float(values.get(key, result[key]))))
        for color, defaults in result["colors"].items():
            source = values.get("colors", {}).get(color, {})
            defaults["hue_tolerance"] = int(max(1, min(60, source.get("hue_tolerance", defaults["hue_tolerance"]))))
            defaults["minimum_saturation"] = int(max(0, min(255, source.get("minimum_saturation", defaults["minimum_saturation"]))))
            defaults["minimum_value"] = int(max(0, min(255, source.get("minimum_value", defaults["minimum_value"]))))
        result["traffic_sign_bottom_ratio"] = max(result["traffic_sign_top_ratio"] + 0.01, result["traffic_sign_bottom_ratio"])
        return result

    def snapshot(self) -> dict:
        with self.lock:
            return _copy_tuning(self.values)

    def update(self, values: dict) -> dict:
        with self.lock:
            merged = self.snapshot()
            self._merge_into(merged, values)
            self.values = self._clamp(merged)
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self.path.write_text(json.dumps(self.values, indent=2) + "\n", encoding="utf-8")
            return _copy_tuning(self.values)

    @staticmethod
    def _merge_into(target: dict, source: dict) -> None:
        if not isinstance(source, dict):
            return
        for key in DEFAULT_TUNING:
            if key != "colors" and key in source:
                target[key] = source[key]
        if isinstance(source.get("colors"), dict):
            for color, values in source["colors"].items():
                if color in target["colors"] and isinstance(values, dict):
                    target["colors"][color].update(values)


@dataclass
class VisionConfig:
    camera_index: int | str = 0
    # Valor inicial conservador; debe calibrarse con velocidad y frenado reales.
    safe_distance_mm: int = 340
    # Valor provisional: representa la parte de la imagen que se considera
    # cercana al robot. No es una distancia física hasta calibrarlo.
    safe_zone_top_ratio: float = 0.68
    minimum_contour_area: int = 100
    traffic_sign_minimum_area: int = 250
    black_wall_minimum_area: int = 250
    vision_interval_frames: int = 3
    show_gray_reference_lines: bool = False
    traffic_sign_top_ratio: float = 0.32
    traffic_sign_bottom_ratio: float = 0.92


@dataclass
class Detection:
    kind: str
    color: str
    bounding_box: tuple[int, int, int, int]
    center: tuple[int, int]
    area: float


@dataclass(frozen=True)
class LineGeometry:
    """Geometría en píxeles de una línea detectada por la máscara de color."""

    x1: int
    y1: int
    x2: int
    y2: int
    midpoint_x: float
    midpoint_y: float
    length_px: float
    angle_deg: float


@dataclass
class VisionResult:
    blue_lines: list[tuple[int, int, int, int]]
    orange_lines: list[tuple[int, int, int, int]]
    blue_geometry: list[LineGeometry]
    orange_geometry: list[LineGeometry]
    gray_reference_lines: list[tuple[int, int, int, int]]
    black_walls: list[Detection]
    obstacles: list[Detection]
    parking_delimiters: list[Detection]
    safe_zone_polygon: np.ndarray
    wall_mask: np.ndarray
    line_mask: np.ndarray


# Colores de referencia. OpenCV trabaja internamente en BGR, pero las
# especificaciones de la pista se expresan en RGB/CMYK.
RED_RGB = (238, 39, 55)
GREEN_RGB = (68, 214, 44)
MAGENTA_RGB = (255, 0, 255)


def cmyk_to_rgb(cmyk: tuple[int, int, int, int]) -> tuple[int, int, int]:
    """Convierte CMYK porcentual a RGB aproximado para crear máscaras."""
    c, m, y, k = (component / 100 for component in cmyk)
    return tuple(round(255 * (1 - component) * (1 - k)) for component in (c, m, y))


ORANGE_CMYK = (0, 60, 100, 0)
BLUE_CMYK = (100, 80, 0, 0)
ORANGE_RGB = cmyk_to_rgb(ORANGE_CMYK)
BLUE_RGB = cmyk_to_rgb(BLUE_CMYK)
GRAY_REFERENCE_CMYK = (0, 0, 0, 30)


def rgb_to_hsv(rgb: tuple[int, int, int]) -> np.ndarray:
    one_pixel_bgr = np.uint8([[list(reversed(rgb))]])
    return cv2.cvtColor(one_pixel_bgr, cv2.COLOR_BGR2HSV)[0, 0]


def color_mask(
    frame_hsv: np.ndarray,
    rgb: tuple[int, int, int],
    hue_tolerance: int = 12,
    minimum_saturation: int = 80,
    minimum_value: int = 60,
) -> np.ndarray:
    """Crea una máscara HSV alrededor de un color RGB de referencia."""
    target_hue = int(rgb_to_hsv(rgb)[0])
    lower_saturation = minimum_saturation
    lower_value = minimum_value

    low_hue = target_hue - hue_tolerance
    high_hue = target_hue + hue_tolerance

    if low_hue < 0:
        first = cv2.inRange(
            frame_hsv,
            np.array([0, lower_saturation, lower_value]),
            np.array([high_hue, 255, 255]),
        )
        second = cv2.inRange(
            frame_hsv,
            np.array([180 + low_hue, lower_saturation, lower_value]),
            np.array([179, 255, 255]),
        )
        return cv2.bitwise_or(first, second)

    if high_hue > 179:
        first = cv2.inRange(
            frame_hsv,
            np.array([low_hue, lower_saturation, lower_value]),
            np.array([179, 255, 255]),
        )
        second = cv2.inRange(
            frame_hsv,
            np.array([0, lower_saturation, lower_value]),
            np.array([high_hue - 180, 255, 255]),
        )
        return cv2.bitwise_or(first, second)

    return cv2.inRange(
        frame_hsv,
        np.array([low_hue, lower_saturation, lower_value]),
        np.array([high_hue, 255, 255]),
    )


def clean_mask(mask: np.ndarray, kernel_size: int = 3) -> np.ndarray:
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)


def detect_lines(mask: np.ndarray) -> list[tuple[int, int, int, int]]:
    lines = cv2.HoughLinesP(
        mask,
        rho=1,
        theta=np.pi / 180,
        threshold=35,
        minLineLength=25,
        maxLineGap=12,
    )
    if lines is None:
        return []
    # OpenCV normalmente devuelve (N, 1, 4), pero algunas versiones o
    # backends pueden devolver (N, 4) o incluso una sola línea (4,).
    # Normalizar a una fila por línea evita tratar x1 como un iterable.
    values = np.asarray(lines)
    if values.size % 4 != 0:
        return []
    return [tuple(int(value) for value in line) for line in values.reshape(-1, 4)]


def extract_line_geometry(lines: list[tuple[int, int, int, int]]) -> list[LineGeometry]:
    """Calcula longitud, ángulo y punto medio de cada segmento detectado."""
    geometries = []
    for x1, y1, x2, y2 in lines:
        dx = x2 - x1
        dy = y2 - y1
        geometries.append(
            LineGeometry(
                x1=x1,
                y1=y1,
                x2=x2,
                y2=y2,
                midpoint_x=(x1 + x2) / 2.0,
                midpoint_y=(y1 + y2) / 2.0,
                length_px=float(math.hypot(dx, dy)),
                angle_deg=float(math.degrees(math.atan2(dy, dx))),
            )
        )
    return geometries


def render_mask_views(
    black_mask: np.ndarray,
    blue_mask: np.ndarray,
    orange_mask: np.ndarray,
    black_walls: list[Detection],
) -> tuple[np.ndarray, np.ndarray]:
    """Prepara la máscara binaria de transitabilidad y la máscara de líneas.

    En la vista del muro, negro significa ``no transitable`` y blanco
    significa ``transitable``. El límite se obtiene del borde inferior del
    muro detectado, no de cada píxel oscuro aislado.
    """
    # Sin un borde de muro confirmado no se declara ninguna zona como libre.
    # Cuando el muro sí aparece, solo el suelo debajo de su borde pasa a
    # blanco/transitable.
    traversability = np.zeros_like(black_mask)
    if black_walls:
        bottom = max(
            y + height
            for _, y, _, height in (wall.bounding_box for wall in black_walls)
        )
        traversability[min(bottom + 1, traversability.shape[0]) :, :] = 255
    wall_view = cv2.cvtColor(traversability, cv2.COLOR_GRAY2BGR)
    line_view = np.zeros((*black_mask.shape, 3), dtype=np.uint8)
    line_view[blue_mask > 0] = (255, 0, 0)
    line_view[orange_mask > 0] = (0, 140, 255)
    return wall_view, line_view


def detect_gray_reference_lines(frame_hsv: np.ndarray) -> list[tuple[int, int, int, int]]:
    """Detecta líneas grises de arranque y marcas auxiliares.

    Las líneas punteadas y los asientos tienen solo 1 mm de grosor, por lo que
    esta detección es una referencia inicial y necesitará ajuste de resolución
    y contraste con la cámara real.
    """
    gray_mask = cv2.inRange(
        frame_hsv,
        np.array([0, 0, 125]),
        np.array([179, 35, 215]),
    )
    return detect_lines(clean_mask(gray_mask))


def detect_regions(
    mask: np.ndarray,
    kind: str,
    color: str,
    minimum_area: int,
) -> list[Detection]:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    detections: list[Detection] = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < minimum_area:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        detections.append(
            Detection(
                kind=kind,
                color=color,
                bounding_box=(x, y, width, height),
                center=(x + width // 2, y + height // 2),
                area=area,
            )
        )

    return detections


def detect_black_wall(mask: np.ndarray, minimum_area: int) -> list[Detection]:
    """Detecta solo la franja horizontal del muro negro.

    El suelo que queda debajo del borde inferior del muro es transitable, por
    lo que no se devuelve como una colección de objetos negros. Se conserva
    únicamente el componente oscuro, ancho y horizontal que representa el
    muro.
    """
    frame_height, frame_width = mask.shape[:2]
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates: list[Detection] = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < minimum_area:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        if width < frame_width * 0.25 or height < 4:
            continue
        if y + height / 2 > frame_height * 0.88:
            continue
        if width / max(height, 1) < 1.5:
            continue
        candidates.append(
            Detection(
                kind="wall_mask",
                color="black",
                bounding_box=(x, y, width, height),
                center=(x + width // 2, y + height // 2),
                area=area,
            )
        )
    return [max(candidates, key=lambda item: item.area)] if candidates else []


def detect_traffic_signs(
    mask: np.ndarray,
    color: str,
    minimum_area: int,
    frame_height: int,
    top_ratio: float,
    bottom_ratio: float,
) -> list[Detection]:
    """Detecta pilares y descarta manchas pequeñas o regiones horizontales."""
    candidates = detect_regions(mask, "obstacle", color, minimum_area)
    signs: list[Detection] = []

    for detection in candidates:
        _, y, width, height = detection.bounding_box
        aspect_ratio = width / height if height else 0
        center_y = detection.center[1]

        # Las señales son verticales (50 x 50 x 100 mm). Este filtro elimina
        # ruido, reflejos y franjas verdosas del muro.
        if detection.area < max(minimum_area, 250):
            continue
        if width < 10 or height < 20:
            continue
        if center_y < frame_height * top_ratio or center_y > frame_height * bottom_ratio:
            continue
        if aspect_ratio > 1.0 or height < width * 1.20:
            continue

        signs.append(detection)

    return signs


def safe_zone_polygon(
    frame_shape: tuple[int, ...],
    tuning: dict,
    calibration=None,
    vehicle_width_cm: float = 14.6,
    vehicle_length_cm: float = 21.15,
) -> np.ndarray:
    height, width = frame_shape[:2]
    if calibration is not None:
        safe_distance_cm = max(
            vehicle_length_cm / 2 + 1.0,
            float(tuning["safe_distance_mm"]) / 10.0,
        )
        near_distance_cm = vehicle_length_cm / 2
        half_width_cm = vehicle_width_cm / 2

        def ground_to_pixel(x_cm: float, y_cm: float) -> tuple[int, int]:
            horizontal_ray = y_cm / max(x_cm, 1e-6)
            vertical_angle = math.atan2(
                calibration.height_cm, max(x_cm, 1e-6)
            ) - math.radians(calibration.pitch_deg)
            u = calibration.center_x_px + calibration.focal_x_px * horizontal_ray
            v = calibration.center_y_px + calibration.focal_y_px * math.tan(vertical_angle)
            return (
                round(u * width / calibration.width_px),
                round(v * height / calibration.height_px),
            )

        near_left = ground_to_pixel(near_distance_cm, -half_width_cm)
        near_right = ground_to_pixel(near_distance_cm, half_width_cm)
        far_right = ground_to_pixel(safe_distance_cm, half_width_cm)
        far_left = ground_to_pixel(safe_distance_cm, -half_width_cm)
        return np.array([near_left, near_right, far_right, far_left], dtype=np.int32)

    top_y = int(height * tuning["safe_zone_top_ratio"])
    left_top = int(width * 0.18)
    right_top = int(width * 0.82)
    return np.array(
        [[0, height - 1], [width - 1, height - 1], [right_top, top_y], [left_top, top_y]],
        dtype=np.int32,
    )


def analyze_frame(
    frame: np.ndarray,
    config: Optional[VisionConfig] = None,
    tuning: Optional[dict] = None,
) -> VisionResult:
    config = config or VisionConfig()
    tuning = tuning or _copy_tuning(DEFAULT_TUNING)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    blue_mask = clean_mask(
        color_mask(hsv, BLUE_RGB, **tuning["colors"]["blue"])
    )
    orange_mask = clean_mask(
        color_mask(hsv, ORANGE_RGB, **tuning["colors"]["orange"])
    )
    red_mask = clean_mask(color_mask(hsv, RED_RGB, **tuning["colors"]["red"]))
    green_mask = clean_mask(color_mask(hsv, GREEN_RGB, **tuning["colors"]["green"]))
    magenta_mask = clean_mask(
        color_mask(hsv, MAGENTA_RGB, **tuning["colors"]["magenta"])
    )

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    black_mask = cv2.inRange(gray, 0, tuning["black_wall_threshold"])
    black_mask = clean_mask(black_mask)

    blue_lines = detect_lines(blue_mask)
    orange_lines = detect_lines(orange_mask)
    black_walls = detect_black_wall(
        black_mask, config.black_wall_minimum_area
    )
    wall_mask_view, line_mask_view = render_mask_views(
        black_mask, blue_mask, orange_mask, black_walls
    )

    return VisionResult(
        blue_lines=blue_lines,
        orange_lines=orange_lines,
        blue_geometry=extract_line_geometry(blue_lines),
        orange_geometry=extract_line_geometry(orange_lines),
        gray_reference_lines=(
            detect_gray_reference_lines(hsv)
            if config.show_gray_reference_lines
            else []
        ),
        # El muro se mantiene separado de ``obstacles``: solo esta máscara
        # negra puede producir una detección de muro.
        black_walls=black_walls,
        obstacles=detect_traffic_signs(
            red_mask,
            "red",
            tuning["traffic_sign_minimum_area"],
            frame.shape[0],
            tuning["traffic_sign_top_ratio"],
            tuning["traffic_sign_bottom_ratio"],
        )
        + detect_traffic_signs(
            green_mask,
            "green",
            tuning["traffic_sign_minimum_area"],
            frame.shape[0],
            tuning["traffic_sign_top_ratio"],
            tuning["traffic_sign_bottom_ratio"],
        ),
        parking_delimiters=detect_regions(
            magenta_mask, "parking_delimiter", "magenta", config.minimum_contour_area
        ),
        safe_zone_polygon=safe_zone_polygon(frame.shape, tuning),
        wall_mask=wall_mask_view,
        line_mask=line_mask_view,
    )


def draw_result(
    frame: np.ndarray,
    result: VisionResult,
    config: Optional[VisionConfig] = None,
    tuning: Optional[dict] = None,
    calibration=None,
) -> np.ndarray:
    config = config or VisionConfig()
    tuning = tuning or _copy_tuning(DEFAULT_TUNING)
    output = frame.copy()

    safe_polygon = safe_zone_polygon(output.shape, tuning, calibration)
    # Franja verde semitransparente de distancia segura.
    overlay = output.copy()
    cv2.fillPoly(overlay, [safe_polygon], (0, 180, 0))
    output = cv2.addWeighted(overlay, 0.20, output, 0.80, 0)
    cv2.polylines(output, [safe_polygon], True, (0, 255, 0), 2)
    cv2.putText(
        output,
        f"ZONA SEGURA: ancho carro 146 mm | hasta {tuning['safe_distance_mm']} mm",
        (15, output.shape[0] - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 180, 0),
        2,
        cv2.LINE_AA,
    )

    for line in result.blue_lines:
        cv2.line(output, line[:2], line[2:], (255, 0, 0), 3)
    for line in result.orange_lines:
        cv2.line(output, line[:2], line[2:], (0, 140, 255), 3)
    for geometry in result.blue_geometry:
        cv2.putText(
            output,
            f"B L={geometry.length_px:.0f}px A={geometry.angle_deg:+.0f}°",
            (round(geometry.midpoint_x), round(geometry.midpoint_y)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.32,
            (255, 180, 80),
            1,
            cv2.LINE_AA,
        )
    for geometry in result.orange_geometry:
        cv2.putText(
            output,
            f"N L={geometry.length_px:.0f}px A={geometry.angle_deg:+.0f}°",
            (round(geometry.midpoint_x), round(geometry.midpoint_y)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.32,
            (0, 190, 255),
            1,
            cv2.LINE_AA,
        )
    for line in result.gray_reference_lines:
        cv2.line(output, line[:2], line[2:], (170, 170, 170), 1)

    colors = {
        "black": (80, 80, 80),
        "red": (0, 0, 255),
        "green": (0, 255, 0),
        "magenta": (255, 0, 255),
    }
    for detection in result.black_walls + result.obstacles + result.parking_delimiters:
        x, y, width, height = detection.bounding_box
        color = colors[detection.color]
        cv2.rectangle(output, (x, y), (x + width, y + height), color, 2)
        if detection.kind == "wall_mask":
            # El límite inferior separa la zona no transitable de la zona de
            # suelo que sí puede usar el robot.
            cv2.line(
                output,
                (x, y + height),
                (x + width, y + height),
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
        cv2.putText(
            output,
            "MURO NEGRO | debajo transitable"
            if detection.kind == "wall_mask"
            else detection.kind + ":" + detection.color,
            (x, max(y - 6, 15)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            1,
            cv2.LINE_AA,
        )

    return output


class FrameStore:
    """Guarda las cuatro vistas de cámara disponibles en el navegador."""

    def __init__(self) -> None:
        self.condition = threading.Condition()
        self.raw_jpeg: Optional[bytes] = None
        self.vision_jpeg: Optional[bytes] = None
        self.wall_mask_jpeg: Optional[bytes] = None
        self.line_mask_jpeg: Optional[bytes] = None
        self.version = 0

    def update(
        self,
        raw_frame: np.ndarray,
        vision_frame: np.ndarray,
        wall_mask_frame: np.ndarray,
        line_mask_frame: np.ndarray,
    ) -> None:
        encode_parameters = [cv2.IMWRITE_JPEG_QUALITY, 60]
        raw_success, raw_encoded = cv2.imencode(".jpg", raw_frame, encode_parameters)
        vision_success, vision_encoded = cv2.imencode(
            ".jpg", vision_frame, encode_parameters
        )
        wall_success, wall_encoded = cv2.imencode(
            ".jpg", wall_mask_frame, encode_parameters
        )
        line_success, line_encoded = cv2.imencode(
            ".jpg", line_mask_frame, encode_parameters
        )
        if not all((raw_success, vision_success, wall_success, line_success)):
            return
        with self.condition:
            self.raw_jpeg = raw_encoded.tobytes()
            self.vision_jpeg = vision_encoded.tobytes()
            self.wall_mask_jpeg = wall_encoded.tobytes()
            self.line_mask_jpeg = line_encoded.tobytes()
            self.version += 1
            self.condition.notify_all()

    def wait_for_new(self, last_version: int, stream_name: str) -> tuple[int, bytes]:
        with self.condition:
            self.condition.wait_for(lambda: self.version > last_version)
            jpeg = {
                "raw": self.raw_jpeg,
                "vision": self.vision_jpeg,
                "wall_mask": self.wall_mask_jpeg,
                "line_mask": self.line_mask_jpeg,
            }[stream_name]
            return self.version, jpeg  # type: ignore[return-value]


class LatestCameraFrame:
    """Captura en segundo plano y conserva únicamente el frame más reciente."""

    def __init__(self, source: int | str, width: int, height: int) -> None:
        self.capture = cv2.VideoCapture(source)
        if not self.capture.isOpened():
            raise RuntimeError(f"No se pudo abrir la cámara {source}")

        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.width = width
        self.height = height
        self.lock = threading.Lock()
        self.latest: Optional[np.ndarray] = None
        self.running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

    def _capture_loop(self) -> None:
        while self.running:
            with self.lock:
                if not self.running:
                    break
                success, frame = self.capture.read()
            if not success:
                time.sleep(0.01)
                continue
            if frame.shape[1] != self.width or frame.shape[0] != self.height:
                frame = cv2.resize(
                    frame,
                    (self.width, self.height),
                    interpolation=cv2.INTER_AREA,
                )
            with self.lock:
                self.latest = frame

    def read_latest(self) -> Optional[np.ndarray]:
        with self.lock:
            return None if self.latest is None else self.latest.copy()

    def close(self) -> None:
        self.running = False
        self.thread.join(timeout=2.0)
        with self.lock:
            self.capture.release()


class CameraStreamHandler(BaseHTTPRequestHandler):
    store: FrameStore
    tuning_state: TuningState

    def _send_body(self, body: bytes, content_type: str) -> None:
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_tuning_page(self) -> None:
        body = """
        <html><head><title>Visión del robot</title>
        <meta name="viewport" content="width=device-width, initial-scale=1">
        <style>
        body{margin:0;background:#202124;color:#eee;font-family:Arial;text-align:center}
        h1{font-size:28px;margin:16px 16px 4px}.subtitle{margin:0 0 16px;color:#9aa0a6}
        .views{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:20px;width:96vw;max-width:1920px;margin:auto;align-items:start}
        .view{width:100%;height:min(42vh,520px);min-height:260px;margin:0 auto 20px;resize:both;overflow:hidden;background:#111;border:2px solid #555;box-sizing:border-box}
        .view img{display:block;width:100%;height:calc(100% - 52px);object-fit:contain}.view h2{font-size:22px;height:36px;line-height:36px;margin:0}
        .panel{width:96vw;max-width:1200px;margin:10px auto 30px;padding:16px;background:#303134;border-radius:8px;text-align:left;box-sizing:border-box}
        .grid{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}.control{display:flex;flex-direction:column;gap:4px}.control span{font-size:13px;color:#bbb}.control output{color:#8ab4f8;font-weight:bold}
        input{width:100%}button{margin-top:14px;padding:9px 16px;background:#8ab4f8;border:0;border-radius:4px;cursor:pointer}#status{margin-left:12px;color:#9be49b}
        @media(max-width:900px){.views{grid-template-columns:1fr}.view{width:96vw}.grid{grid-template-columns:1fr 1fr}}
        </style></head><body><h1>Visión del robot</h1><p class="subtitle">Cámara en vivo y detección durante la prueba autónoma</p>
        <main class="views"><section class="view"><h2>1. Cámara original</h2><img src="/stream/raw"></section>
        <section class="view"><h2>2. Visión procesada</h2><img src="/stream/vision"></section>
        <section class="view"><h2>3. Mask de muro</h2><img src="/stream/wall-mask"></section>
        <section class="view"><h2>4. Mask de líneas</h2><img src="/stream/line-mask"></section></main>
        <section class="panel"><h2>Ajuste manual de visión</h2>
        <div class="grid">
        <label class="control">Distancia segura (mm)<output id="safe_distance_mm_value"></output><input id="safe_distance_mm" type="range" min="100" max="1000" step="10"></label>
        <label class="control">Área mínima de señal<output id="traffic_sign_minimum_area_value"></output><input id="traffic_sign_minimum_area" type="range" min="50" max="3000" step="10"></label>
        <label class="control">Umbral muro negro<output id="black_wall_threshold_value"></output><input id="black_wall_threshold" type="range" min="10" max="150"></label>
        <label class="control">Rojo: tolerancia tono<output id="red_hue_tolerance_value"></output><input id="red_hue_tolerance" type="range" min="1" max="60"></label>
        <label class="control">Rojo: saturación mínima<output id="red_minimum_saturation_value"></output><input id="red_minimum_saturation" type="range" min="0" max="255"></label>
        <label class="control">Rojo: brillo mínimo<output id="red_minimum_value_value"></output><input id="red_minimum_value" type="range" min="0" max="255"></label>
        <label class="control">Verde: tolerancia tono<output id="green_hue_tolerance_value"></output><input id="green_hue_tolerance" type="range" min="1" max="60"></label>
        <label class="control">Verde: saturación mínima<output id="green_minimum_saturation_value"></output><input id="green_minimum_saturation" type="range" min="0" max="255"></label>
        <label class="control">Verde: brillo mínimo<output id="green_minimum_value_value"></output><input id="green_minimum_value" type="range" min="0" max="255"></label>
        <label class="control">Azul: tolerancia tono<output id="blue_hue_tolerance_value"></output><input id="blue_hue_tolerance" type="range" min="1" max="60"></label>
        <label class="control">Azul: saturación mínima<output id="blue_minimum_saturation_value"></output><input id="blue_minimum_saturation" type="range" min="0" max="255"></label>
        <label class="control">Azul: brillo mínimo<output id="blue_minimum_value_value"></output><input id="blue_minimum_value" type="range" min="0" max="255"></label>
        <label class="control">Naranja: tolerancia tono<output id="orange_hue_tolerance_value"></output><input id="orange_hue_tolerance" type="range" min="1" max="60"></label>
        <label class="control">Naranja: saturación mínima<output id="orange_minimum_saturation_value"></output><input id="orange_minimum_saturation" type="range" min="0" max="255"></label>
        <label class="control">Naranja: brillo mínimo<output id="orange_minimum_value_value"></output><input id="orange_minimum_value" type="range" min="0" max="255"></label>
        <label class="control">Magenta: tolerancia tono<output id="magenta_hue_tolerance_value"></output><input id="magenta_hue_tolerance" type="range" min="1" max="60"></label>
        <label class="control">Magenta: saturación mínima<output id="magenta_minimum_saturation_value"></output><input id="magenta_minimum_saturation" type="range" min="0" max="255"></label>
        <label class="control">Magenta: brillo mínimo<output id="magenta_minimum_value_value"></output><input id="magenta_minimum_value" type="range" min="0" max="255"></label>
        </div><button id="save">Guardar ajustes</button><span id="status"></span>
        </section><script>
        const ids=['safe_distance_mm','traffic_sign_minimum_area','black_wall_threshold','red_hue_tolerance','red_minimum_saturation','red_minimum_value','green_hue_tolerance','green_minimum_saturation','green_minimum_value','blue_hue_tolerance','blue_minimum_saturation','blue_minimum_value','orange_hue_tolerance','orange_minimum_saturation','orange_minimum_value','magenta_hue_tolerance','magenta_minimum_saturation','magenta_minimum_value'];
        const $=id=>document.getElementById(id); const show=id=>$(id+'_value').textContent=$(id).value;
        let timer; let saving=false;
        ids.forEach(id=>$(id).addEventListener('input',()=>{show(id);clearTimeout(timer);timer=setTimeout(applyChanges,180)}));
        function load(v){ids.forEach(id=>{let color=['red','green','blue','orange','magenta'].find(c=>id.startsWith(c+'_')),key=color?id.slice(color.length+1):id,n=color?v.colors[color][key]:v[key];$(id).value=n;show(id)})}
        fetch('/api/tuning').then(r=>r.json()).then(load);
        function applyChanges(){if(saving)return;saving=true;let v={safe_distance_mm:+$('safe_distance_mm').value,traffic_sign_minimum_area:+$('traffic_sign_minimum_area').value,black_wall_threshold:+$('black_wall_threshold').value,colors:{}};['red','green','blue','orange','magenta'].forEach(c=>{v.colors[c]={};['hue_tolerance','minimum_saturation','minimum_value'].forEach(k=>v.colors[c][k]=+$(c+'_'+k).value)});$('status').textContent='Aplicando...';fetch('/api/tuning',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(v)}).then(r=>r.json()).then(()=>{$('status').textContent='Aplicado en vivo';setTimeout(()=>$('status').textContent='',1000)}).catch(()=>$('status').textContent='Error al aplicar').finally(()=>saving=false)}
        $('save').onclick=applyChanges;
        </script></body></html>
        """.encode("utf-8")
        self._send_body(body, "text/html; charset=utf-8")

    def do_GET(self) -> None:  # noqa: N802 - nombre requerido por BaseHTTPRequestHandler
        if self.path == "/":
            self._send_tuning_page()
            return

        if self.path == "/api/tuning":
            self._send_body(
                json.dumps(self.tuning_state.snapshot()).encode("utf-8"),
                "application/json; charset=utf-8",
            )
            return

        stream_name = {
            "/stream/raw": "raw",
            "/stream/vision": "vision",
            "/stream/wall-mask": "wall_mask",
            "/stream/line-mask": "line_mask",
        }.get(self.path)
        if stream_name is None:
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()

        last_version = 0
        try:
            while True:
                last_version, jpeg = self.store.wait_for_new(last_version, stream_name)

                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_POST(self) -> None:  # noqa: N802
        if self.path != "/api/tuning":
            self.send_error(404)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            values = json.loads(self.rfile.read(length))
            saved = self.tuning_state.update(values)
        except (ValueError, TypeError, json.JSONDecodeError, OSError):
            self.send_error(400, "JSON de ajustes inválido")
            return
        self._send_body(json.dumps(saved).encode("utf-8"), "application/json; charset=utf-8")

    def log_message(self, format: str, *args: object) -> None:
        return


def start_stream_server(
    host: str, port: int, store: FrameStore, tuning_state: TuningState
) -> ThreadingHTTPServer:
    handler = type(
        "BoundCameraStreamHandler",
        (CameraStreamHandler,),
        {"store": store, "tuning_state": tuning_state},
    )
    server = ThreadingHTTPServer((host, port), handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--camera",
        default="0",
        help="Índice de cámara (0, 1, ...) o dispositivo (/dev/video0)",
    )
    parser.add_argument("--safe-distance-mm", type=int, default=340)
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument("--stream-host", default="0.0.0.0")
    parser.add_argument("--stream-port", type=int, default=8000)
    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--height", type=int, default=240)
    parser.add_argument(
        "--vision-every",
        type=int,
        default=3,
        help="Procesa detecciones cada N frames; usa 1 para cada frame",
    )
    parser.add_argument(
        "--no-vision",
        action="store_true",
        help="Transmite la cámara sin ejecutar las detecciones de OpenCV",
    )
    parser.add_argument(
        "--show-reference-marks",
        action="store_true",
        help="Dibuja las marcas grises auxiliares de la pista",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    camera_source: int | str
    camera_source = int(args.camera) if args.camera.isdigit() else args.camera
    config = VisionConfig(
        camera_index=camera_source,
        safe_distance_mm=args.safe_distance_mm,
        vision_interval_frames=max(1, args.vision_every),
        show_gray_reference_lines=args.show_reference_marks,
    )
    try:
        capture = LatestCameraFrame(config.camera_index, args.width, args.height)
    except RuntimeError as error:
        raise SystemExit(str(error)) from error

    display_available = bool(os.environ.get("DISPLAY")) and not args.no_display
    frame_store = FrameStore()
    tuning_state = TuningState(TUNING_PATH)
    stream_server = start_stream_server(
        args.stream_host, args.stream_port, frame_store, tuning_state
    )
    print(f"Video disponible en: http://pirobot.local:{stream_server.server_port}")
    frame_number = 0
    last_result: Optional[VisionResult] = None

    try:
        while True:
            frame = capture.read_latest()
            if frame is None:
                time.sleep(0.01)
                continue

            if args.no_vision:
                annotated = frame
                wall_mask_view = np.zeros_like(frame)
                line_mask_view = np.zeros_like(frame)
            else:
                frame_number += 1
                if last_result is None or frame_number % config.vision_interval_frames == 0:
                    last_result = analyze_frame(frame, config, tuning_state.snapshot())
                annotated = draw_result(frame, last_result, config, tuning_state.snapshot())
                wall_mask_view = last_result.wall_mask
                line_mask_view = last_result.line_mask
            frame_store.update(frame, annotated, wall_mask_view, line_mask_view)

            if display_available:
                cv2.imshow("Vision del robot", annotated)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            else:
                # En una sesión SSH la salida se visualiza desde el navegador.
                # Ctrl+C termina este proceso.
                time.sleep(0.01)
    finally:
        capture.close()
        stream_server.shutdown()
        if display_available:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
