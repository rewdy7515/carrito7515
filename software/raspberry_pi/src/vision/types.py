"""Tipos de datos intercambiados entre los componentes de visión."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class VisionConfig:
    camera_index: int | str = 0
    safe_distance_mm: int = 340
    safe_zone_top_ratio: float = 0.68
    minimum_contour_area: int = 100
    vision_interval_frames: int = 3
    show_gray_reference_lines: bool = False
    # Temporalmente deshabilitado: la prueba actual se enfoca en YOLO/NCNN.
    detect_track_lines: bool = False
    black_wall_minimum_area: int = 250
    wall_projection_points: int = 9


@dataclass
class Detection:
    kind: str
    color: str
    bounding_box: tuple[int, int, int, int]
    center: tuple[int, int]
    area: float
    confidence: float | None = None
    bottom_center: tuple[int, int] | None = None
    forward_cm: float | None = None
    lateral_cm: float | None = None
    distance_cm: float | None = None
    distance_is_estimated: bool = True


@dataclass(frozen=True)
class LineGeometry:
    x1: int
    y1: int
    x2: int
    y2: int
    midpoint_x: float
    midpoint_y: float
    length_px: float
    angle_deg: float


@dataclass(frozen=True)
class WallGroundPoint:
    """Un punto del borde muro/suelo proyectado al plano del carro."""

    index: int
    pixel: tuple[int, int]
    x_cm: float
    y_cm: float


@dataclass
class VisionResult:
    blue_lines: list[tuple[int, int, int, int]]
    orange_lines: list[tuple[int, int, int, int]]
    blue_geometry: list[LineGeometry]
    orange_geometry: list[LineGeometry]
    gray_reference_lines: list[tuple[int, int, int, int]]
    obstacles: list[Detection]
    parking_walls: list[Detection]
    track_walls: list[Detection]
    wall_ground_points: list[WallGroundPoint]
    parking_delimiters: list[Detection]
    safe_zone_polygon: np.ndarray
    wall_mask: np.ndarray
    line_mask: np.ndarray
