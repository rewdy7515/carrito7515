"""Orquestación de las detecciones clásicas por frame."""

from __future__ import annotations

import cv2
import numpy as np

from .colors import BLUE_RGB, ORANGE_RGB, clean_mask, color_mask
from .config import DEFAULT_TUNING, copy_tuning
from .detectors import detect_black_track_wall, detect_gray_reference_lines, detect_lines, extract_line_geometry
from .geometry import safe_zone_polygon
from .ground_projection import GroundProjection
from .rendering import render_mask_views
from .types import Detection, VisionConfig, VisionResult
from .wall_projection import project_wall_boundary


def analyze_frame(frame: np.ndarray, config: VisionConfig | None = None, tuning: dict | None = None, yolo_detections: tuple[list[Detection], list[Detection]] | None = None, ground_projection: GroundProjection | None = None) -> VisionResult:
    config, tuning = config or VisionConfig(), tuning or copy_tuning(DEFAULT_TUNING)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    if config.detect_track_lines:
        masks = {
            color: clean_mask(color_mask(hsv, rgb, **tuning["colors"][color]))
            for color, rgb in {"blue": BLUE_RGB, "orange": ORANGE_RGB}.items()
        }
        blue_lines, orange_lines = detect_lines(masks["blue"]), detect_lines(masks["orange"])
    else:
        # No ejecutar HSV/Hough para azul o naranja durante la validación YOLO.
        masks = {"blue": np.zeros(frame.shape[:2], dtype=np.uint8), "orange": np.zeros(frame.shape[:2], dtype=np.uint8)}
        blue_lines, orange_lines = [], []
    black_mask = clean_mask(cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), 0, tuning["black_wall_threshold"]))
    obstacles, parking_walls = yolo_detections or ([], [])
    if ground_projection is not None:
        obstacles = [ground_projection.project(item, frame.shape) for item in obstacles]
    # Los pilares pueden ser tan oscuros que se fusionan con la pared en la
    # máscara monocromática. YOLO ya confirmó que no son pared de pista, así
    # que se excluyen antes de buscar el borde horizontal transitable.
    for item in obstacles + parking_walls:
        x, y, width, height = item.bounding_box
        cv2.rectangle(black_mask, (x, y), (x + width, y + height), 0, thickness=-1)
    track_walls = detect_black_track_wall(black_mask, config.black_wall_minimum_area)
    # La máscara de transitabilidad se deriva únicamente del muro negro de la
    # pista. Un parking wall magenta no convierte el suelo de la pista en muro.
    wall_mask, line_mask = render_mask_views(black_mask, masks["blue"], masks["orange"], track_walls)
    wall_ground_points = (
        project_wall_boundary(wall_mask, ground_projection, config.wall_projection_points)
        if ground_projection is not None
        else []
    )
    return VisionResult(blue_lines, orange_lines, extract_line_geometry(blue_lines), extract_line_geometry(orange_lines), detect_gray_reference_lines(hsv) if config.show_gray_reference_lines else [], obstacles, parking_walls, track_walls, wall_ground_points, [], safe_zone_polygon(frame.shape, tuning), wall_mask, line_mask)
