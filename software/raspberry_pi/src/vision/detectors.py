"""Detectores OpenCV clásicos no relacionados con objetos YOLO.

La posición de ``obstacle`` y ``parking_wall`` no se calcula aquí: la fuente
de esas detecciones será exclusivamente el modelo YOLO/NCNN.
"""

from __future__ import annotations

import math
import cv2
import numpy as np

from .colors import clean_mask
from .types import Detection, LineGeometry


def detect_lines(mask: np.ndarray) -> list[tuple[int, int, int, int]]:
    lines = cv2.HoughLinesP(mask, rho=1, theta=np.pi / 180, threshold=35, minLineLength=25, maxLineGap=12)
    if lines is None or np.asarray(lines).size % 4:
        return []
    return [tuple(int(value) for value in line) for line in np.asarray(lines).reshape(-1, 4)]


def extract_line_geometry(lines: list[tuple[int, int, int, int]]) -> list[LineGeometry]:
    return [LineGeometry(x1, y1, x2, y2, (x1 + x2) / 2, (y1 + y2) / 2, float(math.hypot(x2 - x1, y2 - y1)), float(math.degrees(math.atan2(y2 - y1, x2 - x1)))) for x1, y1, x2, y2 in lines]


def detect_gray_reference_lines(frame_hsv: np.ndarray) -> list[tuple[int, int, int, int]]:
    return detect_lines(clean_mask(cv2.inRange(frame_hsv, np.array([0, 0, 125]), np.array([179, 35, 215]))))


def detect_black_track_wall(mask: np.ndarray, minimum_area: int) -> list[Detection]:
    """Localiza la franja horizontal negra que delimita la pista.

    No clasifica obstáculos: esta geometría se mantiene independiente de
    YOLO porque ``track_wall`` no es una de sus dos clases entrenadas.
    """
    frame_height, frame_width = mask.shape[:2]
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < minimum_area:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        if width < frame_width * 0.25 or height < 4:
            continue
        if y + height / 2 > frame_height * 0.88 or width / max(height, 1) < 1.5:
            continue
        candidates.append(Detection("track_wall", "black", (x, y, width, height), (x + width // 2, y + height // 2), area))
    if candidates:
        return [max(candidates, key=lambda item: item.area)]

    # Fallback para una pared parcialmente tapada: se busca una franja que
    # siga siendo oscura en una proporción amplia de la imagen. Mantiene el
    # comportamiento conservador si no existe ninguna fila candidata.
    dark_coverage = np.count_nonzero(mask, axis=1) / frame_width
    eligible_rows = np.flatnonzero(
        (dark_coverage >= 0.18)
        & (np.arange(frame_height) >= frame_height * 0.15)
        & (np.arange(frame_height) <= frame_height * 0.88)
    )
    if eligible_rows.size == 0:
        return []
    bottom = int(eligible_rows[-1])
    top = int(eligible_rows[0])
    return [Detection(
        "track_wall", "black", (0, top, frame_width, bottom - top + 1),
        (frame_width // 2, (top + bottom) // 2),
        float(frame_width * (bottom - top + 1)),
    )]
