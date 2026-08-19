"""Renderizado de resultados y máscaras para depuración visual."""

from __future__ import annotations

import cv2
import numpy as np

from .config import DEFAULT_TUNING, copy_tuning
from .geometry import safe_zone_polygon
from .types import Detection, VisionConfig, VisionResult


def render_mask_views(black_mask: np.ndarray, blue_mask: np.ndarray, orange_mask: np.ndarray, track_walls: list[Detection]) -> tuple[np.ndarray, np.ndarray]:
    """Construye máscaras de depuración.

    ``wall_view`` es estrictamente binaria: 0 (negro) para el muro y todo lo
    que queda arriba de él; 255 (blanco) para el suelo bajo el borde inferior.
    Sin muro confirmado se mantiene toda negra, de forma conservadora.
    """
    traversability = np.zeros_like(black_mask)
    if track_walls:
        # Se sigue el borde inferior del muro por columna, no el máximo global
        # de su rectángulo. Así la separación blanco/negro conserva la forma
        # inclinada que realmente ve la cámara.
        wall = max(track_walls, key=lambda item: item.area)
        boundary = _track_wall_lower_boundary(black_mask, wall)
        for x, bottom in enumerate(boundary):
            traversability[min(bottom + 1, traversability.shape[0]):, x] = 255
    wall_view = traversability
    line_view = np.zeros((*black_mask.shape, 3), dtype=np.uint8)
    line_view[blue_mask > 0], line_view[orange_mask > 0] = (255, 0, 0), (0, 140, 255)
    return wall_view, line_view


def _track_wall_lower_boundary(black_mask: np.ndarray, wall: Detection) -> np.ndarray:
    """Devuelve el borde inferior del muro para cada columna de la imagen."""
    frame_height, frame_width = black_mask.shape
    x, y, width, height = wall.bounding_box
    x_start, x_end = max(0, x), min(frame_width, x + width)
    y_start, y_end = max(0, y), min(frame_height, y + height)
    boundary = np.full(frame_width, np.nan, dtype=np.float32)

    for column in range(x_start, x_end):
        dark_rows = np.flatnonzero(black_mask[y_start:y_end, column] > 0)
        if dark_rows.size:
            boundary[column] = y_start + dark_rows[-1]

    valid_columns = np.flatnonzero(~np.isnan(boundary))
    if valid_columns.size == 0:
        # La detección ya fue confirmada; usar su borde inferior solo como
        # fallback ante una máscara vacía inesperada.
        return np.full(frame_width, min(y_end - 1, frame_height - 1), dtype=np.int32)

    # Interpola huecos pequeños y prolonga los extremos para que toda la
    # imagen mantenga la misma frontera de transitabilidad.
    columns = np.arange(frame_width)
    return np.rint(np.interp(columns, valid_columns, boundary[valid_columns])).astype(np.int32)


def draw_result(frame: np.ndarray, result: VisionResult, config: VisionConfig | None = None, tuning: dict | None = None, calibration=None) -> np.ndarray:
    del config
    tuning, output = tuning or copy_tuning(DEFAULT_TUNING), frame.copy()
    polygon = safe_zone_polygon(output.shape, tuning, calibration)
    overlay = output.copy(); cv2.fillPoly(overlay, [polygon], (0, 180, 0)); output = cv2.addWeighted(overlay, .20, output, .80, 0)
    cv2.polylines(output, [polygon], True, (0, 255, 0), 2)
    cv2.putText(output, f"ZONA SEGURA: ancho carro 146 mm | hasta {tuning['safe_distance_mm']} mm", (15, output.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 180, 0), 2, cv2.LINE_AA)
    for lines, color, label in ((result.blue_lines, (255, 0, 0), "B"), (result.orange_lines, (0, 140, 255), "N")):
        for line in lines: cv2.line(output, line[:2], line[2:], color, 3)
        for item in (result.blue_geometry if label == "B" else result.orange_geometry): cv2.putText(output, f"{label} L={item.length_px:.0f}px A={item.angle_deg:+.0f}°", (round(item.midpoint_x), round(item.midpoint_y)), cv2.FONT_HERSHEY_SIMPLEX, .32, color, 1, cv2.LINE_AA)
    for line in result.gray_reference_lines: cv2.line(output, line[:2], line[2:], (170, 170, 170), 1)
    colors = {"black": (80, 80, 80), "red": (0, 0, 255), "green": (0, 255, 0), "magenta": (255, 0, 255), "unknown": (0, 255, 255)}
    # El muro negro se comunica mediante ``wall_mask``; no se dibuja un
    # recuadro sobre la imagen procesada para no confundirlo con un objeto.
    for item in result.parking_walls + result.obstacles + result.parking_delimiters:
        x, y, width, height = item.bounding_box; color = colors[item.color]
        cv2.rectangle(output, (x, y), (x + width, y + height), color, 2)
        label = f"{item.kind}:{item.color}"
        if item.confidence is not None: label += f" {item.confidence:.0%}"
        if item.bottom_center is not None:
            cv2.circle(output, item.bottom_center, 3, (0, 255, 255), -1, cv2.LINE_AA)
        if item.distance_cm is not None:
            prefix = "D~" if item.distance_is_estimated else "D"
            label += f" | {prefix}{item.distance_cm:.1f}cm"
        cv2.putText(output, label, (x, max(y - 6, 15)), cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1, cv2.LINE_AA)
    for point in result.wall_ground_points:
        cv2.circle(output, point.pixel, 4, (255, 255, 0), -1, cv2.LINE_AA)
        cv2.putText(output, f"P{point.index}", (point.pixel[0] + 4, max(12, point.pixel[1] - 4)), cv2.FONT_HERSHEY_SIMPLEX, .35, (255, 255, 0), 1, cv2.LINE_AA)
    return output
