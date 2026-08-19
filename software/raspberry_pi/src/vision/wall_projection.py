"""Puntos de la unión muro/suelo para el planificador geométrico."""

from __future__ import annotations

import numpy as np

from .ground_projection import GroundProjection
from .types import WallGroundPoint


def project_wall_boundary(wall_mask: np.ndarray, projection: GroundProjection, point_count: int) -> list[WallGroundPoint]:
    """Muestrea el cambio negro→blanco y lo convierte a coordenadas del suelo.

    ``wall_mask`` es binaria: el primer píxel blanco de cada columna pertenece
    al suelo inmediatamente bajo el muro, por eso representa la unión útil
    para distancia y esquive.
    """
    height, width = wall_mask.shape[:2]
    boundary_columns = []
    for x in range(width):
        white_rows = np.flatnonzero(wall_mask[:, x] == 255)
        if white_rows.size and white_rows[0] > 0:
            boundary_columns.append((x, int(white_rows[0])))
    if not boundary_columns:
        return []

    count = min(max(1, point_count), len(boundary_columns))
    sample_indexes = np.unique(np.rint(np.linspace(0, len(boundary_columns) - 1, count)).astype(int))
    points = []
    for number, sample_index in enumerate(sample_indexes, 1):
        pixel = boundary_columns[sample_index]
        ground = projection.project_pixel(pixel, wall_mask.shape)
        if ground is None:
            continue
        x_cm, y_cm = ground
        points.append(WallGroundPoint(number, pixel, x_cm, y_cm))
    return points
