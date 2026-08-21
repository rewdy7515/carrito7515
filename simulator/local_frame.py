"""Marco de referencia local para las cuatro rectas de la pista."""

from __future__ import annotations

import math
from enum import Enum
from typing import Sequence

Point = tuple[float, float]


class LocalSide(str, Enum):
    LEFT = "LEFT"
    RIGHT = "RIGHT"
    CENTER = "CENTER"


def right_vector(forward_vector: Point) -> Point:
    """Vector unitario hacia RIGHT visto desde el avance local."""
    length = math.hypot(*forward_vector)
    if length <= 1e-12:
        raise ValueError("forward_vector no puede ser cero")
    fx, fy = forward_vector[0] / length, forward_vector[1] / length
    return -fy, fx


def left_vector(forward_vector: Point) -> Point:
    rx, ry = right_vector(forward_vector)
    return -rx, -ry


def get_local_side(
    point: Point,
    reference: Point,
    forward_vector: Point,
    tolerance: float = 1e-9,
) -> LocalSide:
    """Clasifica un punto respecto al avance local de una recta."""
    rx, ry = right_vector(forward_vector)
    dx, dy = point[0] - reference[0], point[1] - reference[1]
    lateral = dx * rx + dy * ry
    if lateral > tolerance:
        return LocalSide.RIGHT
    if lateral < -tolerance:
        return LocalSide.LEFT
    return LocalSide.CENTER


def footprint_side(
    vehicle_polygon: Sequence[Point],
    obstacle_polygon: Sequence[Point],
    forward_vector: Point,
    tolerance: float = 1e-9,
) -> LocalSide:
    """Clasifica un footprint que ya quedó completamente separado."""
    rx, ry = right_vector(forward_vector)
    vehicle_lateral = [point[0] * rx + point[1] * ry for point in vehicle_polygon]
    obstacle_lateral = [point[0] * rx + point[1] * ry for point in obstacle_polygon]
    if min(vehicle_lateral) > max(obstacle_lateral) + tolerance:
        return LocalSide.RIGHT
    if max(vehicle_lateral) < min(obstacle_lateral) - tolerance:
        return LocalSide.LEFT
    return LocalSide.CENTER


def crossing_side(
    vehicle_polygon: Sequence[Point],
    obstacle_polygon: Sequence[Point],
    forward_vector: Point,
) -> LocalSide:
    """Devuelve siempre LEFT o RIGHT usando el footprint completo.

    ``CENTER`` no es un resultado válido de un cruce. Si las proyecciones
    laterales todavía se solapan, se escoge el lado con mayor separación
    relativa entre los dos footprints. La decisión sigue usando todos los
    vértices, nunca únicamente el centro del vehículo.
    """
    rx, ry = right_vector(forward_vector)
    vehicle_lateral = [point[0] * rx + point[1] * ry for point in vehicle_polygon]
    obstacle_lateral = [point[0] * rx + point[1] * ry for point in obstacle_polygon]
    right_gap = min(vehicle_lateral) - max(obstacle_lateral)
    left_gap = min(obstacle_lateral) - max(vehicle_lateral)
    return LocalSide.RIGHT if right_gap >= left_gap else LocalSide.LEFT
