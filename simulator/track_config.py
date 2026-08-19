"""Fuente unica de verdad de la geometria de la pista del simulador.

Este modulo no importa Pygame ni el planner. El simulador grafico y el runner
headless deben construir sus entradas a partir de estos mismos datos.
Todas las distancias estan en centimetros.
"""

from __future__ import annotations

import math


Point = tuple[float, float]

TRACK_CM = 300.0
OUTER_WALL = (0.0, 0.0, TRACK_CM, TRACK_CM)
INNER_WALL = (100.0, 100.0, 100.0, 100.0)

# El reto que se esta validando es el corredor de obstaculos de 1000 mm.
# En el modelo de 300 cm x 300 cm esto equivale a 100 cm entre los muros.
CORRIDOR_WIDTH_CM = 100.0

# Zona de salida solicitada por el equipo: 500 mm x 400 mm.
# Se conserva en la recta inferior izquierda y toca el borde inferior del
# campo. Su centro es la pose inicial comun de ambos simuladores.
START_ZONE = (100.0, 260.0, 50.0, 40.0)
START_POSE = (125.0, 280.0, math.pi)

SIGN_SEAT_CENTERS: tuple[Point, ...] = (
    (100.0, 40.0), (150.0, 40.0), (200.0, 40.0),
    (100.0, 60.0), (150.0, 60.0), (200.0, 60.0),
    (40.0, 100.0), (60.0, 100.0), (40.0, 150.0), (60.0, 150.0),
    (40.0, 200.0), (60.0, 200.0),
    (240.0, 100.0), (260.0, 100.0), (240.0, 150.0), (260.0, 150.0),
    (240.0, 200.0), (260.0, 200.0),
    (100.0, 240.0), (150.0, 240.0), (200.0, 240.0),
    (100.0, 260.0), (150.0, 260.0), (200.0, 260.0),
)

STRAIGHT_SEQUENCE_CLOCKWISE = ("bottom", "left", "top", "right", "bottom")
STRAIGHT_SEQUENCE_COUNTERCLOCKWISE = ("bottom", "right", "top", "left", "bottom")


def start_zone_contains(x_cm: float, y_cm: float) -> bool:
    x, y, width, height = START_ZONE
    return x <= x_cm <= x + width and y <= y_cm <= y + height


def straight_sequence(clockwise: bool) -> tuple[str, ...]:
    return STRAIGHT_SEQUENCE_CLOCKWISE if clockwise else STRAIGHT_SEQUENCE_COUNTERCLOCKWISE


def route_waypoints(clockwise: bool = True) -> tuple[Point, ...]:
    """Ruta redondeada comun, con salida y regreso a la misma pose de ruta."""
    if clockwise:
        return (
            (125.0, 280.0), (85.0, 250.0),
            (50.0, 215.0), (50.0, 85.0),
            (85.0, 50.0), (215.0, 50.0),
            (250.0, 85.0), (250.0, 215.0),
            (215.0, 250.0), (125.0, 280.0),
        )
    return (
        (125.0, 280.0), (215.0, 250.0),
        (250.0, 215.0), (250.0, 85.0),
        (215.0, 50.0), (85.0, 50.0),
        (50.0, 85.0), (50.0, 215.0),
        (85.0, 250.0), (125.0, 280.0),
    )


def route_centerline(clockwise: bool = True, sample_spacing_cm: float = 2.5) -> tuple[Point, ...]:
    """Muestrea la ruta para que Pygame y el runner usen exactamente los mismos puntos."""
    if sample_spacing_cm <= 0:
        raise ValueError("sample_spacing_cm debe ser positivo")
    waypoints = route_waypoints(clockwise)
    points: list[Point] = []
    for start, end in zip(waypoints, waypoints[1:]):
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        steps = max(1, math.ceil(distance / sample_spacing_cm))
        points.extend(
            (
                start[0] + (end[0] - start[0]) * step / steps,
                start[1] + (end[1] - start[1]) * step / steps,
            )
            for step in range(steps)
        )
    points.append(waypoints[-1])
    return tuple(points)

