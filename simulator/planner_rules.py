"""Reglas físicas y reglamentarias que no deben ser optimizadas.

Este módulo no contiene parámetros de tuning ni importa Pygame. Los valores
ajustables viven en :mod:`planner_tuning`.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

try:
    from track_config import INNER_WALL, OUTER_WALL, START_POSE, START_ZONE
except ImportError:
    from simulator.track_config import INNER_WALL, OUTER_WALL, START_POSE, START_ZONE


PHYSICAL_MEASUREMENTS_PATH = Path(__file__).resolve().parents[1] / "config" / "physical_measurements.json"


def _physical_measurements() -> dict:
    """Fuente única de medidas del carro para simulador y robot."""
    data = json.loads(PHYSICAL_MEASUREMENTS_PATH.read_text(encoding="utf-8"))
    if not isinstance(data.get("measurements"), dict):
        raise ValueError(f"Formato inválido: {PHYSICAL_MEASUREMENTS_PATH}")
    return data


_PHYSICAL = _physical_measurements()
_MEASUREMENTS = _PHYSICAL["measurements"]
_WHEEL_ANGLES = [
    abs(float(value))
    for row in _PHYSICAL.get("steering_measurements", {}).get("wheel_angles_deg", [])
    for value in (row.get("left_wheel_deg"), row.get("right_wheel_deg"))
    if value is not None
]


def _measured_wheel_angle(turn: str, wheel: str) -> float:
    """Obtiene un ángulo de rueda medido, sin inventar una simetría."""
    for row in _PHYSICAL.get("steering_measurements", {}).get("wheel_angles_deg", []):
        if row.get("turn") == turn:
            return float(row[f"{wheel}_wheel_deg"])
    raise ValueError(f"Falta la medición {turn}/{wheel} en {PHYSICAL_MEASUREMENTS_PATH}")
_WHEELBASE_CM = float(_MEASUREMENTS["wheelbase_cm"])
_RIGHT_RADIUS_CM = float(_MEASUREMENTS["turn_radius_right_cm"])
_LEFT_RADIUS_CM = float(_MEASUREMENTS["turn_radius_left_cm"])
_MAX_STEERING_DEG = max(
    math.degrees(math.atan(_WHEELBASE_CM / _RIGHT_RADIUS_CM)),
    math.degrees(math.atan(_WHEELBASE_CM / _LEFT_RADIUS_CM)),
)


@dataclass(frozen=True)
class FixedRules:
    """Reglas comunes a Pygame, runner y planner."""

    vehicle_length_cm: float = float(_MEASUREMENTS["overall_front_to_rear_cm"])
    # Envolvente lateral de ruedas: es la anchura que debe evitar muros.
    vehicle_width_cm: float = float(_MEASUREMENTS["wheel_outer_envelope_width_cm"])
    wheelbase_cm: float = _WHEELBASE_CM
    turn_radius_right_cm: float = _RIGHT_RADIUS_CM
    turn_radius_left_cm: float = _LEFT_RADIUS_CM
    # Hipótesis geométrica simétrica, derivada del wheelbase medido.
    front_axle_offset_cm: float = _WHEELBASE_CM / 2
    rear_axle_offset_cm: float = _WHEELBASE_CM / 2
    front_wheel_center_track_cm: float = float(_MEASUREMENTS["front_wheel_center_track_cm"])
    wheel_width_cm: float = float(_MEASUREMENTS["wheel_width_cm"])
    wheel_diameter_cm: float = float(_MEASUREMENTS["wheel_diameter_cm"])
    default_obstacle_length_cm: float = 5.0
    default_obstacle_width_cm: float = 5.0
    maximum_physical_steering_deg: float = _MAX_STEERING_DEG
    maximum_measured_front_wheel_deg: float = max(_WHEEL_ANGLES, default=0.0)
    # Pares físicos: rueda interna y externa no giran el mismo ángulo.
    right_turn_left_wheel_deg: float = _measured_wheel_angle("right", "left")
    right_turn_right_wheel_deg: float = _measured_wheel_angle("right", "right")
    left_turn_left_wheel_deg: float = _measured_wheel_angle("left", "left")
    left_turn_right_wheel_deg: float = _measured_wheel_angle("left", "right")
    servo_center_deg: float = float(_MEASUREMENTS["servo_center_deg"])
    servo_logical_center_deg: float = float(_MEASUREMENTS["servo_logical_center_deg"])
    servo_offset_deg: float = float(_MEASUREMENTS["servo_offset_deg"])
    servo_safe_min_deg: float = float(_MEASUREMENTS["servo_safe_min_deg"])
    servo_safe_max_deg: float = float(_MEASUREMENTS["servo_safe_max_deg"])
    camera_height_cm: float = float(_MEASUREMENTS.get("camera_height_cm", 8.8))
    camera_vertical_angle_deg: float | None = _MEASUREMENTS.get("camera_vertical_angle_deg")
    vision_safe_distance_cm: float = float(_PHYSICAL.get("vision", {}).get("safe_distance_mm", 340.0)) / 10.0
    vision_safe_zone_top_ratio: float = float(_PHYSICAL.get("vision", {}).get("safe_zone_top_ratio", 0.68))
    horizontal_fov_deg: float = 70.4
    forward_projection_cm: float = 30.0
    route_alignment_tolerance_deg: float = 8.0
    simulation_dt_s: float = 0.05
    red_pass_side: str = "right"
    green_pass_side: str = "left"
    outer_wall: tuple[float, float, float, float] = OUTER_WALL
    inner_wall: tuple[float, float, float, float] = INNER_WALL
    start_zone: tuple[float, float, float, float] = START_ZONE
    start_pose: tuple[float, float, float] = START_POSE
    phases: tuple[str, ...] = (
        "TURN_OUT", "PASS_HOLD", "COUNTER_STEER", "REJOIN_CENTER"
    )


FIXED_RULES = FixedRules()
