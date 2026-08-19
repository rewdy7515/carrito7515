"""Planificación geométrica de trayectorias seguras alrededor de señales.

El sistema usa el modelo de bicicleta: el vehículo se representa por su eje
trasero y una rueda directriz equivalente. No mueve el robot; devuelve fases
en milímetros y direcciones semánticas que el controlador debe ejecutar con
encoder, odometría visual o una estimación de velocidad calibrada.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional


class BypassSide(str, Enum):
    LEFT = "left"
    RIGHT = "right"


@dataclass(frozen=True)
class VehicleGeometry:
    """Medidas del vehículo referidas al centro del eje trasero."""

    length_mm: float = 211.5
    # Envolvente lateral de ruedas: 149 mm entre centros + 23 mm de rueda.
    width_mm: float = 172.0
    wheelbase_mm: float = 150.0

    @property
    def front_overhang_mm(self) -> float:
        return max(0.0, (self.length_mm - self.wheelbase_mm) / 2)


@dataclass(frozen=True)
class ObstacleGeometry:
    width_mm: float = 50.0
    depth_mm: float = 50.0
    height_mm: float = 100.0


@dataclass(frozen=True)
class ObstaclePose:
    """Posición de la señal respecto al centro del eje trasero.

    ``forward_mm`` es positivo hacia delante; ``lateral_mm`` es positivo a la
    izquierda del robot.
    """

    forward_mm: float
    lateral_mm: float


@dataclass(frozen=True)
class TrajectoryPhase:
    name: str
    steering: BypassSide | None
    distance_mm: float


@dataclass(frozen=True)
class TrajectoryPlan:
    safe: bool
    reason: str
    obstacle: ObstaclePose
    target_lateral_mm: float
    turning_radius_mm: Optional[float]
    phases: tuple[TrajectoryPhase, ...]

    @property
    def total_distance_mm(self) -> float:
        return sum(phase.distance_mm for phase in self.phases)


class ObstaclePoseEstimator:
    """Convierte bounding box de cámara en posición relativa aproximada.

    Requiere una distancia estimada mediante el alto conocido de la señal y
    un foco equivalente calibrado. Supone que la cámara está centrada respecto
    al eje longitudinal; un desplazamiento físico de cámara debe añadirse en
    una siguiente calibración.
    """

    def __init__(self, focal_length_px: float, obstacle_height_mm: float = 100.0) -> None:
        if focal_length_px <= 0 or obstacle_height_mm <= 0:
            raise ValueError("El foco y el alto del obstáculo deben ser positivos")
        self.focal_length_px = focal_length_px
        self.obstacle_height_mm = obstacle_height_mm

    def estimate_from_bbox(
        self,
        bounding_box: tuple[int, int, int, int],
        frame_width_px: int,
    ) -> ObstaclePose:
        x, _, width, height = bounding_box
        if height <= 0 or frame_width_px <= 0:
            raise ValueError("Bounding box o ancho de frame inválido")

        distance_mm = self.focal_length_px * self.obstacle_height_mm / height
        center_x = x + width / 2
        optical_center_x = frame_width_px / 2
        bearing_rad = math.atan2(center_x - optical_center_x, self.focal_length_px)

        # La imagen crece hacia la derecha; en la convención del planificador
        # la izquierda del robot es positiva.
        forward_mm = distance_mm * math.cos(bearing_rad)
        lateral_mm = -distance_mm * math.sin(bearing_rad)
        return ObstaclePose(forward_mm=forward_mm, lateral_mm=lateral_mm)


class SafeTrajectoryPlanner:
    """Genera una trayectoria S: desplazarse, rebasar y regresar al eje."""

    def __init__(
        self,
        vehicle: VehicleGeometry = VehicleGeometry(),
        obstacle: ObstacleGeometry = ObstacleGeometry(),
        lateral_margin_mm: float = 80.0,
        longitudinal_margin_mm: float = 100.0,
        maximum_heading_change_deg: float = 70.0,
    ) -> None:
        self.vehicle = vehicle
        self.obstacle = obstacle
        self.lateral_margin_mm = lateral_margin_mm
        self.longitudinal_margin_mm = longitudinal_margin_mm
        self.maximum_heading_change_rad = math.radians(maximum_heading_change_deg)

    def plan_bypass(
        self,
        obstacle_pose: ObstaclePose,
        side: BypassSide,
        road_wheel_angle_deg: float,
    ) -> TrajectoryPlan:
        """Calcula una trayectoria S segura o explica por qué no es posible.

        ``road_wheel_angle_deg`` es el giro real de las ruedas, no el ángulo
        del servo. Debe obtenerse mediante una prueba física de calibración.
        """
        if road_wheel_angle_deg <= 0 or road_wheel_angle_deg >= 89:
            return self._unsafe(obstacle_pose, "Ángulo real de ruedas no calibrado")

        radius_mm = self.vehicle.wheelbase_mm / math.tan(
            math.radians(road_wheel_angle_deg)
        )
        required_separation_mm = (
            self.vehicle.width_mm / 2
            + self.obstacle.width_mm / 2
            + self.lateral_margin_mm
        )
        target_lateral_mm = (
            obstacle_pose.lateral_mm + required_separation_mm
            if side is BypassSide.LEFT
            else obstacle_pose.lateral_mm - required_separation_mm
        )
        lateral_shift_mm = abs(target_lateral_mm)

        cos_heading = 1 - lateral_shift_mm / (2 * radius_mm)
        if cos_heading < -1 or cos_heading > 1:
            return self._unsafe(
                obstacle_pose,
                "El radio de giro no permite alcanzar el desplazamiento lateral requerido",
                target_lateral_mm,
                radius_mm,
            )
        heading_change_rad = math.acos(cos_heading)
        if heading_change_rad > self.maximum_heading_change_rad:
            return self._unsafe(
                obstacle_pose,
                "El desplazamiento requiere un giro demasiado cerrado",
                target_lateral_mm,
                radius_mm,
            )

        arc_distance_mm = radius_mm * heading_change_rad
        shift_forward_mm = 2 * radius_mm * math.sin(heading_change_rad)
        required_forward_mm = (
            shift_forward_mm
            + self.obstacle.depth_mm / 2
            + self.vehicle.front_overhang_mm
            + self.longitudinal_margin_mm
        )
        if obstacle_pose.forward_mm <= required_forward_mm:
            return self._unsafe(
                obstacle_pose,
                "El obstáculo está demasiado cerca para iniciar un esquive seguro",
                target_lateral_mm,
                radius_mm,
            )

        first_turn = side
        opposite_turn = BypassSide.RIGHT if side is BypassSide.LEFT else BypassSide.LEFT
        pass_distance_mm = (
            self.obstacle.depth_mm
            + self.vehicle.length_mm
            + 2 * self.longitudinal_margin_mm
        )
        advance_before_turn_mm = obstacle_pose.forward_mm - required_forward_mm
        phases = (
            TrajectoryPhase("advance_to_bypass", None, advance_before_turn_mm),
            TrajectoryPhase("shift_out", first_turn, arc_distance_mm),
            TrajectoryPhase("straighten_after_shift", opposite_turn, arc_distance_mm),
            TrajectoryPhase("pass_obstacle", None, pass_distance_mm),
            TrajectoryPhase("return_to_lane", opposite_turn, arc_distance_mm),
            TrajectoryPhase("straighten_after_return", first_turn, arc_distance_mm),
        )
        return TrajectoryPlan(
            safe=True,
            reason="Trayectoria S calculada",
            obstacle=obstacle_pose,
            target_lateral_mm=target_lateral_mm,
            turning_radius_mm=radius_mm,
            phases=phases,
        )

    @staticmethod
    def _unsafe(
        obstacle: ObstaclePose,
        reason: str,
        target_lateral_mm: float = 0.0,
        turning_radius_mm: Optional[float] = None,
    ) -> TrajectoryPlan:
        return TrajectoryPlan(
            safe=False,
            reason=reason,
            obstacle=obstacle,
            target_lateral_mm=target_lateral_mm,
            turning_radius_mm=turning_radius_mm,
            phases=(),
        )
