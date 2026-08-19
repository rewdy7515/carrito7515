"""Conducción autónoma geométrica para Raspberry Pi.

Flujo único:

    camera.py -> detecciones relativas -> tres predicciones -> primer comando

El planner no conoce Pygame ni posiciones secretas de la pista. Solo usa lo
que ve la cámara y la física configurada del carro. El ángulo interno es el
ángulo físico de las ruedas; la conversión a servo está aislada en
``ServoOutput``.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import select
import sys
import termios
import time
import tty

import numpy as np
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

try:
    import serial
except ImportError:
    serial = None

try:
    import cv2
except ImportError:
    cv2 = None

SERIAL_BAUDRATE = 115200
COMMAND_PERIOD_S = 0.05
ARDUINO_STARTUP_DELAY_S = 2.0
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
DEFAULT_CALIBRATION_PATH = Path(__file__).resolve().parents[3] / "config" / "camera_calibration.json"
DEFAULT_PHYSICAL_MEASUREMENTS_PATH = Path(__file__).resolve().parents[3] / "config" / "physical_measurements.json"


class PassSide(str, Enum):
    LEFT = "left"
    RIGHT = "right"


@dataclass(frozen=True)
class VehiclePhysics:
    length_cm: float = 21.15
    # Envolvente lateral de ruedas: 14.9 cm entre centros + 2.3 cm de rueda.
    width_cm: float = 17.2
    wheelbase_cm: float = 15.0
    turn_radius_right_cm: float = 32.2
    turn_radius_left_cm: float = 43.0
    max_steering_deg: float = 20.7
    steering_rate_deg_s: float = 90.0
    # Medición del usuario: 120 cm en 4.9 s con PWM=120.
    fixed_speed_cm_s: float = 24.5
    safety_margin_cm: float = 15.0
    camera_height_cm: float = 8.8
    dt_s: float = 0.05
    front_wheel_center_track_cm: float = 14.9
    wheel_width_cm: float = 2.3
    wheel_diameter_cm: float = 6.8
    servo_center_deg: float = 92.0
    servo_logical_center_deg: float = 92.0
    servo_offset_deg: float = 0.0
    servo_safe_min_deg: float = 60.0
    servo_safe_max_deg: float = 120.0
    vision_safe_distance_cm: float = 34.0
    maximum_measured_front_wheel_deg: float = 0.0

    @classmethod
    def from_measurements(
        cls,
        data: dict,
        fixed_speed_cm_s: float,
        safety_margin_cm: float,
    ) -> "VehiclePhysics":
        measurements = data["measurements"]
        wheel_angles = [
            abs(float(angle))
            for item in data.get("steering_measurements", {}).get("wheel_angles_deg", [])
            for angle in (item.get("left_wheel_deg"), item.get("right_wheel_deg"))
            if angle is not None
        ]
        right_radius = float(measurements["turn_radius_right_cm"])
        left_radius = float(measurements["turn_radius_left_cm"])
        wheelbase = float(measurements["wheelbase_cm"])
        derived_max = max(
            math.degrees(math.atan(wheelbase / right_radius)),
            math.degrees(math.atan(wheelbase / left_radius)),
        )
        vision = data.get("vision", {})
        return cls(
            length_cm=float(measurements["overall_front_to_rear_cm"]),
            width_cm=float(measurements["wheel_outer_envelope_width_cm"]),
            wheelbase_cm=wheelbase,
            turn_radius_right_cm=right_radius,
            turn_radius_left_cm=left_radius,
            max_steering_deg=derived_max,
            steering_rate_deg_s=90.0,
            fixed_speed_cm_s=fixed_speed_cm_s,
            safety_margin_cm=safety_margin_cm,
            camera_height_cm=float(measurements.get("camera_height_cm", 8.8)),
            front_wheel_center_track_cm=float(measurements["front_wheel_center_track_cm"]),
            wheel_width_cm=float(measurements["wheel_width_cm"]),
            wheel_diameter_cm=float(measurements["wheel_diameter_cm"]),
            servo_center_deg=float(measurements["servo_center_deg"]),
            servo_logical_center_deg=float(measurements["servo_logical_center_deg"]),
            servo_offset_deg=float(measurements["servo_offset_deg"]),
            servo_safe_min_deg=float(measurements["servo_safe_min_deg"]),
            servo_safe_max_deg=float(measurements["servo_safe_max_deg"]),
            vision_safe_distance_cm=float(vision.get("safe_distance_mm", 340.0)) / 10.0,
            maximum_measured_front_wheel_deg=max(wheel_angles, default=0.0),
        )

    @property
    def max_right_steering_deg(self) -> float:
        return math.degrees(math.atan(self.wheelbase_cm / self.turn_radius_right_cm))

    @property
    def max_left_steering_deg(self) -> float:
        return math.degrees(math.atan(self.wheelbase_cm / self.turn_radius_left_cm))

    def clamp_steering(self, steering_deg: float) -> float:
        return clamp(steering_deg, -self.max_left_steering_deg, self.max_right_steering_deg)


@dataclass(frozen=True)
class VehicleState:
    x_cm: float = 0.0
    y_cm: float = 0.0
    heading_rad: float = 0.0
    steering_deg: float = 0.0


@dataclass(frozen=True)
class VisibleObstacle:
    object_id: str
    x_cm: float
    y_cm: float
    width_cm: float
    length_cm: float
    color: str


@dataclass(frozen=True)
class VisibleWall:
    x_cm: float
    y_cm: float
    width_cm: float
    length_cm: float


@dataclass(frozen=True)
class PlannerInput:
    vehicle: VehicleState
    obstacles: tuple[VisibleObstacle, ...] = ()
    walls: tuple[VisibleWall, ...] = ()


@dataclass(frozen=True)
class ControlCommand:
    speed_cm_s: float
    steering_angle_deg: float


@dataclass
class Prediction:
    steering_angle_deg: float
    target_lateral_cm: float
    safe: bool = False
    collision: bool = False
    minimum_clearance_cm: float = math.inf
    trajectory: list[tuple[float, float]] = field(default_factory=list)


@dataclass
class PlannerResult:
    command: ControlCommand
    target_id: str | None
    pass_side: PassSide | None
    predictions: list[Prediction]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def load_physical_measurements(path: Path = DEFAULT_PHYSICAL_MEASUREMENTS_PATH) -> dict:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data.get("measurements"), dict):
        raise ValueError(f"Formato inválido: {path}")
    return data


def body_corners(state: VehicleState, physics: VehiclePhysics):
    """Esquinas del rectángulo completo del carro, no un círculo."""
    c, s = math.cos(state.heading_rad), math.sin(state.heading_rad)
    forward, right = (c, s), (-s, c)
    half_l, half_w = physics.length_cm / 2, physics.width_cm / 2
    return tuple(
        (
            state.x_cm + a * half_l * forward[0] + b * half_w * right[0],
            state.y_cm + a * half_l * forward[1] + b * half_w * right[1],
        )
        for a, b in ((1, 1), (1, -1), (-1, -1), (-1, 1))
    )


def rectangle_box(center_x: float, center_y: float, width: float, length: float):
    return (
        (center_x - length / 2, center_y - width / 2),
        (center_x + length / 2, center_y + width / 2),
    )


def body_box(state: VehicleState, physics: VehiclePhysics):
    points = body_corners(state, physics)
    return (
        (min(p[0] for p in points), min(p[1] for p in points)),
        (max(p[0] for p in points), max(p[1] for p in points)),
    )


def overlaps(a, b) -> bool:
    return not (
        a[1][0] < b[0][0] or b[1][0] < a[0][0]
        or a[1][1] < b[0][1] or b[1][1] < a[0][1]
    )


def clearance(a, b) -> float:
    dx = max(b[0][0] - a[1][0], a[0][0] - b[1][0], 0.0)
    dy = max(b[0][1] - a[1][1], a[0][1] - b[1][1], 0.0)
    return math.hypot(dx, dy)


class GeometricPlanner:
    """Calcula tres giros y devuelve únicamente el primer comando."""

    def __init__(self, physics: VehiclePhysics, horizon_s: float = 6.0):
        if physics.wheelbase_cm <= 0 or physics.max_steering_deg <= 0:
            raise ValueError("Geometría física inválida")
        self.physics = physics
        self.horizon_s = max(horizon_s, physics.dt_s)
        self.target_id: str | None = None
        self.pass_side: PassSide | None = None
        self.last_turn_steering_deg = 0.0
        # No hay odometría disponible; conservar el ángulo de ruedas sí evita
        # reiniciar artificialmente el modelo a 0° en cada frame.
        self.last_commanded_steering_deg = 0.0
        self.rejoin_steering_deg = 0.0
        self.rejoin_until = 0.0
        self.turn_started_at = 0.0
        self.turn_duration_s = 0.0

    @staticmethod
    def side_for(obstacle: VisibleObstacle) -> PassSide:
        return PassSide.RIGHT if obstacle.color.lower() == "red" else PassSide.LEFT

    def nearest_obstacle(self, data: PlannerInput) -> VisibleObstacle | None:
        front = self.physics.length_cm / 2
        candidates = [
            item for item in data.obstacles
            if item.x_cm + item.length_cm / 2 >= front
        ]
        return min(candidates, key=lambda item: item.x_cm) if candidates else None

    def advance(self, state: VehicleState, requested_steering: float) -> VehicleState:
        dt = self.physics.dt_s
        max_change = self.physics.steering_rate_deg_s * dt
        steering = state.steering_deg + clamp(
            requested_steering - state.steering_deg, -max_change, max_change
        )
        steering = self.physics.clamp_steering(steering)
        heading = state.heading_rad + (
            self.physics.fixed_speed_cm_s / self.physics.wheelbase_cm
            * math.tan(math.radians(steering)) * dt
        )
        return VehicleState(
            state.x_cm + self.physics.fixed_speed_cm_s * math.cos(heading) * dt,
            state.y_cm + self.physics.fixed_speed_cm_s * math.sin(heading) * dt,
            heading,
            steering,
        )

    def evaluate(
        self,
        data: PlannerInput,
        obstacle: VisibleObstacle,
        steering: float,
        target_lateral: float,
    ) -> Prediction:
        result = Prediction(steering, target_lateral)
        state = data.vehicle
        obstacle_box = rectangle_box(
            obstacle.x_cm, obstacle.y_cm, obstacle.width_cm, obstacle.length_cm
        )
        wall_boxes = [
            rectangle_box(w.x_cm, w.y_cm, w.width_cm, w.length_cm)
            for w in data.walls
        ]
        phase = "TURN_OUT"
        elapsed = 0.0
        result.trajectory.append((state.x_cm, state.y_cm))
        while elapsed < self.horizon_s:
            if phase == "COUNTER_STEER":
                phase_steering = -steering
            elif phase == "REJOIN_CENTER":
                phase_steering = 0.0
            else:
                phase_steering = steering
            state = self.advance(state, phase_steering)
            result.trajectory.append((state.x_cm, state.y_cm))
            car = body_box(state, self.physics)
            result.minimum_clearance_cm = min(
                result.minimum_clearance_cm,
                clearance(car, obstacle_box),
                *(clearance(car, wall) for wall in wall_boxes),
            )
            if overlaps(car, obstacle_box) or any(overlaps(car, wall) for wall in wall_boxes):
                # Marcar la colisión, pero continuar simulando para que la
                # cámara muestre el horizonte completo de la trayectoria.
                result.collision = True

            lateral_shift = abs(state.y_cm - obstacle.y_cm)
            required_shift = abs(target_lateral - obstacle.y_cm)
            rear = state.x_cm - self.physics.length_cm / 2
            obstacle_front = obstacle.x_cm + obstacle.length_cm / 2
            if phase == "TURN_OUT" and lateral_shift >= required_shift:
                phase = "PASS_HOLD"
            elif phase == "PASS_HOLD" and rear > obstacle_front + self.physics.safety_margin_cm:
                phase = "COUNTER_STEER"
            elif phase == "COUNTER_STEER" and abs(state.heading_rad) < math.radians(8):
                phase = "REJOIN_CENTER"
            elapsed += self.physics.dt_s

        side_ok = (state.y_cm - obstacle.y_cm) * (target_lateral - obstacle.y_cm) >= 0
        result.safe = (
            not result.collision
            and side_ok
            and result.minimum_clearance_cm >= self.physics.safety_margin_cm
        )
        return result


    def plan(self, data: PlannerInput) -> PlannerResult:
        obstacle = self.nearest_obstacle(data)

        # Mantener el contra-giro durante toda la ventana de reincorporación.
        # La detección puede seguir apareciendo durante algunos frames después
        # de pasar el obstáculo; no debe cancelar la maniobra inversa.
        if time.monotonic() < self.rejoin_until:
            return PlannerResult(
                ControlCommand(self.physics.fixed_speed_cm_s, self.rejoin_steering_deg),
                None,
                None,
                [],
            )

        if obstacle is None:
            started_rejoin = False
            if self.target_id is not None and abs(self.last_turn_steering_deg) > 0.1:
                now = time.monotonic()
                if self.turn_started_at > 0.0:
                    self.turn_duration_s = clamp(
                        now - self.turn_started_at,
                        0.20,
                        3.00,
                    )
                else:
                    self.turn_duration_s = 0.20
                self.rejoin_steering_deg = self.physics.clamp_steering(
                    -self.last_turn_steering_deg
                )
                self.rejoin_until = now + self.turn_duration_s
                started_rejoin = True
            self.target_id = None
            self.pass_side = None
            self.last_turn_steering_deg = 0.0
            if started_rejoin:
                return PlannerResult(
                    ControlCommand(self.physics.fixed_speed_cm_s, self.rejoin_steering_deg),
                    None,
                    None,
                    [],
                )
            return PlannerResult(
                ControlCommand(self.physics.fixed_speed_cm_s, 0.0), None, None, []
            )

        if obstacle.object_id != self.target_id:
            self.target_id = obstacle.object_id
            self.pass_side = self.side_for(obstacle)
        side = self.pass_side or self.side_for(obstacle)
        sign = 1.0 if side is PassSide.RIGHT else -1.0
        target_lateral = obstacle.y_cm + sign * (
            self.physics.width_cm / 2
            + obstacle.width_cm / 2
            + self.physics.safety_margin_cm
        )
        distance = max(
            obstacle.x_cm - self.physics.length_cm / 2,
            self.physics.length_cm,
        )
        lateral_shift = abs(target_lateral - data.vehicle.y_cm)
        curvature = 2 * lateral_shift / max(
            distance * distance + lateral_shift * lateral_shift, 1e-9
        )
        calculated = math.degrees(math.atan(self.physics.wheelbase_cm * curvature))
        maximum = (
            self.physics.max_right_steering_deg
            if side is PassSide.RIGHT
            else self.physics.max_left_steering_deg
        )
        angles = (
            maximum * 0.55,
            clamp(max(calculated, maximum * 0.70), 0.0, maximum * 0.90),
            maximum,
        )
        predictions = [
            self.evaluate(data, obstacle, sign * angle, target_lateral)
            for angle in angles
        ]
        safe = [item for item in predictions if item.safe]
        selected = max(safe, key=lambda item: item.minimum_clearance_cm, default=None)
        if selected is None:
            # Si ninguna predicción completa todo el horizonte, no detenerse
            # inmediatamente: mientras el obstáculo aún esté fuera de la
            # distancia crítica, elegir la maniobra con mayor separación y
            # seguir avanzando para iniciar el esquive.
            emergency_distance = (
                self.physics.length_cm / 2 + obstacle.length_cm / 2
            )
            if obstacle.x_cm > emergency_distance:
                selected = max(
                    predictions,
                    key=lambda item: (
                        not item.collision,
                        abs(item.steering_angle_deg),
                        item.minimum_clearance_cm,
                    ),
                )
        # Si ninguna predicción cumple colisión, lado y margen, no se envía
        # una trayectoria inválida. Cerca del obstáculo se mantiene la parada
        # de emergencia; antes de ese punto se usa el mejor giro disponible.
        # Conservar el último ángulo real usado para esquivar. Si una
        # predicción falla o devuelve 0°, todavía necesitamos ese ángulo para
        # generar el contra-giro al terminar el adelantamiento.
        if selected is not None and abs(selected.steering_angle_deg) > 0.1:
            now = time.monotonic()
            if (
                abs(self.last_turn_steering_deg) <= 0.1
                or self.last_turn_steering_deg * selected.steering_angle_deg < 0
            ):
                self.turn_started_at = now
            self.last_turn_steering_deg = selected.steering_angle_deg
        return PlannerResult(
            ControlCommand(
                self.physics.fixed_speed_cm_s if selected else 0.0,
                selected.steering_angle_deg if selected else 0.0,
            ),
            obstacle.object_id,
            side,
            predictions,
        )


def draw_planner_predictions(frame, plan: PlannerResult, physics: VehiclePhysics, calibration) -> None:
    """Dibuja en la imagen las franjas de trayectoria evaluadas por el planner."""
    if cv2 is None or calibration is None:
        return

    def ground_to_pixel(x_cm: float, y_cm: float) -> tuple[int, int] | None:
        if x_cm <= 0:
            return None
        u = calibration.center_x_px + calibration.focal_x_px * y_cm / x_cm
        vertical_angle = math.atan2(calibration.height_cm, x_cm) - math.radians(calibration.pitch_deg)
        v = calibration.center_y_px + calibration.focal_y_px * math.tan(vertical_angle)
        return round(u), round(v)

    if not plan.predictions:
        # También dibujar el trayecto durante la reincorporación. Antes solo
        # se dibujaba una línea recta cuando no había un obstáculo visible,
        # ocultando precisamente el contra-giro.
        state = VehicleState()
        path = []
        steps = 120
        for _ in range(steps):
            path.append(ground_to_pixel(state.x_cm, state.y_cm))
            requested = plan.command.steering_angle_deg
            max_change = physics.steering_rate_deg_s * physics.dt_s
            steering = state.steering_deg + clamp(
                requested - state.steering_deg, -max_change, max_change
            )
            steering = physics.clamp_steering(steering)
            heading = state.heading_rad + (
                physics.fixed_speed_cm_s / physics.wheelbase_cm
                * math.tan(math.radians(steering)) * physics.dt_s
            )
            state = VehicleState(
                state.x_cm + physics.fixed_speed_cm_s * math.cos(heading) * physics.dt_s,
                state.y_cm + physics.fixed_speed_cm_s * math.sin(heading) * physics.dt_s,
                heading,
                steering,
            )
        pixels = [point for point in path if point is not None]
        if len(pixels) >= 2:
            color = (255, 0, 255) if abs(plan.command.steering_angle_deg) > 0.1 else (255, 200, 0)
            cv2.polylines(frame, [np.array(pixels, dtype=np.int32)], False, color, 5, cv2.LINE_AA)
            label = "trayectoria contra-giro" if abs(plan.command.steering_angle_deg) > 0.1 else "trayectoria recta"
            cv2.putText(frame, label, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)
        return

    selected = plan.command.steering_angle_deg
    for prediction in plan.predictions:
        pixels = [ground_to_pixel(x, y) for x, y in prediction.trajectory]
        pixels = [point for point in pixels if point is not None]
        if len(pixels) < 2:
            continue
        is_selected = plan.command.speed_cm_s > 0 and abs(prediction.steering_angle_deg - selected) < 1e-6
        if is_selected:
            color, thickness = (0, 255, 255), 5
        elif prediction.safe:
            color, thickness = (0, 200, 0), 3
        else:
            color, thickness = (0, 0, 255), 3
        cv2.polylines(frame, [np.array(pixels, dtype=np.int32)], False, color, thickness, cv2.LINE_AA)

        if is_selected:
            end_x, end_y = pixels[-1]
            cv2.circle(frame, (end_x, end_y), 6, color, -1, cv2.LINE_AA)
            cv2.putText(
                frame,
                f"giro {prediction.steering_angle_deg:+.1f} deg | horizonte completo",
                (8, 36),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.38,
                color,
                1,
                cv2.LINE_AA,
            )

    cv2.putText(
        frame,
        "trayectorias: amarillo=elegida verde=segura rojo=descartada",
        (8, 18),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.38,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )


def detection_to_obstacle(
    detection,
    frame_width_px: int,
    frame_height_px: int,
    focal_y_px: float,
    obstacle_height_cm: float,
    camera_height_cm: float | None = None,
    camera_pitch_deg: float | None = None,
    focal_x_px: float | None = None,
    camera_center_x_px: float | None = None,
    camera_center_y_px: float | None = None,
) -> VisibleObstacle:
    """Convierte una detección usando el centro inferior como referencia.

    El centro inferior representa el punto donde el obstáculo toca el suelo:
    ``(x + width/2, y + height)``. La profundidad se estima con la altura
    aparente conocida; el punto inferior determina la posición lateral del
    objeto y evita usar el centro visual, que queda elevado sobre el suelo.
    """
    x, y, width_px, height_px = detection.bounding_box
    focal_x = focal_y_px if focal_x_px is None else focal_x_px
    center_x = frame_width_px / 2 if camera_center_x_px is None else camera_center_x_px
    center_y = frame_height_px / 2 if camera_center_y_px is None else camera_center_y_px
    if height_px <= 0 or frame_width_px <= 0 or focal_y_px <= 0 or focal_x <= 0:
        raise ValueError("Detección o calibración de cámara inválida")
    bottom_center_x_px = x + width_px / 2
    bottom_center_y_px = y + height_px
    if camera_height_cm is not None and camera_pitch_deg is not None:
        ground_angle = math.atan2(
            bottom_center_y_px - center_y,
            focal_y_px,
        ) + math.radians(camera_pitch_deg)
        if ground_angle > 0:
            distance = camera_height_cm / math.tan(ground_angle)
            lateral = distance * (bottom_center_x_px - center_x) / focal_x
        else:
            # Un muro puede terminar visualmente sobre el horizonte o tener
            # una detección recortada. No abortar todo el autónomo: usar el
            # respaldo por tamaño aparente para conservar una estimación.
            distance = focal_y_px * obstacle_height_cm / height_px
            lateral = distance * (bottom_center_x_px - center_x) / focal_x
    else:
        # Respaldo basado en el tamaño conocido del obstáculo.
        distance = focal_y_px * obstacle_height_cm / height_px
        lateral = distance * (bottom_center_x_px - center_x) / focal_x
    size = distance * width_px / focal_x
    return VisibleObstacle(
        f"{detection.color}-{round(bottom_center_x_px / 4)}-{round(bottom_center_y_px / 4)}",
        distance,
        lateral,
        size,
        size,
        detection.color,
    )


def detection_to_wall(
    detection,
    frame_width_px: int,
    frame_height_px: int,
    focal_y_px: float,
    wall_height_cm: float,
    camera_height_cm: float | None = None,
    camera_pitch_deg: float | None = None,
    focal_x_px: float | None = None,
    camera_center_x_px: float | None = None,
    camera_center_y_px: float | None = None,
    wall_thickness_cm: float = 3.0,
) -> VisibleWall:
    obstacle = detection_to_obstacle(
        detection, frame_width_px, frame_height_px, focal_y_px,
        wall_height_cm, camera_height_cm, camera_pitch_deg,
        focal_x_px, camera_center_x_px, camera_center_y_px,
    )
    # El muro es una barrera vertical; su profundidad física no es la altura
    # aparente de la máscara. Usar una profundidad pequeña evita convertirlo
    # accidentalmente en un bloque enorme dentro del planner.
    return VisibleWall(
        obstacle.x_cm,
        obstacle.y_cm,
        obstacle.width_cm,
        max(wall_thickness_cm, 0.5),
    )


def draw_obstacle_distances(frame, detections, obstacles) -> None:
    """Dibuja la distancia del suelo junto a cada obstáculo detectado."""
    if cv2 is None:
        return
    for detection, obstacle in zip(detections, obstacles):
        x, y, width, height = detection.bounding_box
        distance_cm = math.hypot(obstacle.x_cm, obstacle.y_cm)
        label = f"D={distance_cm:.1f} cm"
        baseline_y = min(frame.shape[0] - 6, max(18, y + height + 18))
        (text_width, text_height), _ = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.48, 1
        )
        text_x = max(2, min(frame.shape[1] - text_width - 2, x))
        cv2.rectangle(
            frame,
            (text_x - 2, baseline_y - text_height - 5),
            (text_x + text_width + 2, baseline_y + 3),
            (0, 0, 0),
            -1,
        )
        cv2.putText(
            frame,
            label,
            (text_x, baseline_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )


class ServoOutput:
    """Mantiene aislada la conversión de rueda física a servo."""

    def __init__(self, port: str, dry_run: bool, center: float, servo_deg_per_wheel_deg: float, motor_pwm: int, safe_min: float, safe_max: float):
        self.dry_run = dry_run
        self.center = center
        self.scale = servo_deg_per_wheel_deg
        self.motor_pwm = int(clamp(motor_pwm, 0, 255))
        self.safe_min = safe_min
        self.safe_max = safe_max
        self.last_servo = int(round(center))
        self.connection = None
        self.last_dry_run_message = None
        if not dry_run:
            if serial is None:
                raise RuntimeError("pyserial no está instalado")
            self.connection = serial.Serial(port, SERIAL_BAUDRATE, timeout=0)
            # Abrir el puerto suele reiniciar el Arduino. Esperar al firmware
            # evita perder el primer comando autónomo.
            time.sleep(ARDUINO_STARTUP_DELAY_S)
            self.connection.reset_input_buffer()
            self.connection.write(f"<0,{round(self.center)}>\n".encode("ascii"))

    def send(self, command: ControlCommand) -> None:
        servo = round(self.center - command.steering_angle_deg * self.scale)
        servo = int(clamp(servo, self.safe_min, self.safe_max))
        self.last_servo = servo
        if command.speed_cm_s == 0:
            motor_command = 0
        else:
            motor_command = self.motor_pwm if command.speed_cm_s > 0 else -self.motor_pwm
        message = f"<{motor_command},{servo}>\n"
        if self.dry_run:
            if message != self.last_dry_run_message:
                print(
                    f"TX {message.rstrip()} ruedas={command.steering_angle_deg:+.2f}°",
                    flush=True,
                )
                self.last_dry_run_message = message
        else:
            self.connection.write(message.encode("ascii"))

    def stop(self) -> None:
        self.send(ControlCommand(0.0, 0.0))
        if self.connection is not None:
            self.connection.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--camera", default="0")
    parser.add_argument(
        "--speed-cm-s",
        type=float,
        default=24.5,
        help="Velocidad física estimada; 120 cm en 4.9 s = 24.5 cm/s",
    )
    parser.add_argument(
        "--safety-margin-cm",
        type=float,
        default=15.0,
        help="Separación lateral adicional respecto al obstáculo",
    )
    parser.add_argument("--motor-pwm", type=int, default=120,
                        help="PWM enviado al Arduino cuando el planner ordena avance")
    parser.add_argument("--focal-length-px", type=float, default=None)
    parser.add_argument("--camera-height-cm", type=float, default=None)
    parser.add_argument("--camera-pitch-deg", type=float, default=None)
    parser.add_argument("--calibration-file", type=Path, default=DEFAULT_CALIBRATION_PATH)
    parser.add_argument("--obstacle-height-cm", type=float, default=10.0)
    parser.add_argument("--wall-height-cm", type=float, default=10.0)
    parser.add_argument("--physical-measurements", type=Path, default=DEFAULT_PHYSICAL_MEASUREMENTS_PATH)
    parser.add_argument(
        "--wall-thickness-cm",
        type=float,
        default=3.0,
        help="Espesor geométrico usado para la barrera del muro negro",
    )
    parser.add_argument("--servo-center", type=float, default=None)
    parser.add_argument(
        "--servo-deg-per-wheel-deg",
        type=float,
        default=0.63,
        help="Conversión calibrada: 13° de servo / 20.7° de rueda",
    )
    parser.add_argument("--stream-host", default="0.0.0.0")
    parser.add_argument("--stream-port", type=int, default=8000)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--start", action="store_true", help="Permite enviar comandos al carro")
    parser.add_argument("--no-display", action="store_true")
    return parser.parse_args()


@dataclass
class KeyboardControl:
    """Control de arranque y aceleración sin quitar el giro al planner."""

    autonomous: bool = False
    manual_speed_until: float = 0.0
    manual_speed_sign: float = 1.0
    buffer: bytes = b""

    def poll(self, now: float) -> bool:
        """Lee Enter/flechas; devuelve True cuando se solicita salir."""
        if not sys.stdin.isatty():
            return False
        readable, _, _ = select.select([sys.stdin], [], [], 0)
        if readable:
            self.buffer += sys.stdin.buffer.read(1)
        while self.buffer:
            if self.buffer.startswith((b"\n", b"\r")):
                self.autonomous = True
                self.buffer = self.buffer[1:]
                print("Enter recibido: avance autónomo habilitado.", flush=True)
            elif self.buffer.startswith(b"\x1b[A"):
                self.manual_speed_sign = 1.0
                self.manual_speed_until = now + 0.35
                self.buffer = self.buffer[3:]
            elif self.buffer.startswith(b"\x1b[B"):
                self.manual_speed_sign = -1.0
                self.manual_speed_until = now + 0.35
                self.buffer = self.buffer[3:]
            elif self.buffer.startswith(b"\x1b") and len(self.buffer) < 3:
                break
            elif self.buffer[:1] in (b"q", b"Q", b"\x03"):
                return True
            else:
                self.buffer = self.buffer[1:]
        return False

    def command(self, plan: PlannerResult, speed_cm_s: float, now: float) -> ControlCommand:
        if self.autonomous:
            return plan.command
        if now <= self.manual_speed_until:
            return ControlCommand(
                self.manual_speed_sign * speed_cm_s,
                plan.command.steering_angle_deg,
            )
        return ControlCommand(0.0, 0.0)


def run() -> None:
    # Se carga solo al iniciar el modo cámara. El algoritmo geométrico puede
    # probarse en un equipo sin OpenCV ni hardware de cámara.
    from camera import (
        TUNING_PATH,
        FrameStore,
        LatestCameraFrame,
        TuningState,
        VisionConfig,
        analyze_frame,
        draw_result,
        start_stream_server,
    )
    from camera_calibration import load_config, set_capture_focus

    args = parse_args()
    if not args.dry_run and not args.start:
        raise SystemExit("Usa --start para permitir movimiento o --dry-run para probar sin motores")
    calibration = None
    physical_data = load_physical_measurements(args.physical_measurements)
    if args.calibration_file.exists():
        calibration, _ = load_config(args.calibration_file)
        print(
            f"Calibración cargada: {args.calibration_file} "
            f"height={calibration.height_cm:.2f} cm "
            f"pitch={calibration.pitch_deg:.2f}° "
            f"focal_x={calibration.focal_x_px:.2f} px "
            f"focal_y={calibration.focal_y_px:.2f} px "
            f"center=({calibration.center_x_px:.2f},{calibration.center_y_px:.2f})"
        )
    effective_height = (
        args.camera_height_cm
        if args.camera_height_cm is not None
        else float(physical_data["measurements"].get("camera_height_cm", 8.8))
    )
    effective_pitch = (
        args.camera_pitch_deg
        if args.camera_pitch_deg is not None
        else calibration.pitch_deg if calibration is not None else None
    )
    effective_focal_y = (
        args.focal_length_px
        if args.focal_length_px is not None
        else calibration.focal_y_px if calibration is not None else 300.0
    )
    effective_focal_x = calibration.focal_x_px if calibration is not None else effective_focal_y
    effective_center_x = calibration.center_x_px if calibration is not None else CAMERA_WIDTH / 2
    effective_center_y = calibration.center_y_px if calibration is not None else CAMERA_HEIGHT / 2
    camera_source = int(args.camera) if args.camera.isdigit() else args.camera
    physics = VehiclePhysics.from_measurements(
        physical_data,
        fixed_speed_cm_s=args.speed_cm_s,
        safety_margin_cm=args.safety_margin_cm,
    )
    physics = VehiclePhysics(
        **{**physics.__dict__, "camera_height_cm": effective_height}
    )
    planner = GeometricPlanner(physics)
    servo_center = physics.servo_center_deg if args.servo_center is None else args.servo_center
    output = ServoOutput(
        args.port, args.dry_run, servo_center + physics.servo_offset_deg,
        args.servo_deg_per_wheel_deg, args.motor_pwm,
        physics.servo_safe_min_deg, physics.servo_safe_max_deg,
    )
    config = VisionConfig(
        camera_index=camera_source,
        safe_distance_mm=round(physics.vision_safe_distance_cm * 10),
    )
    capture = LatestCameraFrame(camera_source, CAMERA_WIDTH, CAMERA_HEIGHT)
    if calibration is not None and calibration.focus_value is not None:
        applied_focus = set_capture_focus(capture.capture, calibration.focus_value)
        if applied_focus is None:
            print("Advertencia: la cámara no permitió aplicar el enfoque guardado")
    frame_store = FrameStore()
    tuning = TuningState(TUNING_PATH)
    server = start_stream_server(args.stream_host, args.stream_port, frame_store, tuning)
    print(f"Cámara lista. Video: http://pirobot.local:{server.server_port}")
    print("Planner geométrico activo. Ctrl+C para detener.")
    keyboard = KeyboardControl(autonomous=not args.start)
    terminal_settings = None
    if args.start and sys.stdin.isatty():
        terminal_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        print("Enter = autónomo | ↑/↓ = avance/reversa manual con giro autónomo | Q = salir", flush=True)
    last_command = None
    last_target_id = None
    last_steering_log_at = 0.0
    try:
        while True:
            now = time.monotonic()
            if args.start and keyboard.poll(now):
                break
            frame = capture.read_latest()
            if frame is None:
                time.sleep(0.01)
                continue
            result = analyze_frame(frame, config, tuning.snapshot())
            obstacles = tuple(
                detection_to_obstacle(
                    item, frame.shape[1], frame.shape[0], effective_focal_y,
                    args.obstacle_height_cm, physics.camera_height_cm, effective_pitch,
                    effective_focal_x, effective_center_x, effective_center_y,
                )
                for item in result.obstacles
            )
            walls = tuple(
                detection_to_wall(
                    item, frame.shape[1], frame.shape[0], effective_focal_y,
                    args.wall_height_cm, physics.camera_height_cm, effective_pitch,
                    effective_focal_x, effective_center_x, effective_center_y,
                    args.wall_thickness_cm,
                )
                for item in result.black_walls
            )
            plan = planner.plan(
                PlannerInput(
                    VehicleState(steering_deg=planner.last_commanded_steering_deg),
                    obstacles,
                    walls,
                )
            )
            command = keyboard.command(plan, physics.fixed_speed_cm_s, now)
            planner.last_commanded_steering_deg = command.steering_angle_deg
            output.send(command)
            if plan.target_id is None and abs(command.steering_angle_deg) > 0.1:
                decision = "contra_giro_reincorporacion"
            elif plan.target_id is None:
                decision = "avance_recto_sin_obstaculo"
            elif plan.command.speed_cm_s <= 0:
                decision = "parada_sin_trayectoria_segura"
            else:
                decision = "giro_calculado"
            should_log_steering = (
                command.speed_cm_s > 0
                and now - last_steering_log_at >= 0.20
            )
            if (
                command != last_command
                or plan.target_id != last_target_id
                or should_log_steering
            ):
                print(
                    f"objetivo={plan.target_id} lado={plan.pass_side} "
                    f"steering={command.steering_angle_deg:+.2f}° "
                    f"servo={output.last_servo}° "
                    f"predicciones={len(plan.predictions)} decision={decision}",
                    flush=True,
                )
                if command.speed_cm_s > 0:
                    last_steering_log_at = now
            last_target_id = plan.target_id
            last_command = command
            annotated = draw_result(frame, result, config, tuning.snapshot(), calibration)
            draw_planner_predictions(annotated, plan, physics, calibration)
            draw_obstacle_distances(annotated, result.obstacles, obstacles)
            frame_store.update(
                frame,
                annotated,
                result.wall_mask,
                result.line_mask,
            )
            if cv2 is not None and not args.no_display and os.environ.get("DISPLAY"):
                cv2.imshow("Vision autonoma", annotated)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            time.sleep(COMMAND_PERIOD_S)
    except KeyboardInterrupt:
        pass
    finally:
        output.stop()
        capture.close()
        server.shutdown()
        if cv2 is not None:
            cv2.destroyAllWindows()
        if terminal_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal_settings)


if __name__ == "__main__":
    run()
