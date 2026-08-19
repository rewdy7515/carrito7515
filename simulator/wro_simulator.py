"""Simulador 2D aislado del robot físico para WRO Future Engineers 2026.

Las coordenadas internas son centímetros. No hay acceso a GPIO, serial, cámara
ni código de control del robot.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from datetime import datetime, timezone
from dataclasses import dataclass, field
from pathlib import Path

import pygame

try:
    from geometric_planner import (
        ControlCommand,
        PlannerState,
        TrackDirection,
        VehicleGeometry,
        VehicleState,
        polygon_distance as planner_polygon_distance,
        polygons_intersect as planner_polygons_intersect,
        rectangle_polygon as planner_rectangle_polygon,
        vehicle_step,
    )
except ImportError:  # Permite importar como paquete desde el runner de pruebas.
    from simulator.geometric_planner import (
        ControlCommand,
        PlannerState,
        TrackDirection,
        VehicleGeometry,
        VehicleState,
        polygon_distance as planner_polygon_distance,
        polygons_intersect as planner_polygons_intersect,
        rectangle_polygon as planner_rectangle_polygon,
        vehicle_step,
    )

try:
    from simulator_adapter import AvoidState, SimulatorAutonomousAdapter
except ImportError:
    from simulator.simulator_adapter import AvoidState, SimulatorAutonomousAdapter

try:
    from planner_rules import FIXED_RULES
    from planner_tuning import PlannerTuning, load_planner_tuning
    from scenario import default_scenario
except ImportError:
    from simulator.planner_rules import FIXED_RULES
    from simulator.planner_tuning import PlannerTuning, load_planner_tuning
    from simulator.scenario import default_scenario

try:
    from track_config import (
        INNER_WALL as TRACK_INNER_WALL,
        OUTER_WALL as TRACK_OUTER_WALL,
        SIGN_SEAT_CENTERS as TRACK_SIGN_SEAT_CENTERS,
        START_ZONE as TRACK_START_ZONE,
        route_centerline as shared_route_centerline,
        start_zone_contains,
        straight_sequence,
    )
except ImportError:
    from simulator.track_config import (
        INNER_WALL as TRACK_INNER_WALL,
        OUTER_WALL as TRACK_OUTER_WALL,
        SIGN_SEAT_CENTERS as TRACK_SIGN_SEAT_CENTERS,
        START_ZONE as TRACK_START_ZONE,
        route_centerline as shared_route_centerline,
        start_zone_contains,
        straight_sequence,
    )


TRACK_CM = TRACK_OUTER_WALL[2]
# Los picos de las líneas son las cuatro esquinas del muro interno.
INNER_WALL_X_CM, INNER_WALL_Y_CM = TRACK_INNER_WALL[:2]
INNER_WALL_CM = TRACK_INNER_WALL[2]
# El panel gráfico de 800 mm queda dentro del muro; no representa su colisión.
CENTRAL_PANEL_CM = 80.0
CENTRAL_PANEL_X_CM = (TRACK_CM - CENTRAL_PANEL_CM) / 2
CENTRAL_PANEL_Y_CM = (TRACK_CM - CENTRAL_PANEL_CM) / 2
# Zona fija de salida: recta inferior, cuadro izquierdo de 500 x 400 mm.
START_ZONE_X_CM, START_ZONE_Y_CM, START_ZONE_WIDTH_CM, START_ZONE_HEIGHT_CM = TRACK_START_ZONE
CAR_LENGTH_CM = FIXED_RULES.vehicle_length_cm
CAR_WIDTH_CM = FIXED_RULES.vehicle_width_cm
WHEELBASE_CM = FIXED_RULES.wheelbase_cm
OBSTACLE_SIZE_CM = FIXED_RULES.default_obstacle_width_cm
SIGN_SEAT_SIZE_CM = 5.0
SIGN_EVALUATION_DIAMETER_CM = 8.5
LINE_WIDTH_CM = 2.0
GUIDE_STROKE_CM = 0.3
DEFAULT_MAX_STEERING_DEG = FIXED_RULES.maximum_physical_steering_deg
AUTONOMOUS_TURN_REFERENCE_DEG = FIXED_RULES.maximum_physical_steering_deg
AUTONOMOUS_TURN_REFERENCE_SAMPLE = 5
LINE_TURN_DURATION_S = 3.0
LINE_TURN_COOLDOWN_S = 1.25
LINE_ALIGNMENT_TOLERANCE_DEG = 8.0
SERVO_LOGICAL_CENTER_DEG = FIXED_RULES.servo_logical_center_deg
SERVO_SAFE_MIN_DEG = FIXED_RULES.servo_safe_min_deg
SERVO_SAFE_MAX_DEG = FIXED_RULES.servo_safe_max_deg
CAMERA_DIAGONAL_FOV_DEG = 78.0
CAMERA_ASPECT_RATIO = (16, 9)
DEFAULT_HORIZONTAL_FOV_DEG = FIXED_RULES.horizontal_fov_deg
SCALE = 2.0
PANEL_WIDTH = 360
WINDOW_SIZE = (int(TRACK_CM * SCALE) + PANEL_WIDTH, int(TRACK_CM * SCALE))
CALIBRATION_FILE = Path(__file__).resolve().parents[1] / "config" / "simulator_steering_calibration.json"
MANUAL_RUNS_DIR = Path(__file__).resolve().parents[1] / "config" / "simulator_manual_runs"
REFERENCE_ROUTE_SAMPLE = 10

# Centros (cm) transcritos del plano de la pista aportado por el equipo.
# Los pilares se colocan centrados en uno de estos asientos de 50 x 50 mm.
SIGN_SEAT_CENTERS = TRACK_SIGN_SEAT_CENTERS

LINE_PEAKS = {
    "top_left": (100.0, 100.0), "top_right": (200.0, 100.0),
    "bottom_left": (100.0, 200.0), "bottom_right": (200.0, 200.0),
}
LINE_OFFSET_CM = 100.0 * math.tan(math.radians(30))
# Segmentos con el color que una cámara/detector de líneas puede identificar.
TRACK_LINE_SEGMENTS = (
    ("blue", (100.0 - LINE_OFFSET_CM, 0.0), LINE_PEAKS["top_left"]),
    ("orange", (0.0, 100.0 - LINE_OFFSET_CM), LINE_PEAKS["top_left"]),
    ("orange", (200.0 + LINE_OFFSET_CM, 0.0), LINE_PEAKS["top_right"]),
    ("blue", (300.0, 100.0 - LINE_OFFSET_CM), LINE_PEAKS["top_right"]),
    ("blue", (0.0, 200.0 + LINE_OFFSET_CM), LINE_PEAKS["bottom_left"]),
    ("orange", (100.0 - LINE_OFFSET_CM, 300.0), LINE_PEAKS["bottom_left"]),
    ("blue", (200.0 + LINE_OFFSET_CM, 300.0), LINE_PEAKS["bottom_right"]),
    ("orange", (300.0, 200.0 + LINE_OFFSET_CM), LINE_PEAKS["bottom_right"]),
)


STRAIGHT_SEQUENCE = {
    TrackDirection.CLOCKWISE: straight_sequence(True),
    TrackDirection.COUNTERCLOCKWISE: straight_sequence(False),
}


def direction_from_first_line(color: str) -> TrackDirection:
    if color == "orange":
        return TrackDirection.CLOCKWISE
    if color == "blue":
        return TrackDirection.COUNTERCLOCKWISE
    raise ValueError("El color inicial debe ser orange o blue")


@dataclass
class Obstacle:
    x: float
    y: float
    color: str
    passed: bool = False


@dataclass
class Vehicle:
    x: float = START_ZONE_X_CM + START_ZONE_WIDTH_CM / 2
    y: float = START_ZONE_Y_CM + START_ZONE_HEIGHT_CM / 2
    heading: float = math.pi  # Sale hacia la esquina inferior izquierda.
    steering_deg: float = 0.0
    speed_cm_s: float = 0.0
    acceleration_cm_s2: float = 0.0
    target_steering_deg: float = 0.0
    target_speed_cm_s: float = 0.0
    path: list[tuple[float, float]] = field(default_factory=list)

    def corners(self) -> list[tuple[float, float]]:
        forward = (math.cos(self.heading), math.sin(self.heading))
        right = (-math.sin(self.heading), math.cos(self.heading))
        half_l, half_w = CAR_LENGTH_CM / 2, CAR_WIDTH_CM / 2
        return [
            (self.x + sx * half_l * forward[0] + sy * half_w * right[0],
             self.y + sx * half_l * forward[1] + sy * half_w * right[1])
            for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1))
        ]

    def rear_center(self) -> tuple[float, float]:
        return (
            self.x - math.cos(self.heading) * CAR_LENGTH_CM / 2,
            self.y - math.sin(self.heading) * CAR_LENGTH_CM / 2,
        )

    def radius_cm(self) -> float | None:
        return VehicleGeometry().turning_radius_cm(self.steering_deg)

    def step(self, dt: float) -> None:
        state = vehicle_step(
            VehicleState(
                self.x,
                self.y,
                self.heading,
                self.speed_cm_s,
                0.0,
                self.steering_deg,
            ),
            ControlCommand(self.target_speed_cm_s, self.target_steering_deg),
            dt,
            VehicleGeometry(),
        )
        self.x, self.y = state.x_cm, state.y_cm
        self.heading = state.heading_rad
        self.speed_cm_s = state.speed_cm_s
        self.acceleration_cm_s2 = state.acceleration_cm_s2
        self.steering_deg = state.steering_angle_deg
        self.path.append((self.x, self.y))
        if len(self.path) > 900:
            self.path.pop(0)


def project(axis: tuple[float, float], polygon: list[tuple[float, float]]) -> tuple[float, float]:
    values = [axis[0] * point[0] + axis[1] * point[1] for point in polygon]
    return min(values), max(values)


def polygons_intersect(a: list[tuple[float, float]], b: list[tuple[float, float]]) -> bool:
    for polygon in (a, b):
        for index, point in enumerate(polygon):
            other = polygon[(index + 1) % len(polygon)]
            axis = (-(other[1] - point[1]), other[0] - point[0])
            a_min, a_max = project(axis, a)
            b_min, b_max = project(axis, b)
            if a_max < b_min or b_max < a_min:
                return False
    return True


def rectangle(x: float, y: float, width: float, height: float) -> list[tuple[float, float]]:
    return [(x, y), (x + width, y), (x + width, y + height), (x, y + height)]


def distance_to_segment(point: tuple[float, float], start: tuple[float, float], end: tuple[float, float]) -> float:
    """Distancia mínima entre un punto y un segmento de línea."""
    px, py = point
    sx, sy = start
    ex, ey = end
    dx, dy = ex - sx, ey - sy
    length_squared = dx * dx + dy * dy
    if length_squared == 0:
        return math.hypot(px - sx, py - sy)
    fraction = max(0.0, min(1.0, ((px - sx) * dx + (py - sy) * dy) / length_squared))
    return math.hypot(px - (sx + fraction * dx), py - (sy + fraction * dy))


def detect_first_line(vehicle: Vehicle, detection_distance_cm: float = 12.0) -> str | None:
    """Devuelve el color de la primera línea alcanzada por el frente del carro."""
    front = (
        vehicle.x + math.cos(vehicle.heading) * CAR_LENGTH_CM / 2,
        vehicle.y + math.sin(vehicle.heading) * CAR_LENGTH_CM / 2,
    )
    candidates = [(distance_to_segment(front, start, end), color)
                  for color, start, end in TRACK_LINE_SEGMENTS]
    distance, color = min(candidates, default=(math.inf, None))
    return color if distance <= detection_distance_cm else None


def has_collision(vehicle: Vehicle, obstacles: list[Obstacle],
                  ignored_obstacles: set[int] | None = None) -> bool:
    body = VehicleGeometry().footprint(
        VehicleState(vehicle.x, vehicle.y, vehicle.heading, vehicle.speed_cm_s, 0.0, vehicle.steering_deg)
    )
    # Outer walls are a geometric boundary because WRO does not specify thickness.
    if any(x <= 0 or x >= TRACK_CM or y <= 0 or y >= TRACK_CM for x, y in body):
        return True
    # Los picos de las diagonales fijan las esquinas del muro interno de 1000 mm.
    inner = planner_rectangle_polygon((INNER_WALL_X_CM, INNER_WALL_Y_CM, INNER_WALL_CM, INNER_WALL_CM))
    if planner_polygons_intersect(body, inner):
        return True
    ignored_obstacles = ignored_obstacles or set()
    return any(index not in ignored_obstacles and planner_polygons_intersect(
        body, planner_rectangle_polygon((o.x - OBSTACLE_SIZE_CM / 2,
                                          o.y - OBSTACLE_SIZE_CM / 2,
                                          OBSTACLE_SIZE_CM, OBSTACLE_SIZE_CM)))
        for index, o in enumerate(obstacles, start=1))


def is_in_start_zone(vehicle: Vehicle) -> bool:
    return start_zone_contains(vehicle.x, vehicle.y)


def is_start_straight_obstacle(obstacle: Obstacle) -> bool:
    """Obstacles placed on the lower starting straight."""
    return 80.0 <= obstacle.x <= 220.0 and obstacle.y >= 230.0


def track_straight_sector(vehicle: Vehicle) -> str | None:
    """Return the straight currently occupied around the central square."""
    if 80.0 <= vehicle.x <= 220.0 and vehicle.y < 90.0:
        return "top"
    if vehicle.x > 210.0 and 80.0 <= vehicle.y <= 220.0:
        return "right"
    if 80.0 <= vehicle.x <= 220.0 and vehicle.y > 210.0:
        return "bottom"
    if vehicle.x < 90.0 and 80.0 <= vehicle.y <= 220.0:
        return "left"
    return None


def obstacle_straight_sector(obstacle: Obstacle) -> str | None:
    """Associate a valid sign seat with the straight that contains it."""
    if obstacle.y <= 70.0:
        return "top"
    if obstacle.y >= 230.0:
        return "bottom"
    if obstacle.x <= 70.0:
        return "left"
    if obstacle.x >= 230.0:
        return "right"
    return None


def in_fov(vehicle: Vehicle, obstacle: Obstacle, fov_deg: float,
           range_cm: float = math.inf) -> bool:
    dx, dy = obstacle.x - vehicle.x, obstacle.y - vehicle.y
    distance = math.hypot(dx, dy)
    if distance > range_cm:
        return False
    angle = math.atan2(dy, dx) - vehicle.heading
    angle = (angle + math.pi) % (2 * math.pi) - math.pi
    return abs(angle) <= math.radians(fov_deg) / 2


def world_to_screen(point: tuple[float, float]) -> tuple[int, int]:
    return int(point[0] * SCALE), int(point[1] * SCALE)
def movement_record(
    simulation_time_s: float,
    vehicle: Vehicle,
    obstacles: list[Obstacle],
    state: AvoidState,
    track_direction: TrackDirection | None = None,
    planning_phase: str = "SENSE_DIRECTION",
    controller: SimulatorAutonomousAdapter | None = None,
    automatic: bool = False,
    servo_command_deg: float | None = None,
    lap_completed: bool = False,
    straight_progress: int = 0,
    direction_locked: bool = False,
) -> dict[str, object]:
    """Registro neutral para calibración manual y evaluación del planner."""
    geometry = controller.planner.geometry if controller else VehicleGeometry()
    body = geometry.footprint(VehicleState(vehicle.x, vehicle.y, vehicle.heading,
        vehicle.speed_cm_s, vehicle.acceleration_cm_s2, vehicle.steering_deg, simulation_time_s))
    inner = planner_rectangle_polygon((INNER_WALL_X_CM, INNER_WALL_Y_CM, INNER_WALL_CM, INNER_WALL_CM))
    wall_clearance = min(min(min(x, TRACK_CM-x, y, TRACK_CM-y) for x,y in body),
                         planner_polygon_distance(body, inner))
    obstacle_distances = {str(index): planner_polygon_distance(body,
        planner_rectangle_polygon((obstacle.x-OBSTACLE_SIZE_CM/2, obstacle.y-OBSTACLE_SIZE_CM/2,
                                   OBSTACLE_SIZE_CM, OBSTACLE_SIZE_CM)))
        for index, obstacle in enumerate(obstacles, 1) if not obstacle.passed}
    obstacle_clearance = min(obstacle_distances.values(), default=math.inf)
    result = controller.latest_result if controller and automatic else None
    diagnostics = result.diagnostics if result else None
    selected_angle = diagnostics.selected_angle_deg if diagnostics else vehicle.target_steering_deg
    selected_speed = diagnostics.selected_speed_cm_s if diagnostics else vehicle.target_speed_cm_s
    return {
        "time_s": round(simulation_time_s,4), "mode": "AUTO" if automatic else "MANUAL",
        "vehicle_cm_deg": {"x":round(vehicle.x,3),"y":round(vehicle.y,3),
            "heading":round(math.degrees(vehicle.heading),3),"steering":round(vehicle.steering_deg,3),
            "target_steering":round(vehicle.target_steering_deg,3),"speed_cm_s":round(vehicle.speed_cm_s,3),
            "target_speed_cm_s":round(vehicle.target_speed_cm_s,3),"acceleration_cm_s2":round(vehicle.acceleration_cm_s2,3)},
        "angles_deg": {"wheel_actual":round(vehicle.steering_deg,3),"wheel_target":round(vehicle.target_steering_deg,3),
            "planner_selected":round(selected_angle,3),"servo_command":servo_command_deg},
        "distances_cm": {"minimum":None if not math.isfinite(min(wall_clearance,obstacle_clearance)) else round(min(wall_clearance,obstacle_clearance),3),
            "minimum_obstacle":None if not math.isfinite(obstacle_clearance) else round(obstacle_clearance,3),
            "minimum_wall":round(wall_clearance,3),"minimum_corridor":round(wall_clearance,3),
            "obstacles":{key:round(value,3) for key,value in obstacle_distances.items()}},
        "decision": {"source":"PLANNER" if automatic else "MANUAL_INPUT",
            "state":result.state.value if result else state.name,"phase":planning_phase,
            "reason":result.reason if result else "keyboard_control",
            "active_target_id":None,"selected_pass_side":None,
            "selected_angle_deg":round(selected_angle,3),"selected_speed_cm_s":round(selected_speed,3),
            "selected_radius_cm":None if geometry.turning_radius_cm(selected_angle) is None else round(abs(geometry.turning_radius_cm(selected_angle) or 0),3),
            "physical_collision":has_collision(vehicle,obstacles),"memory_states":{},
            "candidates_generated":diagnostics.candidates_generated if diagnostics else 0,
            "candidates_evaluated":diagnostics.candidates_evaluated if diagnostics else 0,
            "calculation_time_ms":diagnostics.calculation_time_ms if diagnostics else 0.0},
        "obstacles_cm":[{"id":i,"x":round(o.x,3),"y":round(o.y,3),"color":o.color,"passed":o.passed}
                        for i,o in enumerate(obstacles,1)],
        "avoidance_state":state.name,"track_direction":track_direction.value if track_direction else "pending_first_line",
        "planning_phase":planning_phase,"route":{"lap_completed":lap_completed,"straight_progress":straight_progress,
        "direction_locked":direction_locked},
    }


def save_calibration_sample(
    vehicle: Vehicle,
    servo_command_deg: float | None,
    max_steering_deg: float,
    fov_deg: float,
    samples: list[dict[str, object]],
    movement_history: list[dict[str, object]],
) -> None:
    """Persist a steering sample in a file separate from hardware control code."""
    samples.append({
        "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
        "servo_command_deg": servo_command_deg,
        "wheel_steering_deg": round(vehicle.steering_deg, 3),
        "turning_radius_cm": None if vehicle.radius_cm() is None else round(abs(vehicle.radius_cm() or 0), 3),
        "vehicle_pose_cm_deg": {"x": round(vehicle.x, 3), "y": round(vehicle.y, 3),
                                "heading": round(math.degrees(vehicle.heading), 3)},
        "trajectory": movement_history,
    })
    payload = {
        "units": {"distance": "cm", "angle": "degrees"},
        "purpose": "Simulator samples for steering calibration; validate physically before sending to the robot.",
        "wheel_steering_limits_deg": {
            "left": -max_steering_deg,
            "right": max_steering_deg,
            "source": "Confirmed by the team; wheel angle, not servo command.",
        },
        "servo_command_limits_deg": {
            "minimum": SERVO_SAFE_MIN_DEG,
            "center_logical": SERVO_LOGICAL_CENTER_DEG,
            "maximum": SERVO_SAFE_MAX_DEG,
            "source": "Current firmware configuration; verify the mechanical limits physically.",
        },
        "camera": {
            "model": "Logitech C922 Pro Stream Webcam",
            "diagonal_fov_deg": CAMERA_DIAGONAL_FOV_DEG,
            "aspect_ratio": "16:9",
            "horizontal_fov_deg": fov_deg,
            "source": "78 degree diagonal specification supplied by the team; horizontal FOV derived geometrically.",
        },
        "samples": samples,
    }
    existing_status = load_calibration_status()
    if existing_status:
        payload["calibration_status"] = existing_status
    CALIBRATION_FILE.parent.mkdir(parents=True, exist_ok=True)
    CALIBRATION_FILE.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def save_manual_recording(
    movement_history: list[dict[str, object]],
    vehicle: Vehicle,
    obstacle_mode: int,
    fixed_speed_cm_s: float,
) -> tuple[Path, Path]:
    """Export the current manual run as detailed JSON and calibration CSV."""
    MANUAL_RUNS_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    base = MANUAL_RUNS_DIR / f"manual_run_{stamp}"
    metadata = {
        "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
        "mode": "MANUAL",
        "scenario": obstacle_mode,
        "units": {"distance": "cm", "angle": "degrees", "time": "seconds"},
        "fixed_speed_target_cm_s": fixed_speed_cm_s,
        "frame_count": len(movement_history),
        "final_vehicle": {
            "x_cm": round(vehicle.x, 3),
            "y_cm": round(vehicle.y, 3),
            "heading_deg": round(math.degrees(vehicle.heading), 3),
        },
        "records": movement_history,
        "extraction": {
            "angles": "records[*].angles_deg",
            "distances": "records[*].distances_cm",
            "decisions": "records[*].decision",
        },
    }
    json_path = base.with_suffix(".json")
    csv_path = base.with_suffix(".csv")
    json_path.write_text(json.dumps(metadata, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    columns = [
        "time_s", "mode", "x_cm", "y_cm", "heading_deg", "wheel_actual_deg",
        "wheel_target_deg", "planner_selected_deg", "servo_command_deg", "speed_cm_s",
        "target_speed_cm_s", "acceleration_cm_s2", "selected_radius_cm",
        "target_distance_cm", "required_lateral_shift_cm", "current_wall_clearance_cm",
        "front_clearance_cm", "recovery_phase", "recovery_side",
        "forward_projection_cm", "obstacle_in_forward_projection",
        "recovery_reverse_distance_cm", "recovery_advance_distance_cm",
        "recovery_reverse_steering_deg",
        "safety_limit_triggered",
        "minimum_distance_cm", "minimum_obstacle_cm", "minimum_wall_cm",
        "physical_collision", "planner_state", "phase", "active_target_id",
        "selected_pass_side", "decision_reason", "candidates_evaluated",
        "calculation_time_ms", "lap_completed", "straight_progress", "track_direction",
        "memory_states_json", "obstacle_distances_json",
    ]
    with csv_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        for record in movement_history:
            pose = record.get("vehicle_cm_deg", {})
            angles = record.get("angles_deg", {})
            distances = record.get("distances_cm", {})
            decision = record.get("decision", {})
            route = record.get("route", {})
            writer.writerow({
                "time_s": record.get("time_s"),
                "mode": record.get("mode"),
                "x_cm": pose.get("x"), "y_cm": pose.get("y"),
                "heading_deg": pose.get("heading"),
                "wheel_actual_deg": angles.get("wheel_actual"),
                "wheel_target_deg": angles.get("wheel_target"),
                "planner_selected_deg": angles.get("planner_selected"),
                "servo_command_deg": angles.get("servo_command"),
                "speed_cm_s": pose.get("speed_cm_s"),
                "target_speed_cm_s": pose.get("target_speed_cm_s"),
                "acceleration_cm_s2": pose.get("acceleration_cm_s2"),
                "selected_radius_cm": decision.get("selected_radius_cm"),
                "target_distance_cm": decision.get("target_distance_cm"),
                "required_lateral_shift_cm": decision.get("required_lateral_shift_cm"),
                "current_wall_clearance_cm": decision.get("current_wall_clearance_cm"),
                "front_clearance_cm": decision.get("front_clearance_cm"),
                "forward_projection_cm": decision.get("forward_projection_cm"),
                "obstacle_in_forward_projection": decision.get("obstacle_in_forward_projection"),
                "recovery_phase": decision.get("recovery_phase"),
                "recovery_side": decision.get("recovery_side"),
                "recovery_reverse_distance_cm": decision.get("recovery_reverse_distance_cm"),
                "recovery_reverse_steering_deg": decision.get("recovery_reverse_steering_deg"),
                "recovery_advance_distance_cm": decision.get("recovery_advance_distance_cm"),
                "safety_limit_triggered": decision.get("safety_limit_triggered"),
                "minimum_distance_cm": distances.get("minimum"),
                "minimum_obstacle_cm": distances.get("minimum_obstacle"),
                "minimum_wall_cm": distances.get("minimum_wall"),
                "physical_collision": decision.get("physical_collision"),
                "planner_state": decision.get("state"),
                "phase": decision.get("phase"),
                "active_target_id": decision.get("active_target_id"),
                "selected_pass_side": decision.get("selected_pass_side"),
                "decision_reason": decision.get("reason"),
                "candidates_evaluated": decision.get("candidates_evaluated"),
                "calculation_time_ms": decision.get("calculation_time_ms"),
                "lap_completed": route.get("lap_completed"),
                "straight_progress": route.get("straight_progress"),
                "track_direction": record.get("track_direction"),
                "memory_states_json": json.dumps(decision.get("memory_states", {}), ensure_ascii=False),
                "obstacle_distances_json": json.dumps(distances.get("obstacles", {}), ensure_ascii=False),
            })
    return json_path, csv_path


def load_calibration_samples() -> list[dict[str, object]]:
    """Keep prior samples when the simulator is opened again."""
    try:
        payload = json.loads(CALIBRATION_FILE.read_text(encoding="utf-8"))
        samples = payload.get("samples", [])
        return samples if isinstance(samples, list) else []
    except (FileNotFoundError, json.JSONDecodeError):
        return []


def load_reference_route() -> list[tuple[float, float]]:
    """Load the latest complete manual lap as the autonomous route guide."""
    try:
        payload = json.loads(CALIBRATION_FILE.read_text(encoding="utf-8"))
        sample = payload.get("samples", [])[REFERENCE_ROUTE_SAMPLE - 1]
        raw = sample.get("trajectory", [])
        points = [(float(item["vehicle_cm_deg"]["x"]),
                   float(item["vehicle_cm_deg"]["y"])) for item in raw]
        # The manual sample is recorded at 60 Hz; reduce it without changing
        # the measured route shape or its start/end points.
        return points[::20] + (points[-1:] if points and points[-1] != points[::20][-1] else [])
    except (OSError, ValueError, IndexError, KeyError, TypeError):
        return []


def orient_reference_route(route: list[tuple[float, float]], direction: TrackDirection) -> list[tuple[float, float]]:
    """Keep the same start while selecting clockwise or counterclockwise order."""
    if direction is TrackDirection.CLOCKWISE:
        return list(route)
    if not route:
        return []
    start = route[0]
    return [start] + list(reversed(route[1:]))


def centerline_route(direction: TrackDirection) -> list[tuple[float, float]]:
    """Adaptador Pygame para la ruta compartida con el runner headless."""
    return list(shared_route_centerline(direction is TrackDirection.CLOCKWISE))


def load_calibration_status() -> dict[str, object]:
    """Preserve the physical-validation status when later samples are added."""
    try:
        payload = json.loads(CALIBRATION_FILE.read_text(encoding="utf-8"))
        status = payload.get("calibration_status", {})
        return status if isinstance(status, dict) else {}
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


def draw_track(surface: pygame.Surface) -> None:
    surface.fill((246, 246, 246))
    outer = pygame.Rect(0, 0, int(TRACK_CM * SCALE), int(TRACK_CM * SCALE))
    inner = pygame.Rect(int(INNER_WALL_X_CM * SCALE), int(INNER_WALL_Y_CM * SCALE),
                        int(INNER_WALL_CM * SCALE), int(INNER_WALL_CM * SCALE))
    panel = pygame.Rect(int(CENTRAL_PANEL_X_CM * SCALE), int(CENTRAL_PANEL_Y_CM * SCALE),
                        int(CENTRAL_PANEL_CM * SCALE), int(CENTRAL_PANEL_CM * SCALE))
    start_zone = pygame.Rect(int(START_ZONE_X_CM * SCALE), int(START_ZONE_Y_CM * SCALE),
                             int(START_ZONE_WIDTH_CM * SCALE), int(START_ZONE_HEIGHT_CM * SCALE))
    pygame.draw.rect(surface, (20, 20, 20), outer, width=5)
    pygame.draw.rect(surface, (20, 20, 20), inner, width=5)
    # The supplied drawing marks both perimeters with a 3 mm yellow guide.
    guide_width = max(1, round(GUIDE_STROKE_CM * SCALE))
    pygame.draw.rect(surface, (204, 255, 0), outer, width=guide_width)
    pygame.draw.rect(surface, (204, 255, 0), inner, width=guide_width)
    pygame.draw.rect(surface, (30, 48, 100), panel)
    pygame.draw.rect(surface, (180, 180, 180), start_zone, width=1)
    # Los picos de cada diagonal coinciden con las esquinas del muro interno.
    blue, orange = (0, 51, 255), (255, 102, 0)
    w = max(1, int(LINE_WIDTH_CM * SCALE))
    for color_name, start, end in TRACK_LINE_SEGMENTS:
        color = orange if color_name == "orange" else blue
        pygame.draw.line(surface, color, world_to_screen(start), world_to_screen(end), w)
    draw_sign_seats(surface)


def draw_sign_seats(surface: pygame.Surface) -> None:
    """Draw every valid 50 mm sign seat and its 85 mm evaluation circle."""
    seat_color, circle_color = (190, 190, 190), (204, 255, 0)
    seat_px = max(1, round(SIGN_SEAT_SIZE_CM * SCALE))
    circle_px = max(1, round(SIGN_EVALUATION_DIAMETER_CM * SCALE / 2))
    for center in SIGN_SEAT_CENTERS:
        point = world_to_screen(center)
        pygame.draw.circle(surface, circle_color, point, circle_px, width=1)
        rect = pygame.Rect(point[0] - seat_px // 2, point[1] - seat_px // 2, seat_px, seat_px)
        pygame.draw.rect(surface, seat_color, rect, width=1)


DISPLAY_SAFETY_MARGIN_SCALE = 0.5


def expanded_vehicle_corners(vehicle: Vehicle, margin_cm: float) -> list[tuple[float, float]]:
    """Devuelve la zona segura rectangular alrededor del carro completo.

    El margen se suma en los dos ejes locales del vehículo. Por tanto, la
    zona gira con el carro y conserva su forma rectangular: no es un radio
    circular aproximado del centro.
    """
    forward = (math.cos(vehicle.heading), math.sin(vehicle.heading))
    right = (-forward[1], forward[0])
    half_length = CAR_LENGTH_CM / 2 + max(0.0, margin_cm)
    half_width = CAR_WIDTH_CM / 2 + max(0.0, margin_cm)
    return [
        (
            vehicle.x + sx * half_length * forward[0] + sy * half_width * right[0],
            vehicle.y + sx * half_length * forward[1] + sy * half_width * right[1],
        )
        for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1))
    ]


def draw_vehicle(surface: pygame.Surface, vehicle: Vehicle, fov_deg: float, safety_margin_cm: float) -> None:
    corners = [world_to_screen(p) for p in vehicle.corners()]
    pygame.draw.polygon(surface, (65, 110, 230), corners)
    pygame.draw.polygon(surface, (10, 30, 70), corners, 2)
    nose = world_to_screen((vehicle.x + math.cos(vehicle.heading) * CAR_LENGTH_CM / 2,
                            vehicle.y + math.sin(vehicle.heading) * CAR_LENGTH_CM / 2))
    pygame.draw.line(surface, (255, 255, 255), world_to_screen((vehicle.x, vehicle.y)), nose, 2)
    # Las dos ruedas delanteras muestran la geometría Ackermann efectiva.
    geometry = VehicleGeometry()
    left_wheel_deg, right_wheel_deg = geometry.ackermann_wheel_angles_deg(vehicle.steering_deg)
    forward = (math.cos(vehicle.heading), math.sin(vehicle.heading))
    right = (-forward[1], forward[0])
    front_axle = (
        vehicle.x + forward[0] * geometry.front_axle_offset_cm,
        vehicle.y + forward[1] * geometry.front_axle_offset_cm,
    )
    for lateral_cm, wheel_deg in ((-geometry.front_track_cm / 2, left_wheel_deg),
                                  (geometry.front_track_cm / 2, right_wheel_deg)):
        center = (front_axle[0] + right[0] * lateral_cm, front_axle[1] + right[1] * lateral_cm)
        wheel_heading = vehicle.heading + math.radians(wheel_deg)
        wheel_half_cm = geometry.wheel_diameter_cm / 2
        start = (center[0] - math.cos(wheel_heading) * wheel_half_cm,
                 center[1] - math.sin(wheel_heading) * wheel_half_cm)
        end = (center[0] + math.cos(wheel_heading) * wheel_half_cm,
               center[1] + math.sin(wheel_heading) * wheel_half_cm)
        pygame.draw.line(surface, (245, 245, 245), world_to_screen(start), world_to_screen(end), 2)
    # FOV wedge and a rectangular safety zone around the complete body.
    origin = world_to_screen((vehicle.x, vehicle.y))
    reach = 100.0
    angles = [vehicle.heading + math.radians(-fov_deg / 2 + fov_deg * i / 20) for i in range(21)]
    wedge = [origin] + [world_to_screen((vehicle.x + reach * math.cos(a), vehicle.y + reach * math.sin(a))) for a in angles]
    overlay = pygame.Surface(surface.get_size(), pygame.SRCALPHA)
    pygame.draw.polygon(overlay, (90, 190, 255, 45), wedge)
    projection_start = (
        vehicle.x + math.cos(vehicle.heading) * CAR_LENGTH_CM / 2,
        vehicle.y + math.sin(vehicle.heading) * CAR_LENGTH_CM / 2,
    )
    projection_end = (
        projection_start[0] + math.cos(vehicle.heading) * FIXED_RULES.forward_projection_cm,
        projection_start[1] + math.sin(vehicle.heading) * FIXED_RULES.forward_projection_cm,
    )
    pygame.draw.line(
        overlay,
        (255, 235, 40, 210),
        world_to_screen(projection_start),
        world_to_screen(projection_end),
        max(1, round(SCALE)),
    )
    # La zona mostrada se reduce a la mitad para no ocultar la carrocería y la
    # pista. El cálculo del planner conserva el margen obligatorio completo.
    display_margin_cm = safety_margin_cm * DISPLAY_SAFETY_MARGIN_SCALE
    safe_zone = [world_to_screen(point) for point in expanded_vehicle_corners(vehicle, display_margin_cm)]
    pygame.draw.polygon(overlay, (255, 215, 0, 40), safe_zone)
    pygame.draw.polygon(overlay, (255, 215, 0, 150), safe_zone, width=max(1, round(SCALE)))
    surface.blit(overlay, (0, 0))


def draw_obstacles(surface: pygame.Surface, obstacles: list[Obstacle]) -> None:
    for obstacle in obstacles:
        color = (238, 39, 55) if obstacle.color == "red" else (68, 214, 44)
        x = int((obstacle.x - OBSTACLE_SIZE_CM / 2) * SCALE)
        y = int((obstacle.y - OBSTACLE_SIZE_CM / 2) * SCALE)
        size = int(OBSTACLE_SIZE_CM * SCALE)
        pygame.draw.rect(surface, (160, 160, 160) if obstacle.passed else color, (x, y, size, size))


def draw_planner_overlay(surface: pygame.Surface, controller: SimulatorAutonomousAdapter) -> None:
    """Dibuja exactamente las trayectorias simuladas y validadas."""
    result = controller.latest_result
    if result is None:
        return
    for candidate in result.candidates:
        if len(candidate.points) < 2:
            continue
        if result.best_candidate is candidate:
            color = (55, 220, 75)
            width = 3
        elif candidate.safe:
            color = (245, 205, 45)
            width = 1
        else:
            color = (235, 55, 55)
            width = 1
        validated = [world_to_screen(point) for point in candidate.points]
        pygame.draw.lines(surface, color, False, validated, width)


def draw_panel_section(
    surface: pygame.Surface,
    font: pygame.font.Font,
    x: int,
    y: int,
    width: int,
    title: str,
    rows: list[tuple[str, str]],
    accent: tuple[int, int, int],
) -> int:
    """Draw a compact status card and return the next available y position."""
    row_height = 14
    header_height = 19
    height = header_height + row_height * len(rows) + 5
    card = pygame.Rect(x, y, width, height)
    pygame.draw.rect(surface, (36, 41, 51), card, border_radius=4)
    pygame.draw.rect(surface, accent, (x, y, width, 3), border_radius=2)
    title_surface = font.render(title.upper(), True, accent)
    surface.blit(title_surface, (x + 8, y + 5))
    for index, (label, value) in enumerate(rows):
        text = f"{label}: {value}"
        # Keep long memory/file values inside the fixed-width panel.
        text = text[:52]
        row_surface = font.render(text, True, (232, 235, 240))
        surface.blit(row_surface, (x + 8, y + header_height + index * row_height))
    return y + height + 5


def default_obstacles() -> list[Obstacle]:
    """Convert the shared scenario representation to the Pygame model."""
    return [
        Obstacle(item.x_cm, item.y_cm, item.color)
        for item in default_scenario(2).objects
    ]


def seat_slot(seat: tuple[float, float]) -> tuple[str, int]:
    """Return side and one of its 0/500/1000 mm longitudinal slots."""
    x, y = seat
    if y in (40.0, 60.0):
        return "top", int((x - 100.0) / 50.0)
    if y in (240.0, 260.0):
        return "bottom", int((x - 100.0) / 50.0)
    if x in (40.0, 60.0):
        return "left", int((y - 100.0) / 50.0)
    if x in (240.0, 260.0):
        return "right", int((y - 100.0) / 50.0)
    raise ValueError(f"Asiento fuera del patrón: {seat}")


def add_obstacle_ahead(vehicle: Vehicle, obstacles: list[Obstacle], color: str) -> None:
    """Place a manual obstacle in the closest unoccupied valid seat ahead."""
    x = vehicle.x + math.cos(vehicle.heading) * 55
    y = vehicle.y + math.sin(vehicle.heading) * 55
    occupied = {(obstacle.x, obstacle.y) for obstacle in obstacles}
    occupied_slots = {seat_slot((obstacle.x, obstacle.y)) for obstacle in obstacles}
    choices = [seat for seat in SIGN_SEAT_CENTERS
               if seat not in occupied and seat_slot(seat) not in occupied_slots]
    if choices:
        seat = min(choices, key=lambda point: math.hypot(point[0] - x, point[1] - y))
        obstacles.append(Obstacle(*seat, color))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--max-steering-deg", type=float, default=None,
                        help="Sobrescribe el máximo ángulo REAL del tuning JSON.")
    parser.add_argument("--fov-deg", type=float, default=DEFAULT_HORIZONTAL_FOV_DEG,
                        help=f"FOV horizontal de cámara (por defecto: {DEFAULT_HORIZONTAL_FOV_DEG} grados derivados de 78 grados diagonales a 16:9).")
    parser.add_argument("--safety-margin-cm", type=float, default=None,
                        help="Sobrescribe el margen obligatorio del tuning JSON.")
    parser.add_argument("--desired-clearance-cm", type=float, default=None,
                        help="Sobrescribe el margen deseado del tuning JSON.")
    parser.add_argument("--fixed-speed-cm-s", type=float, default=None,
                        help="Sobrescribe la velocidad fija del tuning JSON.")
    parser.add_argument("--replanning-period-s", type=float, default=None,
                        help="Sobrescribe el periodo del tuning JSON.")
    parser.add_argument("--max-candidates", type=int, default=None,
                        help="Sobrescribe el presupuesto de candidatos del tuning JSON.")
    parser.add_argument("--max-planning-time-ms", type=float, default=None,
                        help="Sobrescribe el presupuesto de tiempo del tuning JSON.")
    parser.add_argument("--preview-horizon-s", type=float, default=None,
                        help="Sobrescribe el horizonte visual del tuning JSON.")
    parser.add_argument("--planning-horizon-s", type=float, default=None,
                        help="Sobrescribe el horizonte de validación del tuning JSON.")
    parser.add_argument("--max-steering-rate-deg-s", type=float, default=None,
                        help="Sobrescribe la velocidad máxima de steering del tuning JSON.")
    parser.add_argument("--max-acceleration-cm-s2", type=float, default=None,
                        help="Sobrescribe la aceleración del tuning JSON.")
    parser.add_argument("--max-deceleration-cm-s2", type=float, default=None,
                        help="Sobrescribe la desaceleración del tuning JSON.")
    parser.add_argument("--planner-config", type=Path, default=None,
                        help="Archivo JSON de PlannerTuning.")
    parser.add_argument("--servo-command-deg", type=float, default=SERVO_LOGICAL_CENTER_DEG,
                        help="Comando lógico inicial del servo (por defecto: centro lógico 92 grados).")
    args = parser.parse_args()
    tuning_values = load_planner_tuning(args.planner_config)
    override_names = (
        "max_steering_deg", "safety_margin_cm", "desired_clearance_cm",
        "fixed_speed_cm_s", "replanning_period_s", "max_candidates",
        "max_planning_time_ms", "preview_horizon_s", "planning_horizon_s",
        "max_steering_rate_deg_s", "max_acceleration_cm_s2", "max_deceleration_cm_s2",
    )
    override_fields = {
        name: getattr(args, name)
        for name in override_names
        if getattr(args, name) is not None
    }
    tuning = tuning_values.with_overrides(**override_fields)
    args.max_steering_deg = tuning.max_steering_deg
    args.safety_margin_cm = tuning.mandatory_clearance_cm
    args.desired_clearance_cm = tuning.desired_clearance_cm
    args.fixed_speed_cm_s = tuning.fixed_speed_cm_s
    args.replanning_period_s = tuning.replanning_period_s
    args.max_candidates = tuning.max_candidates
    args.max_planning_time_ms = tuning.max_planning_time_ms
    args.preview_horizon_s = tuning.preview_horizon_s
    args.planning_horizon_s = tuning.planning_horizon_s
    args.max_steering_rate_deg_s = tuning.max_steering_rate_deg_s
    args.max_acceleration_cm_s2 = tuning.max_acceleration_cm_s2
    args.max_deceleration_cm_s2 = tuning.max_deceleration_cm_s2
    if (not 0 < args.max_steering_deg < 89 or not 0 < args.fov_deg <= 180
            or args.safety_margin_cm < 0 or args.desired_clearance_cm < args.safety_margin_cm
            or args.fixed_speed_cm_s <= 0 or args.fixed_speed_cm_s > 32.0
            or args.replanning_period_s <= 0 or args.max_candidates <= 0
            or args.max_planning_time_ms <= 0 or args.preview_horizon_s <= 0
            or args.planning_horizon_s <= 0 or args.max_steering_rate_deg_s <= 0
            or args.max_acceleration_cm_s2 <= 0 or args.max_deceleration_cm_s2 <= 0):
        parser.error("Parámetros geométricos o presupuestos inválidos.")
    if not SERVO_SAFE_MIN_DEG <= args.servo_command_deg <= SERVO_SAFE_MAX_DEG:
        parser.error(f"El comando de servo debe estar entre {SERVO_SAFE_MIN_DEG:.0f} y {SERVO_SAFE_MAX_DEG:.0f} grados.")

    pygame.init()
    screen = pygame.display.set_mode(WINDOW_SIZE)
    pygame.display.set_caption("WRO 2026 - Simulador 2D aislado")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("monospace", 16)
    panel_font = pygame.font.SysFont("monospace", 12)
    vehicle = Vehicle()
    obstacle_mode = 1
    obstacles: list[Obstacle] = []
    # No route is active until the first colored line fixes the sense of travel.
    controller = SimulatorAutonomousAdapter(
        tuning, args.fov_deg, TRACK_OUTER_WALL, TRACK_INNER_WALL, OBSTACLE_SIZE_CM
    )
    automatic, running = False, True
    lap_completed = False
    visited_straights: set[str] = set()
    track_direction: TrackDirection | None = None
    direction_locked = False
    straight_progress = 0
    last_straight: str | None = None
    finish_armed = False
    servo_command_deg = args.servo_command_deg
    calibration_samples = load_calibration_samples()
    simulation_time_s = 0.0
    last_manual_run: tuple[Path, Path] | None = None

    def current_record() -> dict[str, object]:
        return movement_record(
            simulation_time_s, vehicle, obstacles, controller.state, track_direction,
            controller.planning_phase, controller, automatic, servo_command_deg,
            lap_completed, straight_progress, direction_locked,
        )

    movement_history = [current_record()]

    while running:
        dt = min(clock.tick(60) / 1000, 0.05)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_a:
                    # Starting automatic mode is always a fresh trial. Before
                    # this reset, pressing A after a saved/manual/finished run
                    # could leave the vehicle at an old pose with a completed
                    # route and look as if A did nothing.
                    if automatic:
                        automatic = False
                        vehicle.target_speed_cm_s = vehicle.target_steering_deg = 0.0
                    else:
                        automatic = True
                        vehicle = Vehicle()
                        obstacles = [] if obstacle_mode == 1 else default_obstacles()
                        controller.reset()
                        controller.route_points = []
                        lap_completed = False
                        visited_straights.clear()
                        simulation_time_s = 0.0
                        track_direction = None
                        direction_locked = False
                        straight_progress = 0
                        last_straight = None
                        finish_armed = False
                        movement_history = [current_record()]
                elif event.key == pygame.K_r:
                    vehicle = Vehicle()
                    obstacles = [] if obstacle_mode == 1 else default_obstacles()
                    controller.reset()
                    controller.route_points = []
                    lap_completed = False
                    visited_straights.clear()
                    simulation_time_s = 0.0
                    track_direction = None
                    direction_locked = False
                    straight_progress = 0
                    last_straight = None
                    finish_armed = False
                    movement_history = [current_record()]
                elif event.key in (pygame.K_1, pygame.K_2):
                    obstacle_mode = 1 if event.key == pygame.K_1 else 2
                    vehicle = Vehicle()
                    obstacles = [] if obstacle_mode == 1 else default_obstacles()
                    controller.reset()
                    controller.route_points = []
                    lap_completed = False
                    visited_straights.clear()
                    track_direction = None
                    direction_locked = False
                    straight_progress = 0
                    last_straight = None
                    simulation_time_s = 0.0
                    finish_armed = False
                    movement_history = [current_record()]
                elif event.key == pygame.K_o:
                    if not direction_locked:
                        track_direction = direction_from_first_line("orange")
                        direction_locked = True
                        controller.reset()
                        controller.set_track_direction(track_direction, centerline_route(track_direction))
                elif event.key == pygame.K_b:
                    if not direction_locked:
                        track_direction = direction_from_first_line("blue")
                        direction_locked = True
                        # La opción azul representa la salida equivalente con
                        # el frente ya orientado a la derecha. No es una
                        # inversión de marcha una vez iniciado el recorrido.
                        vehicle.heading = 0.0
                        controller.reset()
                        controller.set_track_direction(track_direction, centerline_route(track_direction))
                elif event.key in (pygame.K_LEFTBRACKET, pygame.K_COMMA):
                    servo_command_deg = max(SERVO_SAFE_MIN_DEG, servo_command_deg - 1)
                elif event.key in (pygame.K_RIGHTBRACKET, pygame.K_PERIOD):
                    servo_command_deg = min(SERVO_SAFE_MAX_DEG, servo_command_deg + 1)
                elif event.key == pygame.K_c:
                    save_calibration_sample(vehicle, servo_command_deg, args.max_steering_deg, args.fov_deg,
                                            calibration_samples, movement_history)
                elif event.key == pygame.K_s:
                    last_manual_run = save_manual_recording(
                        movement_history, vehicle, obstacle_mode, controller.planner.geometry.fixed_speed_cm_s
                    )

        if automatic:
            if track_direction is None:
                # Desde la zona de salida avanza recto hasta que el frente
                # reconoce la primera línea; su color fija el sentido.
                if controller.state is AvoidState.FOLLOW:
                    vehicle.target_speed_cm_s = 20.0
                    vehicle.target_steering_deg = 0.0
                    first_line = detect_first_line(vehicle)
                    if first_line is not None:
                        track_direction = direction_from_first_line(first_line)
                        direction_locked = True
                        if track_direction is TrackDirection.COUNTERCLOCKWISE:
                            vehicle.heading = 0.0
                        controller.reset()
                        controller.set_track_direction(track_direction, centerline_route(track_direction))
                    else:
                        # Obstacles in the fixed start zone must still be
                        # considered before the first colored line appears.
                        controller.update(vehicle, obstacles, dt)
                        # There is no route until the first line fixes the
                        # direction.  The controller intentionally leaves the
                        # vehicle stopped when route_points is empty, so keep
                        # this initial sensing phase moving straight.
                        vehicle.target_speed_cm_s = 20.0
                        vehicle.target_steering_deg = 0.0
                else:
                    controller.update(vehicle, obstacles, dt)
            else:
                controller.update(vehicle, obstacles, dt)
        else:
            keys = pygame.key.get_pressed()
            # El retroceso manual es una orden directa del operador; no forma
            # parte de la lógica automática del planner.
            vehicle.target_speed_cm_s = (
                32.0 if keys[pygame.K_UP]
                else -18.0 if keys[pygame.K_DOWN]
                else 0.0
            )
            vehicle.target_steering_deg = ((args.max_steering_deg if keys[pygame.K_RIGHT] else 0.0)
                                           + (-args.max_steering_deg if keys[pygame.K_LEFT] else 0.0))
        previous = (vehicle.x, vehicle.y, vehicle.heading, vehicle.speed_cm_s, vehicle.steering_deg)
        vehicle.step(dt)
        # Obstacles on the starting straight are active from the first frame.
        ignored_obstacles = set()
        if has_collision(vehicle, obstacles, ignored_obstacles):
            vehicle.x, vehicle.y, vehicle.heading, vehicle.speed_cm_s, vehicle.steering_deg = previous
            vehicle.target_speed_cm_s = vehicle.target_steering_deg = 0.0
            if automatic:
                controller.state = AvoidState.EMERGENCY_STOP
        if automatic and track_direction is not None:
            sector = track_straight_sector(vehicle)
            if sector is not None and sector != last_straight:
                sequence = STRAIGHT_SEQUENCE[track_direction]
                expected = sequence[min(straight_progress, len(sequence) - 1)]
                if sector == expected:
                    straight_progress += 1
                    visited_straights.add(sector)
                    finish_armed = straight_progress >= len(sequence)
                last_straight = sector
        if (automatic and track_direction is not None and finish_armed
                and simulation_time_s > 20.0
                and is_in_start_zone(vehicle)
                and controller.aligned_after_line(vehicle)):
            lap_completed = True
            automatic = False
            vehicle.target_speed_cm_s = vehicle.target_steering_deg = 0.0
            vehicle.speed_cm_s = vehicle.steering_deg = 0.0
        simulation_time_s += dt
        movement_history.append(current_record())

        draw_track(screen)
        if len(vehicle.path) > 1:
            pygame.draw.lines(screen, (90, 90, 90), False, [world_to_screen(p) for p in vehicle.path], 1)
        draw_obstacles(screen, obstacles)
        draw_planner_overlay(screen, controller)
        draw_vehicle(screen, vehicle, args.fov_deg, args.safety_margin_cm)
        panel = pygame.Rect(int(TRACK_CM * SCALE), 0, PANEL_WIDTH, int(TRACK_CM * SCALE))
        pygame.draw.rect(screen, (28, 32, 40), panel)
        panel_x = int(TRACK_CM * SCALE) + 10
        panel_width = PANEL_WIDTH - 20
        result = controller.latest_result
        diagnostics = result.diagnostics if result else None

        def metric(value: float | None) -> str:
            return "-" if value is None or not math.isfinite(value) else f"{value:.1f}"

        selected_radius = diagnostics.selected_radius_cm if diagnostics else vehicle.radius_cm()
        selected_angle = diagnostics.selected_angle_deg if diagnostics else vehicle.target_steering_deg
        memory = "solo objetos visibles"
        saved_name = last_manual_run[0].stem if last_manual_run else "-"

        title = panel_font.render("WRO 2026  |  SIMULATOR 2D", True, (245, 245, 245))
        screen.blit(title, (panel_x + 8, 6))
        mode_text = panel_font.render(
            f"{'AUTO' if automatic else 'MANUAL'}  |  t={simulation_time_s:05.1f}s",
            True, (100, 230, 130) if automatic else (255, 210, 90),
        )
        screen.blit(mode_text, (panel_x + panel_width - mode_text.get_width() - 8, 6))
        y = 30
        y = draw_panel_section(screen, panel_font, panel_x, y, panel_width, "Run / route", [
            ("scenario", f"{obstacle_mode} | {'clear' if obstacle_mode == 1 else 'obstacles'}"),
            ("lap / straights", f"{'DONE' if lap_completed else 'run'} | {min(straight_progress, 4)}/4"),
            ("direction", track_direction.value if track_direction else "searching line"),
            ("direction lock", "YES" if direction_locked else "NO"),
        ], (80, 170, 255))
        y = draw_panel_section(screen, panel_font, panel_x, y, panel_width, "Planner decision", [
            ("FSM", controller.planner_state),
            ("phase", controller.planning_phase),
            ("candidate", diagnostics.selected_candidate_id if diagnostics and diagnostics.selected_candidate_id else "-"),
            ("commitment", diagnostics.committed_candidate_id if diagnostics and diagnostics.committed_candidate_id else "-"),
            ("angle / radius", f"{selected_angle:+.1f} deg / {metric(abs(selected_radius) if selected_radius is not None else None)} cm"),
            ("candidates / time", f"{diagnostics.candidates_evaluated if diagnostics else 0} / {diagnostics.calculation_time_ms if diagnostics else 0:.2f} ms"),
            ("reason", result.reason if result and result.reason else "continuous / manual"),
        ], (150, 205, 255) if not result or result.state is not PlannerState.NO_SAFE_TRAJECTORY else (255, 150, 80))
        left_wheel_deg, right_wheel_deg = controller.planner.geometry.ackermann_wheel_angles_deg(vehicle.steering_deg)
        target_left_wheel_deg, target_right_wheel_deg = controller.planner.geometry.ackermann_wheel_angles_deg(vehicle.target_steering_deg)
        y = draw_panel_section(screen, panel_font, panel_x, y, panel_width, "Vehicle / command", [
            ("chassis steer", f"{vehicle.steering_deg:+.1f} -> {vehicle.target_steering_deg:+.1f} deg"),
            ("Ackermann L / R", f"{left_wheel_deg:+.1f} / {right_wheel_deg:+.1f} deg"),
            ("target L / R", f"{target_left_wheel_deg:+.1f} / {target_right_wheel_deg:+.1f} deg"),
            ("servo logical", f"{servo_command_deg:.0f} deg"),
            ("speed / target", f"{vehicle.speed_cm_s:.1f} / {controller.planner.geometry.fixed_speed_cm_s:.1f} cm/s"),
            ("acceleration", f"{vehicle.acceleration_cm_s2:+.1f} cm/s2"),
            ("steering rate", f"{controller.planner.geometry.max_steering_rate_deg_s:.0f} deg/s"),
        ], (120, 230, 160))
        y = draw_panel_section(screen, panel_font, panel_x, y, panel_width, "Safety / sensing", [
            ("min obstacle", f"{metric(diagnostics.minimum_obstacle_clearance_cm if diagnostics else None)} cm"),
            ("min wall / total", f"{metric(diagnostics.minimum_wall_clearance_cm if diagnostics else None)} / {metric(diagnostics.minimum_clearance_cm if diagnostics else None)} cm"),
            ("clearance req / desired", f"{args.safety_margin_cm:.1f} / {args.desired_clearance_cm:.1f} cm"),
            ("straight projection", "clear" if diagnostics and diagnostics.straight_projection_safe else "blocked"),
            ("FOV / replan", f"{args.fov_deg:.1f} deg / {args.replanning_period_s:.2f} s"),
            ("lines / valid", f"{controller.planner.preview_horizon_s:.1f} / {controller.planner.planning_horizon_s:.1f} s"),
        ], (255, 205, 80))
        y = draw_panel_section(screen, panel_font, panel_x, y, panel_width, "Manual recording", [
            ("control", "arrows | S save JSON+CSV"),
            ("frames / samples", f"{len(movement_history)} / {len(calibration_samples)}"),
            ("last file", saved_name),
        ], (245, 180, 75))
        draw_panel_section(screen, panel_font, panel_x, y, panel_width, "Perception input", [
            ("states", memory),
            ("C", "save steering calibration sample"),
            ("R / A / 1-2", "reset / auto / scenario"),
        ], (190, 160, 255))
        pygame.display.flip()
    pygame.quit()


if __name__ == "__main__":
    main()
