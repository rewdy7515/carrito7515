"""Planificador geométrico puro para un carro Ackermann; sin Pygame ni hardware."""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field, replace
from enum import Enum
from typing import Sequence

try:
    from planner_rules import FIXED_RULES
    from planner_tuning import PlannerTuning
    from trajectory_scorer import score_trajectory_breakdown
    from local_frame import LocalSide, footprint_side, right_vector
except ImportError:
    from simulator.planner_rules import FIXED_RULES
    from simulator.planner_tuning import PlannerTuning
    from simulator.trajectory_scorer import score_trajectory_breakdown
    from simulator.local_frame import LocalSide, footprint_side, right_vector

Point = tuple[float, float]
Polygon = tuple[Point, ...]
NUMERICAL_CLEARANCE_TOLERANCE_CM = 0.15


@dataclass(frozen=True)
class ClearanceResult:
    """Resultado único de colisión y distancias para una pose."""

    collision: bool
    collision_type: str | None
    object_id: str | None
    min_clearance_cm: float
    obstacle_clearance_cm: float
    wall_clearance_cm: float


@dataclass(frozen=True)
class GeometryCache:
    """Geometrías estáticas compartidas por todos los candidatos de un ciclo."""

    obstacle_polygons: tuple[tuple[str, Polygon], ...]
    obstacle_geometry: tuple[tuple[str, Polygon, tuple[tuple[Point, Point], ...], tuple[Point, ...]], ...]
    wall_segments: tuple[tuple[str, Point, Point, float], ...]
    boundary_edges: tuple[tuple[Point, Point], ...]
    boundary_polygon: Polygon | None = None
    active_target_polygon: Polygon | None = None


class TrackDirection(str, Enum):
    CLOCKWISE = "CLOCKWISE"
    COUNTERCLOCKWISE = "COUNTERCLOCKWISE"


class PrimitiveType(str, Enum):
    STRAIGHT = "STRAIGHT"
    REVERSE = "REVERSE"
    ARC_LEFT = "ARC_LEFT"
    ARC_RIGHT = "ARC_RIGHT"


class TrajectoryProfile(str, Enum):
    CONSERVATIVE = "CONSERVATIVE"
    NOMINAL = "NOMINAL"
    TIGHT = "TIGHT"


class PlannerState(str, Enum):
    FOLLOW = "FOLLOW"
    MANEUVERING = "MANEUVERING"
    NO_SAFE_TRAJECTORY = "NO_SAFE_TRAJECTORY"


@dataclass(frozen=True)
class VehicleGeometry:
    """Medidas tomadas únicamente de config/physical_measurements.json."""
    length_cm: float = FIXED_RULES.vehicle_length_cm
    width_cm: float = FIXED_RULES.vehicle_width_cm
    wheelbase_cm: float = FIXED_RULES.wheelbase_cm
    front_axle_offset_cm: float = FIXED_RULES.front_axle_offset_cm
    rear_axle_offset_cm: float = FIXED_RULES.rear_axle_offset_cm
    front_track_cm: float = FIXED_RULES.front_wheel_center_track_cm
    wheel_width_cm: float = FIXED_RULES.wheel_width_cm
    wheel_diameter_cm: float = FIXED_RULES.wheel_diameter_cm
    minimum_right_radius_cm: float = FIXED_RULES.turn_radius_right_cm
    minimum_left_radius_cm: float = FIXED_RULES.turn_radius_left_cm
    right_turn_left_wheel_deg: float = FIXED_RULES.right_turn_left_wheel_deg
    right_turn_right_wheel_deg: float = FIXED_RULES.right_turn_right_wheel_deg
    left_turn_left_wheel_deg: float = FIXED_RULES.left_turn_left_wheel_deg
    left_turn_right_wheel_deg: float = FIXED_RULES.left_turn_right_wheel_deg
    max_speed_cm_s: float = FIXED_RULES.max_speed_cm_s
    fixed_speed_cm_s: float = FIXED_RULES.fixed_speed_cm_s
    max_acceleration_cm_s2: float = FIXED_RULES.max_acceleration_cm_s2
    max_deceleration_cm_s2: float = FIXED_RULES.max_deceleration_cm_s2
    max_steering_rate_deg_s: float = FIXED_RULES.max_steering_rate_deg_s

    @property
    def max_right_steering_deg(self) -> float:
        return math.degrees(math.atan(self.wheelbase_cm / self.minimum_right_radius_cm))

    @property
    def max_left_steering_deg(self) -> float:
        return math.degrees(math.atan(self.wheelbase_cm / self.minimum_left_radius_cm))

    @property
    def max_steering_deg(self) -> float:
        return max(self.max_left_steering_deg, self.max_right_steering_deg)

    def steering_limit_deg(self, right: bool) -> float:
        return self.max_right_steering_deg if right else self.max_left_steering_deg

    def clamp_steering(self, angle_deg: float) -> float:
        return clamp(angle_deg, -self.max_left_steering_deg, self.max_right_steering_deg)

    def wheel_angles_deg(self, steering_deg: float) -> tuple[float, float]:
        steering = self.clamp_steering(steering_deg)
        if abs(steering) < 1e-9:
            return 0.0, 0.0
        if steering > 0:
            scale = steering / self.max_right_steering_deg
            return self.right_turn_left_wheel_deg * scale, self.right_turn_right_wheel_deg * scale
        scale = -steering / self.max_left_steering_deg
        return -self.left_turn_left_wheel_deg * scale, -self.left_turn_right_wheel_deg * scale

    def ackermann_wheel_angles_deg(self, steering_deg: float) -> tuple[float, float]:
        return self.wheel_angles_deg(steering_deg)

    def turning_radius_cm(self, steering_deg: float) -> float | None:
        steering = self.clamp_steering(steering_deg)
        if abs(steering) < 1e-9:
            return None
        limit = self.steering_limit_deg(steering > 0)
        minimum = self.minimum_right_radius_cm if steering > 0 else self.minimum_left_radius_cm
        return math.copysign(minimum * limit / abs(steering), steering)

    def footprint(self, state: "VehicleState") -> Polygon:
        return rotated_rectangle((state.x_cm, state.y_cm), state.heading_rad, self.length_cm, self.width_cm)


@dataclass(frozen=True)
class VehicleState:
    x_cm: float
    y_cm: float
    heading_rad: float
    speed_cm_s: float = 0.0
    acceleration_cm_s2: float = 0.0
    steering_angle_deg: float = 0.0
    time_s: float = 0.0


@dataclass(frozen=True)
class VisibleObstacle:
    object_id: str
    x_cm: float
    y_cm: float
    width_cm: float
    length_cm: float
    color: str = "unknown"
    heading_rad: float = 0.0
    # El objeto permanece en la geometría de colisión después de superarlo,
    # pero ya no debe iniciar otra maniobra reglamentaria durante esa vuelta.
    already_passed: bool = False

    def polygon(self) -> Polygon:
        return rotated_rectangle((self.x_cm, self.y_cm), self.heading_rad, self.length_cm, self.width_cm)


@dataclass(frozen=True)
class VisibleWall:
    wall_id: str
    start: Point
    end: Point
    thickness_cm: float = 0.0


@dataclass(frozen=True)
class PlannerInput:
    vehicle_state: VehicleState
    visible_obstacles: tuple[VisibleObstacle, ...] = ()
    visible_walls: tuple[VisibleWall, ...] = ()
    drivable_boundary: Polygon | None = None
    track_direction: TrackDirection | None = None
    desired_heading_rad: float | None = None
    timestamp_s: float = 0.0
    # Ruta longitudinal ordenada por percepción/ruta. Es opcional para que
    # el planner también pueda funcionar solo con FOV y muros.
    route_centerline: tuple[Point, ...] = ()
    # Objetos detectados anteriormente que siguen siendo relevantes para la
    # geometría. ``visible_obstacles`` siempre tiene prioridad si se repite
    # el mismo object_id.
    tracked_obstacles: tuple[VisibleObstacle, ...] = ()
    # Identificador fijado por AutonomousController mientras el obstáculo no
    # haya sido confirmado como PASSED.
    active_target_id: str | None = None


@dataclass(frozen=True)
class MotionPrimitive:
    kind: PrimitiveType
    distance_cm: float
    steering_angle_deg: float
    target_speed_cm_s: float

    def __post_init__(self) -> None:
        if self.distance_cm <= 0:
            raise ValueError("distance_cm debe ser positivo")
        if self.kind is PrimitiveType.STRAIGHT and abs(self.steering_angle_deg) > 1e-9:
            raise ValueError("STRAIGHT requiere steering cero")
        if self.kind is PrimitiveType.ARC_LEFT and self.steering_angle_deg >= 0:
            raise ValueError("ARC_LEFT requiere steering negativo")
        if self.kind is PrimitiveType.ARC_RIGHT and self.steering_angle_deg <= 0:
            raise ValueError("ARC_RIGHT requiere steering positivo")


@dataclass(frozen=True)
class TrajectoryPoint:
    state: VehicleState
    primitive_index: int
    traveled_cm: float
    footprint: Polygon
    clearance: ClearanceResult | None = None


@dataclass
class CandidateTrajectory:
    candidate_id: str
    profile: TrajectoryProfile | None
    primitives: tuple[MotionPrimitive, ...]
    points: list[Point] = field(default_factory=list)
    trajectory_points: list[TrajectoryPoint] = field(default_factory=list)
    safe: bool = False
    physical_safe: bool = False
    physical_collision: bool = False
    minimum_clearance_cm: float = math.inf
    minimum_obstacle_clearance_cm: float = math.inf
    minimum_wall_clearance_cm: float = math.inf
    progress_cm: float = 0.0
    final_heading_error_deg: float = 0.0
    steering_effort: float = 0.0
    steering_changes: float = 0.0
    length_cm: float = 0.0
    correct_pass_side: bool = True
    wrong_pass_side: bool = False
    actual_pass_side: LocalSide | None = None
    current_pass_side_satisfied: bool = False
    future_pass_viable: bool = False
    target_passed: bool = False
    target_passed_correctly: bool = False
    initial_lateral_error_cm: float = 0.0
    final_lateral_error_cm: float = 0.0
    pass_progress_cm: float = 0.0
    horizon_cm: float = 0.0
    raw_score: float = -math.inf
    final_score: float = -math.inf
    score_components: dict[str, float] = field(default_factory=dict)
    pass_side_adjustment: float = 0.0
    beam_pruned: bool = False
    beam_pruned_reason: str | None = None
    rejection_reason: str | None = None
    diagnostic_rejection_reason: str | None = None
    collision_type: str | None = None
    collision_object_id: str | None = None
    target_obstacle_id: str | None = None
    desired_pass_side: LocalSide | None = None
    current_lateral_offset_cm: float | None = None
    target_lateral_offset_cm: float | None = None
    lateral_error_cm: float | None = None
    pass_side_feasible: bool | None = None
    recovery_probe_valid: bool = False
    score: float = -math.inf
    hard_failure: bool = False
    geometry_cache: GeometryCache | None = None
    final_state: VehicleState | None = None
    final_footprint: Polygon | None = None
    closest_target_footprint: Polygon | None = None
    closest_target_distance_cm: float = math.inf


@dataclass(frozen=True)
class ControlCommand:
    target_speed_cm_s: float
    steering_angle_deg: float


@dataclass(frozen=True)
class PassTarget:
    """Objetivo lateral reglamentario respecto a la recta del obstáculo.

    ``side`` expresa la regla RED/GREEN. ``target_lateral_offset_cm`` es la
    frontera lateral mínima que debe superar el centro del carro para dejar
    el clearance hard entre ambos footprints. No es un punto exacto.
    """

    obstacle_id: str
    side: LocalSide
    tangent_rad: float
    forward_vector: Point
    right_vector: Point
    target_lateral_offset_cm: float


@dataclass
class PlannerDiagnostics:
    calculation_time_ms: float = 0.0
    candidates_generated: int = 0
    candidates_evaluated: int = 0
    rejected_collision: int = 0
    rejected_clearance: int = 0
    rejected_kinematics: int = 0
    minimum_clearance_cm: float = math.inf
    minimum_obstacle_clearance_cm: float = math.inf
    minimum_wall_clearance_cm: float = math.inf
    selected_candidate_id: str | None = None
    committed_candidate_id: str | None = None
    selected_angle_deg: float = 0.0
    selected_speed_cm_s: float = 0.0
    selected_radius_cm: float | None = None
    straight_projection_safe: bool = False
    reason: str = ""
    budget_exhausted: bool = False
    budget_reason: str | None = None
    forward_candidates_generated: int = 0
    forward_candidates_valid: int = 0
    reverse_candidates_generated: int = 0
    reverse_candidates_valid: int = 0
    rejection_reasons: dict[str, int] = field(default_factory=dict)
    forward_rejections: dict[str, int] = field(default_factory=dict)
    reverse_rejections: dict[str, int] = field(default_factory=dict)
    candidate_diagnostics: list[dict[str, object]] = field(default_factory=list)
    representative_rejected_candidate: dict[str, object] | None = None
    reverse_recovery_attempted: bool = False
    reverse_distance_cm: float = 0.0
    target_obstacle_id: str | None = None
    desired_pass_side: str | None = None
    current_lateral_offset_cm: float | None = None
    target_lateral_offset_cm: float | None = None
    lateral_error_cm: float | None = None
    pass_side_feasible: bool | None = None
    active_target_id: str | None = None
    no_safe_reason: str | None = None
    no_safe_detail: str | None = None
    commitment_mode: str = "flexible"
    current_plan_score: float = -math.inf
    new_plan_score: float = -math.inf
    switch_margin: float = 0.0
    switched_plan: bool = False
    execution_horizon_cm: float = 0.0
    simulation_calls: int = 0
    segment_simulations: int = 0
    clearance_evaluations: int = 0
    fast_path: bool = False
    diagnostic_level: str = "full"
    committed_horizon_cm: float = 0.0
    new_horizon_cm: float = 0.0
    committed_future_pass_viable: bool | None = None
    new_future_pass_viable: bool | None = None
    switch_reason: str | None = None
    reverse_steering_deg: float = 0.0
    forward_after_reverse_candidate_id: str | None = None
    recovery_min_clearance_cm: float | None = None
    recovery_final_score: float | None = None


@dataclass
class PlannerResult:
    command: ControlCommand
    state: PlannerState
    candidates: list[CandidateTrajectory]
    best_candidate: CandidateTrajectory | None
    diagnostics: PlannerDiagnostics
    reason: str


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def rotated_rectangle(center: Point, heading: float, length: float, width: float) -> Polygon:
    forward, right = (math.cos(heading), math.sin(heading)), (-math.sin(heading), math.cos(heading))
    return tuple((center[0] + forward[0] * a + right[0] * b, center[1] + forward[1] * a + right[1] * b)
                 for a, b in ((length/2, width/2), (length/2, -width/2), (-length/2, -width/2), (-length/2, width/2)))


def rectangle_polygon(rect: tuple[float, float, float, float]) -> Polygon:
    x, y, w, h = rect
    return ((x, y), (x+w, y), (x+w, y+h), (x, y+h))


def _axes(polygon: Sequence[Point]) -> list[Point]:
    return [(-(b[1]-a[1]), b[0]-a[0]) for a, b in zip(polygon, polygon[1:] + polygon[:1])]


def _polygon_edges(polygon: Sequence[Point]) -> tuple[tuple[Point, Point], ...]:
    return tuple(zip(polygon, polygon[1:] + polygon[:1]))


def _polygon_distance_prepared(
    first: Sequence[Point], first_edges: Sequence[tuple[Point, Point]],
    first_axes: Sequence[Point], second: Sequence[Point],
    second_edges: Sequence[tuple[Point, Point]], second_axes: Sequence[Point],
) -> float:
    for axis in (*first_axes, *second_axes):
        first_projection = [point[0] * axis[0] + point[1] * axis[1] for point in first]
        second_projection = [point[0] * axis[0] + point[1] * axis[1] for point in second]
        if max(first_projection) < min(second_projection) or max(second_projection) < min(first_projection):
            return min(
                min(
                    point_segment_distance(point, start, end)
                    for point in first
                    for start, end in second_edges
                ),
                min(
                    point_segment_distance(point, start, end)
                    for point in second
                    for start, end in first_edges
                ),
            )
    return 0.0


def polygons_intersect(first: Sequence[Point], second: Sequence[Point]) -> bool:
    for axis in _axes(tuple(first)) + _axes(tuple(second)):
        f = [p[0]*axis[0] + p[1]*axis[1] for p in first]
        s = [p[0]*axis[0] + p[1]*axis[1] for p in second]
        if max(f) < min(s) or max(s) < min(f):
            return False
    return True


def point_segment_distance(point: Point, start: Point, end: Point) -> float:
    dx, dy = end[0]-start[0], end[1]-start[1]
    length2 = dx*dx + dy*dy
    if length2 == 0:
        return math.dist(point, start)
    t = clamp(((point[0]-start[0])*dx + (point[1]-start[1])*dy)/length2, 0, 1)
    return math.dist(point, (start[0]+t*dx, start[1]+t*dy))


def polygon_distance(first: Sequence[Point], second: Sequence[Point]) -> float:
    if polygons_intersect(first, second):
        return 0.0
    return min(point_segment_distance(p, a, b) for polygon, other in ((first, second), (second, first))
               for p in polygon for a, b in zip(other, other[1:] + other[:1]))


def polygon_segment_distance(polygon: Sequence[Point], start: Point, end: Point) -> float:
    if any(_segments_intersect(start,end,a,b) for a,b in zip(polygon,polygon[1:]+polygon[:1])):
        return 0.0
    return min(min(point_segment_distance(p, start, end) for p in polygon),
               min(point_segment_distance(p, a, b) for p in (start, end)
                   for a, b in zip(polygon, polygon[1:] + polygon[:1])))


def _segments_intersect(a:Point,b:Point,c:Point,d:Point)->bool:
    def side(p:Point,q:Point,r:Point)->float:
        return (q[0]-p[0])*(r[1]-p[1])-(q[1]-p[1])*(r[0]-p[0])
    ab_c,ab_d,cd_a,cd_b=side(a,b,c),side(a,b,d),side(c,d,a),side(c,d,b)
    boxes_overlap=(max(min(a[0],b[0]),min(c[0],d[0]))<=min(max(a[0],b[0]),max(c[0],d[0]))
                   and max(min(a[1],b[1]),min(c[1],d[1]))<=min(max(a[1],b[1]),max(c[1],d[1])))
    return boxes_overlap and ab_c*ab_d<=0 and cd_a*cd_b<=0


def point_in_polygon(point: Point, polygon: Sequence[Point]) -> bool:
    x, y, inside, previous = point[0], point[1], False, polygon[-1]
    for current in polygon:
        x1, y1 = previous
        x2, y2 = current
        if (y1 > y) != (y2 > y) and x < (x2-x1)*(y-y1)/(y2-y1)+x1:
            inside = not inside
        previous = current
    return inside


def vehicle_step(state: VehicleState, command: ControlCommand, dt: float, geometry: VehicleGeometry) -> VehicleState:
    steering = state.steering_angle_deg + clamp(geometry.clamp_steering(command.steering_angle_deg)-state.steering_angle_deg,
        -geometry.max_steering_rate_deg_s*dt, geometry.max_steering_rate_deg_s*dt)
    target = clamp(command.target_speed_cm_s, -geometry.max_speed_cm_s, geometry.max_speed_cm_s)
    rate = geometry.max_acceleration_cm_s2 if abs(target) > abs(state.speed_cm_s) else geometry.max_deceleration_cm_s2
    delta = clamp(target-state.speed_cm_s, -rate*dt, rate*dt)
    speed = state.speed_cm_s + delta
    radius = geometry.turning_radius_cm(steering)
    heading = normalize_angle(state.heading_rad + (0 if radius is None else speed/radius)*dt)
    return VehicleState(state.x_cm+speed*math.cos(heading)*dt, state.y_cm+speed*math.sin(heading)*dt,
                        heading, speed, delta/dt if dt else 0, steering, state.time_s+dt)


class GeometricPlanner:
    def __init__(self, geometry: VehicleGeometry | None = None, tuning: PlannerTuning | None = None) -> None:
        self.tuning = (tuning or PlannerTuning()).validate()
        self.geometry = geometry or VehicleGeometry()
        self._cycle_stats = {
            "simulation_calls": 0,
            "segment_simulations": 0,
            "clearance_evaluations": 0,
        }
        self._pruned_candidates: list[CandidateTrajectory] = []

    def _hard_clearance(self, axis: str) -> float:
        if self.tuning.disable_hard_safety_margins:
            return 0.0
        return {
            "front": FIXED_RULES.hard_front_clearance_cm,
            "side": FIXED_RULES.hard_side_clearance_cm,
            "rear": FIXED_RULES.hard_rear_clearance_cm,
        }[axis]

    @property
    def prediction_horizon_cm(self) -> float:
        return min(self.tuning.planning_horizon_cm, FIXED_RULES.perception_range_cm)

    @property
    def planning_horizon_s(self) -> float:
        """Derivación informativa; no participa en la decisión."""
        return self.tuning.planning_horizon_cm / max(self.geometry.fixed_speed_cm_s, 1e-9)

    @property
    def preview_horizon_s(self) -> float:
        """Valor temporal histórico conservado para visualización."""
        return self.tuning.preview_horizon_s

    @staticmethod
    def _known_obstacles(data: PlannerInput) -> tuple[VisibleObstacle, ...]:
        """Combina detecciones actuales y memoria sin duplicar objetos."""
        known = {item.object_id: item for item in data.tracked_obstacles}
        known.update({item.object_id: item for item in data.visible_obstacles})
        return tuple(known.values())

    def _nearest(self, data: PlannerInput) -> VisibleObstacle | None:
        """Devuelve el obstáculo visible más próximo que está por delante.

        La capa de percepción ya entrega exclusivamente objetos dentro del
        FOV. No se aplica un segundo umbral de distancia: desde la primera
        detección el planner debe poder predecir cómo pasar el objeto, aunque
        la acción inmediata siga siendo recta mientras la proyección frontal
        esté libre.
        """
        s = data.vehicle_state
        forward = (math.cos(s.heading_rad), math.sin(s.heading_rad))
        known_obstacles = self._known_obstacles(data)
        if data.active_target_id is not None:
            active = next(
                (
                    obstacle for obstacle in known_obstacles
                    if obstacle.object_id == data.active_target_id
                    and not obstacle.already_passed
                ),
                None,
            )
            if active is not None:
                return active
        ahead = []
        for obstacle in known_obstacles:
            if obstacle.already_passed:
                continue
            dx, dy = obstacle.x_cm - s.x_cm, obstacle.y_cm - s.y_cm
            longitudinal = dx * forward[0] + dy * forward[1]
            if longitudinal > 0.0:
                ahead.append(obstacle)
        return min(ahead, key=lambda o: math.dist((s.x_cm,s.y_cm),(o.x_cm,o.y_cm)), default=None)

    def _track_tangent(self, data: PlannerInput) -> float:
        """Dirección longitudinal local de la ruta o, sin ella, de los muros."""
        # El adaptador de percepción/ruta conoce la dirección de avance de la
        # recta actual. Es más fiable que inferirla con un segmento de muro
        # cuando el carro está en una esquina, donde hay muros verticales y
        # horizontales visibles al mismo tiempo.
        if data.desired_heading_rad is not None:
            return data.desired_heading_rad
        headings: list[float] = []
        if data.drivable_boundary:
            headings.extend(
                math.atan2(b[1] - a[1], b[0] - a[0])
                for a, b in zip(
                    data.drivable_boundary,
                    data.drivable_boundary[1:] + data.drivable_boundary[:1],
                )
            )
        headings.extend(
            math.atan2(wall.end[1] - wall.start[1], wall.end[0] - wall.start[0])
            for wall in data.visible_walls
            if wall.start != wall.end
        )
        if not headings:
            return data.vehicle_state.heading_rad
        oriented = [
            min((heading, normalize_angle(heading + math.pi)),
                key=lambda value: abs(normalize_angle(value - data.vehicle_state.heading_rad)))
            for heading in headings
        ]
        return min(
            oriented,
            key=lambda value: abs(normalize_angle(value - data.vehicle_state.heading_rad)),
        )

    def _target_tangent(self, data: PlannerInput, target: VisibleObstacle) -> float:
        """Tangente local de la recta a la que pertenece el obstáculo."""
        if len(data.route_centerline) >= 2:
            points = data.route_centerline
            start, end = min(
                zip(points, points[1:]),
                key=lambda segment: point_segment_distance(
                    (target.x_cm, target.y_cm), *segment,
                ),
            )
            if start != end:
                return math.atan2(end[1] - start[1], end[0] - start[0])

        # Fallback para una integración que no tenga aún centro de ruta.
        tangent = self._track_tangent(data)
        right = (-math.sin(tangent), math.cos(tangent))
        lateral = ((target.x_cm - data.vehicle_state.x_cm) * right[0]
                   + (target.y_cm - data.vehicle_state.y_cm) * right[1])
        corridor_half_width = min(
            self.geometry.minimum_left_radius_cm,
            self.geometry.minimum_right_radius_cm,
        )
        if abs(lateral) <= corridor_half_width or data.track_direction is None:
            return tangent
        turn = math.pi / 2 if data.track_direction is TrackDirection.CLOCKWISE else -math.pi / 2
        return normalize_angle(tangent + turn)

    def _pass_target(
        self, data: PlannerInput, target: VisibleObstacle
    ) -> PassTarget | None:
        """Construye el objetivo lateral de paso para un obstáculo coloreado."""
        color = target.color.lower()
        if color not in {"red", "green"}:
            return None
        tangent = self._target_tangent(data, target)
        forward = (math.cos(tangent), math.sin(tangent))
        side = LocalSide.RIGHT if color == "red" else LocalSide.LEFT
        clearance = (
            target.width_cm / 2
            + self.geometry.width_cm / 2
            + self._hard_clearance("side")
            + NUMERICAL_CLEARANCE_TOLERANCE_CM
        )
        return PassTarget(
            obstacle_id=target.object_id,
            side=side,
            tangent_rad=tangent,
            forward_vector=forward,
            right_vector=right_vector(forward),
            target_lateral_offset_cm=clearance if side is LocalSide.RIGHT else -clearance,
        )

    @staticmethod
    def _lateral_offset(point: Point, reference: Point, pass_target: PassTarget) -> float:
        dx, dy = point[0] - reference[0], point[1] - reference[1]
        return dx * pass_target.right_vector[0] + dy * pass_target.right_vector[1]

    @staticmethod
    def _pass_boundary_error(
        current_lateral: float, pass_target: PassTarget
    ) -> float:
        """Devuelve solo el error que aún falta para cruzar la frontera.

        ``target_lateral_offset_cm`` es el límite mínimo de separación, no un
        punto que el vehículo deba alcanzar exactamente. Por eso cualquier
        posición más allá de la frontera ya tiene error cero.
        """
        boundary = pass_target.target_lateral_offset_cm
        if pass_target.side is LocalSide.LEFT:
            return min(0.0, boundary - current_lateral)
        return max(0.0, boundary - current_lateral)

    def _steering_towards_pass_target(
        self, data: PlannerInput, target: VisibleObstacle, pass_target: PassTarget
    ) -> int:
        """Convierte el error lateral de la recta a LEFT/RIGHT del vehículo."""
        state = data.vehicle_state
        current = self._lateral_offset(
            (state.x_cm, state.y_cm), (target.x_cm, target.y_cm), pass_target,
        )
        error = self._pass_boundary_error(current, pass_target)
        if abs(error) <= NUMERICAL_CLEARANCE_TOLERANCE_CM:
            return 0
        desired_track_direction = 1.0 if error > 0.0 else -1.0
        vehicle_right = (-math.sin(state.heading_rad), math.cos(state.heading_rad))
        projected = (
            desired_track_direction * pass_target.right_vector[0] * vehicle_right[0]
            + desired_track_direction * pass_target.right_vector[1] * vehicle_right[1]
        )
        return 1 if projected >= 0.0 else -1

    def _planning_distance(
        self, data: PlannerInput, target: VisibleObstacle | None,
        distance_override_cm: float | None = None,
    ) -> float:
        """Horizonte único en centímetros, ampliado por el active target."""
        requested = (
            distance_override_cm
            if distance_override_cm is not None
            else self.tuning.planning_horizon_cm
        )
        if target is not None:
            tangent = self._target_tangent(data, target)
            forward = (math.cos(tangent), math.sin(tangent))
            relative = (
                target.x_cm - data.vehicle_state.x_cm,
                target.y_cm - data.vehicle_state.y_cm,
            )
            target_distance = max(
                0.0,
                relative[0] * forward[0] + relative[1] * forward[1],
            )
            required = (
                target_distance
                + target.length_cm / 2
                + self.geometry.length_cm / 2
                + self._hard_clearance("rear")
                + self.tuning.post_pass_margin_cm
            )
            requested = max(requested, required)
        return min(requested, FIXED_RULES.perception_range_cm)

    def _straight(self, distance_cm: float) -> CandidateTrajectory:
        return CandidateTrajectory(
            "STRAIGHT", None,
            (MotionPrimitive(
                PrimitiveType.STRAIGHT, distance_cm, 0.0,
                self.geometry.fixed_speed_cm_s,
            ),),
        )

    def _beam_actions(
        self, level: int = 0,
    ) -> tuple[tuple[str, PrimitiveType, float, TrajectoryProfile | None], ...]:
        """Acciones discretas Ackermann usadas por cada nivel del beam."""
        actions = [("STRAIGHT", PrimitiveType.STRAIGHT, 0.0, None)]
        fractions = self.tuning.steering_fractions
        for side, kind in (
            ("LEFT", PrimitiveType.ARC_LEFT),
            ("RIGHT", PrimitiveType.ARC_RIGHT),
        ):
            limit = self.geometry.steering_limit_deg(kind is PrimitiveType.ARC_RIGHT)
            for index, fraction in enumerate(fractions):
                label = "SOFT" if index == 0 else "STRONG" if index == len(fractions) - 1 else str(index + 1)
                actions.append((
                    f"{side}_{label}", kind, fraction * limit,
                    TrajectoryProfile.CONSERVATIVE if index == 0 else TrajectoryProfile.TIGHT,
                ))
        return tuple(actions)

    def _beam_primitive(
        self,
        kind: PrimitiveType,
        steering_deg: float,
        distance_cm: float,
    ) -> MotionPrimitive:
        if kind is PrimitiveType.STRAIGHT:
            steering = 0.0
        else:
            steering = steering_deg if kind is PrimitiveType.ARC_RIGHT else -steering_deg
        return MotionPrimitive(
            kind, distance_cm, steering, self.geometry.fixed_speed_cm_s,
        )

    def _beam_partial_valid(self, candidate: CandidateTrajectory) -> bool:
        """Valida fallos duros ya calculados durante la expansión."""
        return not candidate.hard_failure and candidate.rejection_reason is None

    def _beam_rank(self, candidate: CandidateTrajectory, data: PlannerInput) -> float:
        """Heurística mínima de poda; nunca sustituye al scorer final."""
        if candidate.final_state is None:
            return -math.inf
        final = candidate.final_state
        desired = data.desired_heading_rad
        heading_error = 0.0 if desired is None else abs(math.degrees(
            normalize_angle(final.heading_rad - desired),
        ))
        candidate.progress_cm = math.dist(
            (data.vehicle_state.x_cm, data.vehicle_state.y_cm),
            (final.x_cm, final.y_cm),
        )
        candidate.final_heading_error_deg = heading_error
        return candidate.progress_cm - heading_error * 0.1

    @staticmethod
    def _beam_family(candidate: CandidateTrajectory) -> str:
        if not candidate.primitives:
            return "STRAIGHT"
        # La diversidad se mantiene sobre la expansión más reciente. Usar
        # únicamente la primera acción haría que todos los descendientes de
        # una rama LEFT parecieran la misma familia y podaría sus continuaciones
        # STRAIGHT/RIGHT útiles.
        first = candidate.primitives[-1].kind
        return {
            PrimitiveType.ARC_LEFT: "LEFT",
            PrimitiveType.ARC_RIGHT: "RIGHT",
        }.get(first, "STRAIGHT")

    def _prune_beam(
        self, expanded: list[CandidateTrajectory], data: PlannerInput,
    ) -> list[CandidateTrajectory]:
        """Conserva ranking y diversidad de familias de steering."""
        ranked = sorted(expanded, key=lambda item: self._beam_rank(item, data), reverse=True)
        if len(ranked) <= self.tuning.beam_width:
            return ranked
        if self.tuning.beam_width == 1:
            return ranked[:1]
        selected: list[CandidateTrajectory] = [ranked[0]]
        families = ("STRAIGHT", "LEFT", "RIGHT")
        for family in families:
            candidate = next((item for item in ranked if self._beam_family(item) == family), None)
            if candidate is not None and candidate not in selected and len(selected) < self.tuning.beam_width:
                selected.append(candidate)
        for candidate in ranked:
            if len(selected) >= self.tuning.beam_width:
                break
            if candidate not in selected:
                selected.append(candidate)
        return selected

    def _geometry_cache(self, data: PlannerInput) -> GeometryCache:
        """Precalcula las geometrías estáticas de un ciclo de planificación."""
        target_polygon = None
        known_obstacles = self._known_obstacles(data)
        if data.active_target_id is not None:
            target = next(
                (item for item in known_obstacles
                 if item.object_id == data.active_target_id),
                None,
            )
            target_polygon = target.polygon() if target is not None else None
        obstacle_geometry_list = []
        for item in known_obstacles:
            polygon = item.polygon()
            obstacle_geometry_list.append((
                item.object_id,
                polygon,
                _polygon_edges(polygon),
                tuple(_axes(polygon)),
            ))
        obstacle_geometry = tuple(obstacle_geometry_list)
        return GeometryCache(
            obstacle_polygons=tuple(
                (object_id, polygon)
                for object_id, polygon, _, _ in obstacle_geometry
            ),
            obstacle_geometry=obstacle_geometry,
            wall_segments=tuple(
                (item.wall_id, item.start, item.end, item.thickness_cm)
                for item in data.visible_walls
            ),
            boundary_edges=tuple(
                zip(data.drivable_boundary,
                    data.drivable_boundary[1:] + data.drivable_boundary[:1])
            ) if data.drivable_boundary else (),
            boundary_polygon=data.drivable_boundary,
            active_target_polygon=target_polygon,
        )

    def _initialize_candidate(
        self, initial: VehicleState, candidate: CandidateTrajectory,
        cache: GeometryCache,
    ) -> None:
        candidate.geometry_cache = cache
        footprint = self.geometry.footprint(initial)
        candidate.final_state = initial
        candidate.final_footprint = footprint
        candidate.closest_target_footprint = footprint
        candidate.closest_target_distance_cm = math.inf
        clearance = self._evaluate_footprint(footprint, cache)
        candidate.points = []
        candidate.trajectory_points = []
        if self.tuning.diagnostic_level == "full":
            candidate.points.append((initial.x_cm, initial.y_cm))
            candidate.trajectory_points.append(TrajectoryPoint(
                initial, 0, 0.0, footprint, clearance,
            ))
        self._accumulate_clearance(candidate, clearance)

    @staticmethod
    def _copy_candidate(parent: CandidateTrajectory, candidate_id: str,
                        profile: TrajectoryProfile, primitives: tuple[MotionPrimitive, ...],
                        target_obstacle_id: str | None,
                        desired_pass_side: LocalSide | None) -> CandidateTrajectory:
        return replace(
            parent,
            candidate_id=candidate_id,
            profile=profile,
            primitives=primitives,
            points=list(parent.points),
            trajectory_points=list(parent.trajectory_points),
            target_obstacle_id=target_obstacle_id,
            desired_pass_side=desired_pass_side,
            safe=False,
            rejection_reason=None,
            diagnostic_rejection_reason=None,
            collision_type=None,
            collision_object_id=None,
            hard_failure=False,
            score=-math.inf,
        )

    @staticmethod
    def _accumulate_clearance(
        candidate: CandidateTrajectory, clearance: ClearanceResult,
    ) -> None:
        candidate.minimum_clearance_cm = min(
            candidate.minimum_clearance_cm, clearance.min_clearance_cm,
        )
        candidate.minimum_obstacle_clearance_cm = min(
            candidate.minimum_obstacle_clearance_cm,
            clearance.obstacle_clearance_cm,
        )
        candidate.minimum_wall_clearance_cm = min(
            candidate.minimum_wall_clearance_cm,
            clearance.wall_clearance_cm,
        )
        if clearance.collision:
            candidate.physical_collision = True
            candidate.hard_failure = True
            if candidate.collision_type is None:
                candidate.collision_type = clearance.collision_type
                candidate.collision_object_id = clearance.object_id

    def _simulate_segment(
        self, initial: VehicleState, primitive: MotionPrimitive,
        candidate: CandidateTrajectory, primitive_index: int,
        cache: GeometryCache, *, stop_on_collision: bool = True,
    ) -> VehicleState:
        """Simula únicamente un nuevo segmento y acumula sus métricas."""
        self._cycle_stats["segment_simulations"] += 1
        if not self._primitive_valid(primitive):
            candidate.rejection_reason = "kinematics"
            candidate.diagnostic_rejection_reason = "KINEMATIC_LIMIT"
            candidate.hard_failure = True
            return initial
        state = initial
        traveled = 0.0
        for _ in range(2000):
            if traveled >= primitive.distance_cm:
                break
            old = state
            state = vehicle_step(
                state,
                ControlCommand(primitive.target_speed_cm_s, primitive.steering_angle_deg),
                FIXED_RULES.simulation_dt_s,
                self.geometry,
            )
            increment = math.dist(
                (old.x_cm, old.y_cm), (state.x_cm, state.y_cm),
            )
            traveled += increment
            total = candidate.length_cm + increment
            footprint = self.geometry.footprint(state)
            clearance = self._evaluate_footprint(footprint, cache)
            if self.tuning.diagnostic_level == "full":
                candidate.points.append((state.x_cm, state.y_cm))
                candidate.trajectory_points.append(TrajectoryPoint(
                    state, primitive_index, total, footprint, clearance,
                ))
            candidate.final_state = state
            candidate.final_footprint = footprint
            if candidate.target_obstacle_id is not None:
                target_polygon = next(
                    (
                        polygon for object_id, polygon in cache.obstacle_polygons
                        if object_id == candidate.target_obstacle_id
                    ),
                    None,
                )
                if target_polygon is not None:
                    target_center = (
                        sum(point[0] for point in target_polygon) / len(target_polygon),
                        sum(point[1] for point in target_polygon) / len(target_polygon),
                    )
                    target_distance = math.dist(
                        (state.x_cm, state.y_cm), target_center,
                    )
                    if target_distance < candidate.closest_target_distance_cm:
                        candidate.closest_target_distance_cm = target_distance
                        candidate.closest_target_footprint = footprint
            candidate.length_cm = total
            self._accumulate_clearance(candidate, clearance)
            if clearance.collision and stop_on_collision:
                return state
        else:
            candidate.rejection_reason = "simulation_limit"
            candidate.diagnostic_rejection_reason = "SIMULATION_LIMIT"
        candidate.steering_effort += abs(primitive.steering_angle_deg) * primitive.distance_cm
        if len(candidate.primitives) > 1:
            previous = candidate.primitives[-2]
            candidate.steering_changes += abs(
                primitive.steering_angle_deg - previous.steering_angle_deg,
            )
        return state

    def _straight_beam_candidate(
        self, data: PlannerInput, cache: GeometryCache,
    ) -> CandidateTrajectory:
        """Fast path equivalente al beam recto completo."""
        horizon = self._planning_distance(data, None)
        segment = horizon / self.tuning.prediction_segments
        candidate = CandidateTrajectory(
            "BEAM:" + ">".join(["STRAIGHT"] * self.tuning.prediction_segments),
            TrajectoryProfile.NOMINAL,
            (),
        )
        candidate.horizon_cm = horizon
        self._initialize_candidate(data.vehicle_state, candidate, cache)
        state = data.vehicle_state
        for index in range(self.tuning.prediction_segments):
            primitive = MotionPrimitive(
                PrimitiveType.STRAIGHT, segment, 0.0,
                self.geometry.fixed_speed_cm_s,
            )
            candidate.primitives += (primitive,)
            state = self._simulate_segment(
                state, primitive, candidate, index, cache,
            )
        return candidate

    def _combine_recovery_candidate(
        self, probe: CandidateTrajectory, forward: CandidateTrajectory,
    ) -> CandidateTrajectory:
        """Une un reverse ya simulado con un forward ya simulado."""
        reverse_distance = self._reverse_distance(probe)
        points = list(probe.points) + list(forward.points[1:])
        trajectory_points = list(probe.trajectory_points)
        trajectory_points.extend(
            replace(
                point,
                traveled_cm=point.traveled_cm + reverse_distance,
            )
            for point in forward.trajectory_points[1:]
        )
        return replace(
            forward,
            candidate_id=f"RECOVERY_{reverse_distance:g}CM:{forward.candidate_id}",
            primitives=probe.primitives + forward.primitives,
            points=points,
            trajectory_points=trajectory_points,
            length_cm=probe.length_cm + forward.length_cm,
            horizon_cm=probe.length_cm + forward.horizon_cm,
            minimum_clearance_cm=min(
                probe.minimum_clearance_cm, forward.minimum_clearance_cm,
            ),
            minimum_obstacle_clearance_cm=min(
                probe.minimum_obstacle_clearance_cm,
                forward.minimum_obstacle_clearance_cm,
            ),
            minimum_wall_clearance_cm=min(
                probe.minimum_wall_clearance_cm,
                forward.minimum_wall_clearance_cm,
            ),
            physical_collision=probe.physical_collision or forward.physical_collision,
            hard_failure=probe.hard_failure or forward.hard_failure,
            collision_type=probe.collision_type or forward.collision_type,
            collision_object_id=probe.collision_object_id or forward.collision_object_id,
            geometry_cache=probe.geometry_cache,
            steering_effort=probe.steering_effort + forward.steering_effort,
            steering_changes=(
                probe.steering_changes + forward.steering_changes
                + abs(
                    forward.primitives[0].steering_angle_deg
                    - probe.primitives[-1].steering_angle_deg
                )
                if probe.primitives and forward.primitives
                else probe.steering_changes + forward.steering_changes
            ),
            safe=False,
            rejection_reason=None,
            diagnostic_rejection_reason=None,
            score=-math.inf,
        )

    def _forward_candidates(
        self, data: PlannerInput, distance_override_cm: float | None = None,
        cache: GeometryCache | None = None,
    ) -> list[CandidateTrajectory]:
        """Genera trayectorias completas con expansión incremental del beam."""
        cache = cache or self._geometry_cache(data)
        target = self._nearest(data)
        pass_target = self._pass_target(data, target) if target else None
        horizon_cm = self._planning_distance(data, target, distance_override_cm)
        segment_cm = horizon_cm / self.tuning.prediction_segments
        beam: list[CandidateTrajectory] = [CandidateTrajectory(
            "BEAM_ROOT", TrajectoryProfile.NOMINAL, (),
            target_obstacle_id=target.object_id if target else None,
            desired_pass_side=pass_target.side if pass_target else None,
        )]
        self._initialize_candidate(data.vehicle_state, beam[0], cache)
        beam[0].horizon_cm = horizon_cm

        for level in range(self.tuning.prediction_segments):
            expanded: list[CandidateTrajectory] = []
            for parent in beam:
                for label, kind, fraction, profile in self._beam_actions(level):
                    primitive = self._beam_primitive(kind, fraction, segment_cm)
                    candidate = self._copy_candidate(
                        parent,
                        f"BEAM:{label}" if parent.candidate_id == "BEAM_ROOT"
                        else f"{parent.candidate_id}>{label}",
                        profile or parent.profile or TrajectoryProfile.NOMINAL,
                        parent.primitives + (primitive,),
                        target.object_id if target else None,
                        pass_target.side if pass_target else None,
                    )
                    self._simulate_segment(
                        parent.final_state or data.vehicle_state,
                        primitive,
                        candidate,
                        level,
                        cache,
                    )
                    if self._beam_partial_valid(candidate):
                        expanded.append(candidate)
            beam = self._prune_beam(expanded, data)
            kept = {id(item) for item in beam}
            for discarded in expanded:
                if id(discarded) not in kept:
                    discarded.beam_pruned = True
                    discarded.beam_pruned_reason = "beam_width_or_family_diversity"
                    self._pruned_candidates.append(discarded)
            if not beam:
                break

        complete = [
            candidate for candidate in beam
            if len(candidate.primitives) == self.tuning.prediction_segments
        ]
        for candidate in complete:
            maximum = max(
                (abs(primitive.steering_angle_deg) for primitive in candidate.primitives),
                default=0.0,
            )
            if maximum >= self.geometry.max_steering_deg * 0.95:
                candidate.profile = TrajectoryProfile.TIGHT
            elif maximum > 0.0:
                candidate.profile = TrajectoryProfile.CONSERVATIVE
            else:
                candidate.profile = TrajectoryProfile.NOMINAL
        return complete

    def _reverse_probes(self, data: PlannerInput) -> list[CandidateTrajectory]:
        """Variantes Ackermann de un incremento de reverse."""
        target = self._nearest(data)
        pass_target = self._pass_target(data, target) if target else None
        distance = self.tuning.reverse_step_cm
        angles = [0.0]
        for fraction in self.tuning.steering_fractions:
            angles.extend((
                -fraction * self.geometry.max_left_steering_deg,
                fraction * self.geometry.max_right_steering_deg,
            ))
        return [
            CandidateTrajectory(
                f"REVERSE_{distance:g}CM:{'STRAIGHT' if angle == 0 else 'LEFT' if angle < 0 else 'RIGHT'}",
                TrajectoryProfile.TIGHT,
                (MotionPrimitive(
                    PrimitiveType.REVERSE, distance, angle,
                    -self.geometry.fixed_speed_cm_s / 2,
                ),),
                target_obstacle_id=target.object_id if target else None,
                desired_pass_side=pass_target.side if pass_target else None,
            )
            for angle in angles
        ]

    @staticmethod
    def _reverse_distance(candidate: CandidateTrajectory) -> float:
        return sum(
            primitive.distance_cm for primitive in candidate.primitives
            if primitive.kind is PrimitiveType.REVERSE
        )

    def _primitive_valid(self, p: MotionPrimitive) -> bool:
        if abs(p.steering_angle_deg-self.geometry.clamp_steering(p.steering_angle_deg)) > 1e-9: return False
        radius = self.geometry.turning_radius_cm(p.steering_angle_deg)
        if p.kind is PrimitiveType.ARC_RIGHT: return radius is not None and radius >= self.geometry.minimum_right_radius_cm-1e-6
        if p.kind is PrimitiveType.ARC_LEFT: return radius is not None and abs(radius) >= self.geometry.minimum_left_radius_cm-1e-6
        return (
            (p.kind is PrimitiveType.STRAIGHT and radius is None)
            or p.kind is PrimitiveType.REVERSE
        )

    def simulate(
        self, initial: VehicleState, candidate: CandidateTrajectory,
        cache: GeometryCache | None = None,
    ) -> None:
        # Una trayectoria comprometida se vuelve a evaluar en cada ciclo. No
        # conservar el resultado anterior evita que un choque nuevo herede
        # ``safe=True`` o un score viejo.
        candidate.points = []
        candidate.trajectory_points = []
        candidate.safe = False
        candidate.physical_safe = False
        candidate.physical_collision = False
        candidate.minimum_clearance_cm = math.inf
        candidate.minimum_obstacle_clearance_cm = math.inf
        candidate.minimum_wall_clearance_cm = math.inf
        candidate.progress_cm = 0.0
        candidate.final_heading_error_deg = 0.0
        candidate.steering_effort = 0.0
        candidate.steering_changes = 0.0
        candidate.length_cm = 0.0
        candidate.correct_pass_side = True
        candidate.wrong_pass_side = False
        candidate.actual_pass_side = None
        candidate.current_pass_side_satisfied = False
        candidate.future_pass_viable = False
        candidate.target_passed = False
        candidate.target_passed_correctly = False
        candidate.initial_lateral_error_cm = 0.0
        candidate.final_lateral_error_cm = 0.0
        candidate.pass_progress_cm = 0.0
        candidate.raw_score = -math.inf
        candidate.final_score = -math.inf
        candidate.score_components = {}
        candidate.pass_side_adjustment = 0.0
        candidate.rejection_reason = None
        candidate.diagnostic_rejection_reason = None
        candidate.collision_type = None
        candidate.collision_object_id = None
        candidate.pass_side_feasible = None
        candidate.recovery_probe_valid = False
        candidate.score = -math.inf
        self._cycle_stats["simulation_calls"] += 1
        if cache is None:
            cache = self._geometry_cache(PlannerInput(initial))
        self._initialize_candidate(initial, candidate, cache)
        state = initial
        for index, primitive in enumerate(candidate.primitives):
            if not self._primitive_valid(primitive):
                candidate.rejection_reason="kinematics"
                candidate.diagnostic_rejection_reason="KINEMATIC_LIMIT"
                return
            state = self._simulate_segment(
                state, primitive, candidate, index, cache,
                stop_on_collision=False,
            )

    def _evaluate_footprint(
        self, footprint: Polygon, cache: GeometryCache,
    ) -> ClearanceResult:
        """Calcula una vez todas las distancias y el diagnóstico de colisión."""
        self._cycle_stats["clearance_evaluations"] += 1
        obstacle_clearance = math.inf
        obstacle_hits: list[str] = []
        footprint_edges = _polygon_edges(footprint)
        footprint_axes = tuple(_axes(footprint))
        for object_id, polygon, edges, axes in cache.obstacle_geometry:
            distance = _polygon_distance_prepared(
                footprint, footprint_edges, footprint_axes,
                polygon, edges, axes,
            )
            obstacle_clearance = min(obstacle_clearance, distance)
            if distance <= 1e-9:
                obstacle_hits.append(object_id)

        wall_clearance = math.inf
        wall_hit = False
        for _, start, end, thickness in cache.wall_segments:
            if any(_segments_intersect(start, end, a, b) for a, b in footprint_edges):
                distance = -thickness / 2
            else:
                distance = min(
                    min(point_segment_distance(point, start, end) for point in footprint),
                    min(
                        point_segment_distance(point, a, b)
                        for point in (start, end)
                        for a, b in footprint_edges
                    ),
                ) - thickness / 2
            wall_clearance = min(wall_clearance, distance)
            wall_hit |= distance <= 1e-9

        out_of_track = bool(cache.boundary_polygon) and not all(
            point_in_polygon(point, cache.boundary_polygon)
            for point in footprint
        )
        if cache.boundary_edges:
            wall_clearance = min(
                wall_clearance,
                min(
                    point_segment_distance(point, start, end)
                    for point in footprint
                    for start, end in cache.boundary_edges
                ),
            )

        collision = out_of_track or bool(obstacle_hits) or wall_hit
        if out_of_track:
            collision_type = "out_of_track"
        elif obstacle_hits and wall_hit:
            collision_type = "wall_and_obstacle"
        elif obstacle_hits:
            collision_type = "obstacle"
        elif wall_hit:
            collision_type = "wall"
        else:
            collision_type = None
        return ClearanceResult(
            collision,
            collision_type,
            obstacle_hits[0] if obstacle_hits else None,
            min(obstacle_clearance, wall_clearance),
            obstacle_clearance,
            wall_clearance,
        )

    def _clearance(self, footprint: Polygon, data: PlannerInput) -> tuple[bool,float,float,float]:
        """Compatibilidad para telemetría: el cálculo real está fusionado."""
        result = self._evaluate_footprint(footprint, self._geometry_cache(data))
        return (
            result.collision,
            result.min_clearance_cm,
            result.obstacle_clearance_cm,
            result.wall_clearance_cm,
        )

    def _refresh_candidate_clearance(
        self, candidate: CandidateTrajectory, cache: GeometryCache,
    ) -> None:
        """Actualiza una trayectoria creada con la API legacy de ``simulate``."""
        candidate.geometry_cache = cache
        candidate.minimum_clearance_cm = math.inf
        candidate.minimum_obstacle_clearance_cm = math.inf
        candidate.minimum_wall_clearance_cm = math.inf
        candidate.physical_collision = False
        candidate.hard_failure = False
        candidate.collision_type = None
        candidate.collision_object_id = None
        refreshed = []
        for point in candidate.trajectory_points:
            clearance = self._evaluate_footprint(point.footprint, cache)
            refreshed.append(replace(point, clearance=clearance))
            self._accumulate_clearance(candidate, clearance)
        candidate.trajectory_points = refreshed

    def _passed_target(
        self,
        candidate: CandidateTrajectory,
        data: PlannerInput,
        target: VisibleObstacle,
        cache: GeometryCache | None = None,
    ) -> bool:
        tangent=self._target_tangent(data,target)
        forward=(math.cos(tangent),math.sin(tangent))
        target_polygon = self._cached_obstacle_polygon(target, cache)
        obstacle_front=max(
            p[0]*forward[0]+p[1]*forward[1] for p in target_polygon
        )
        final_footprint = candidate.final_footprint
        if final_footprint is None and candidate.trajectory_points:
            final_footprint = candidate.trajectory_points[-1].footprint
        if final_footprint is None:
            return False
        final_rear=min(
            p[0]*forward[0]+p[1]*forward[1]
            for p in final_footprint
        )
        return final_rear > obstacle_front + self._hard_clearance("rear")

    def obstacle_passed_now(
        self, data: PlannerInput, target: VisibleObstacle,
        cache: GeometryCache | None = None,
    ) -> bool:
        """Confirma que el footprint completo ya rebasó el obstáculo.

        El lado correcto se diagnostica por separado. Un pase incorrecto no
        debe retener el active target ni provocar un reverse posterior.
        """
        candidate = CandidateTrajectory(
            "CURRENT_POSE", None, (), final_state=data.vehicle_state,
            final_footprint=self.geometry.footprint(data.vehicle_state),
            target_obstacle_id=target.object_id,
        )
        if not self._passed_target(candidate, data, target, cache):
            return False
        return True

    def _candidate_target(
        self, candidate: CandidateTrajectory, data: PlannerInput,
    ) -> VisibleObstacle | None:
        known = self._known_obstacles(data)
        if candidate.target_obstacle_id is not None:
            target = next(
                (item for item in known
                 if item.object_id == candidate.target_obstacle_id),
                None,
            )
            if target is not None:
                return target
        return self._nearest(data)

    def _correct_side(
        self, candidate:CandidateTrajectory,data:PlannerInput,
        cache: GeometryCache | None = None,
    )->bool:
        target=self._candidate_target(candidate, data)
        pass_target = self._pass_target(data, target) if target else None
        if not target or pass_target is None:
            return True
        if not self._passed_target(candidate, data, target, cache):
            return True
        if candidate.trajectory_points:
            closest_footprint = min(
                candidate.trajectory_points,
                key=lambda p: math.dist(
                    (p.state.x_cm, p.state.y_cm),
                    (target.x_cm, target.y_cm),
                ),
            ).footprint
        else:
            closest_footprint = candidate.closest_target_footprint or candidate.final_footprint
        if closest_footprint is None:
            return True
        actual_side = footprint_side(
            closest_footprint,
            self._cached_obstacle_polygon(target, cache),
            pass_target.forward_vector,
        )
        return actual_side is pass_target.side

    def _pass_side_at_pass(
        self, candidate: CandidateTrajectory, data: PlannerInput,
        target: VisibleObstacle, cache: GeometryCache | None = None,
    ) -> LocalSide | None:
        """Obtiene el lado del footprint en el primer instante de PASSED."""
        pass_target = self._pass_target(data, target)
        if pass_target is None:
            return None
        obstacle_polygon = self._cached_obstacle_polygon(target, cache)
        tangent = pass_target.forward_vector
        obstacle_front = max(point[0] * tangent[0] + point[1] * tangent[1]
                             for point in obstacle_polygon)
        for point in candidate.trajectory_points:
            rear = min(vertex[0] * tangent[0] + vertex[1] * tangent[1]
                       for vertex in point.footprint)
            if rear > obstacle_front + self._hard_clearance("rear"):
                side = footprint_side(point.footprint, obstacle_polygon, tangent)
                if side is not LocalSide.CENTER:
                    return side
        if candidate.final_footprint is not None:
            return footprint_side(candidate.final_footprint, obstacle_polygon, tangent)
        return None

    @staticmethod
    def _cached_obstacle_polygon(
        target: VisibleObstacle, cache: GeometryCache | None,
    ) -> Polygon:
        if cache is not None:
            for object_id, polygon in cache.obstacle_polygons:
                if object_id == target.object_id:
                    return polygon
        return target.polygon()

    def _record_pass_diagnostics(
        self, candidate: CandidateTrajectory, data: PlannerInput,
        cache: GeometryCache | None = None,
    ) -> bool:
        """Registra el estado lateral; la simulación decide la validez.

        No estima futuros radios ni invalida candidatos antes de que la
        trayectoria simulada alcance el obstáculo. Solo cuando el footprint
        lo supera existe un lado real que pueda ser correcto o incorrecto.
        """
        target = self._candidate_target(candidate, data)
        pass_target = self._pass_target(data, target) if target else None
        if not target or pass_target is None or (
            not candidate.trajectory_points and candidate.final_state is None
        ):
            return True
        initial = data.vehicle_state
        initial_lateral = self._lateral_offset(
            (initial.x_cm, initial.y_cm), (target.x_cm, target.y_cm), pass_target,
        )
        final = candidate.final_state or candidate.trajectory_points[-1].state
        current_lateral = self._lateral_offset(
            (final.x_cm, final.y_cm), (target.x_cm, target.y_cm), pass_target,
        )
        initial_error = self._pass_boundary_error(initial_lateral, pass_target)
        lateral_error = self._pass_boundary_error(current_lateral, pass_target)
        trajectory_errors = [initial_error]
        trajectory_errors.extend(
            self._pass_boundary_error(
                self._lateral_offset(
                    (point.state.x_cm, point.state.y_cm),
                    (target.x_cm, target.y_cm),
                    pass_target,
                ),
                pass_target,
            )
            for point in candidate.trajectory_points
        )
        candidate.target_obstacle_id = target.object_id
        candidate.desired_pass_side = pass_target.side
        candidate.initial_lateral_error_cm = abs(initial_error)
        candidate.final_lateral_error_cm = abs(lateral_error)
        candidate.pass_progress_cm = max(
            0.0,
            candidate.initial_lateral_error_cm - min(abs(error) for error in trajectory_errors),
        )
        candidate.current_lateral_offset_cm = current_lateral
        candidate.target_lateral_offset_cm = pass_target.target_lateral_offset_cm
        candidate.lateral_error_cm = lateral_error
        candidate.current_pass_side_satisfied = abs(initial_error) <= NUMERICAL_CLEARANCE_TOLERANCE_CM
        candidate.target_passed = self._passed_target(candidate, data, target, cache)
        candidate.actual_pass_side = self._pass_side_at_pass(candidate, data, target, cache)
        candidate.target_passed_correctly = (
            candidate.target_passed
            and candidate.actual_pass_side is pass_target.side
        )
        candidate.wrong_pass_side = (
            candidate.target_passed and not candidate.target_passed_correctly
        )
        # Future viability comes from the simulated footprint near the
        # obstacle, not from the current side alone. The adaptive horizon
        # normally reaches the obstacle and the post-pass margin; if the
        # perception cap prevents that, keeping the frontier satisfied at
        # the end is the only information available to the planner.
        obstacle_polygon = self._cached_obstacle_polygon(target, cache)
        tangent = pass_target.forward_vector
        obstacle_rear = min(
            point[0] * tangent[0] + point[1] * tangent[1]
            for point in obstacle_polygon
        )
        crossing_errors: list[float] = []
        for point in candidate.trajectory_points:
            front = max(
                vertex[0] * tangent[0] + vertex[1] * tangent[1]
                for vertex in point.footprint
            )
            if front >= obstacle_rear - NUMERICAL_CLEARANCE_TOLERANCE_CM:
                crossing_errors.append(
                    self._pass_boundary_error(
                        self._lateral_offset(
                            (point.state.x_cm, point.state.y_cm),
                            (target.x_cm, target.y_cm),
                            pass_target,
                        ),
                        pass_target,
                    )
                )
        if candidate.target_passed:
            candidate.future_pass_viable = candidate.target_passed_correctly
        elif crossing_errors:
            candidate.future_pass_viable = min(
                abs(error) for error in crossing_errors
            ) <= NUMERICAL_CLEARANCE_TOLERANCE_CM
        else:
            candidate.future_pass_viable = (
                abs(lateral_error) <= NUMERICAL_CLEARANCE_TOLERANCE_CM
            )
        candidate.correct_pass_side = candidate.target_passed_correctly or not candidate.target_passed
        candidate.pass_side_feasible = candidate.future_pass_viable
        return True

    def validate(
        self,
        candidate: CandidateTrajectory,
        data: PlannerInput,
        *,
        enforce_pass_side: bool = True,
        cache: GeometryCache | None = None,
    ) -> None:
        if candidate.rejection_reason:return
        cache = cache or self._geometry_cache(data)
        if candidate.geometry_cache is not cache:
            self._refresh_candidate_clearance(candidate, cache)
        # La separación medida entre dos footprints es lateral en los
        # corredores de paso. El margen frontal se usa para anticipar la
        # maniobra, no para estrechar artificialmente todo el corredor.
        hard=self._hard_clearance("side")
        if candidate.physical_collision:
            collision_type = candidate.collision_type
            if (
                collision_type in {"wall", "wall_and_obstacle", "out_of_track"}
                or not self.tuning.allow_physical_collisions
            ):
                candidate.rejection_reason = "collision"
                candidate.diagnostic_rejection_reason = {
                    "obstacle": "PHYSICAL_COLLISION",
                    "wall": "WALL_COLLISION",
                    "wall_and_obstacle": "WALL_COLLISION",
                    "out_of_track": "OUT_OF_TRACK",
                }.get(collision_type or "", "PHYSICAL_COLLISION")
                return
        candidate.physical_safe = not candidate.physical_collision
        if candidate.minimum_clearance_cm + NUMERICAL_CLEARANCE_TOLERANCE_CM < hard:
            candidate.rejection_reason="clearance"
            candidate.diagnostic_rejection_reason="CLEARANCE"
            return
        # Una sonda REVERSE solo valida que pueda crear espacio físico. El
        # lado obligatorio se comprueba en el forward que se genera desde su
        # pose final; exigirlo durante el retroceso impediría probar 4/8/12.
        if not enforce_pass_side:
            candidate.recovery_probe_valid = candidate.rejection_reason is None
            return
        self._record_pass_diagnostics(candidate, data, cache)
        target = self._candidate_target(candidate, data)
        initial = data.vehicle_state
        final = candidate.final_state or candidate.trajectory_points[-1].state
        desired=data.desired_heading_rad if data.desired_heading_rad is not None else initial.heading_rad
        candidate.progress_cm=(final.x_cm-initial.x_cm)*math.cos(desired)+(final.y_cm-initial.y_cm)*math.sin(desired)
        candidate.final_heading_error_deg=abs(math.degrees(normalize_angle(final.heading_rad-desired)))
        steer=[p.steering_angle_deg for p in candidate.primitives]
        candidate.steering_effort=sum(abs(p.steering_angle_deg)*p.distance_cm for p in candidate.primitives)
        candidate.steering_changes=sum(abs(b-a) for a,b in zip(steer,steer[1:]))
        preferred=self.tuning.preferred_clearance_cm
        reverse_distance=sum(
            primitive.distance_cm for primitive in candidate.primitives
            if primitive.kind is PrimitiveType.REVERSE
        )
        breakdown = score_trajectory_breakdown(
            minimum_clearance_cm=candidate.minimum_clearance_cm,
            preferred_clearance_cm=preferred,
            progress_cm=candidate.progress_cm,
            final_heading_error_deg=candidate.final_heading_error_deg,
            steering_effort=candidate.steering_effort,
            steering_changes=candidate.steering_changes,
            length_cm=candidate.length_cm,
            reverse_distance_cm=reverse_distance,
            physical_collision=candidate.physical_collision,
            allow_collisions=self.tuning.allow_physical_collisions,
            pass_progress_cm=candidate.pass_progress_cm,
            pass_progress_reward=self.tuning.pass_progress_reward,
            wrong_pass_side=candidate.wrong_pass_side,
            wrong_pass_side_penalty=self.tuning.wrong_pass_side_penalty,
            weights=self.tuning.score_weights,
        )
        candidate.raw_score = breakdown.raw_score
        candidate.final_score = breakdown.final_score
        candidate.pass_side_adjustment = breakdown.pass_side_adjustment
        candidate.score_components = {
            "clearance": breakdown.score_clearance,
            "progress": breakdown.score_progress,
            "heading": breakdown.score_heading,
            "steering": breakdown.score_steering,
            "steering_changes": breakdown.score_steering_changes,
            "length": breakdown.score_length,
            "pass_progress": breakdown.score_pass_progress,
            "wrong_pass_side": breakdown.score_wrong_pass_side,
            "reverse": breakdown.score_reverse,
        }
        candidate.score = candidate.final_score
        candidate.safe=True

    @staticmethod
    def command_for(candidate:CandidateTrajectory)->ControlCommand:
        primitive=candidate.primitives[0];return ControlCommand(primitive.target_speed_cm_s,primitive.steering_angle_deg)

    @staticmethod
    def _is_reverse_candidate(candidate: CandidateTrajectory) -> bool:
        return bool(candidate.primitives and candidate.primitives[0].kind is PrimitiveType.REVERSE)

    def _apply_pass_side_priority(
        self, candidates: Sequence[CandidateTrajectory],
    ) -> None:
        """Ajusta scores sin eliminar candidatos incorrectos.

        Cuando existe una alternativa segura que mantiene/completa el lado
        correcto, el ajuste se calcula a partir del rango real de raw scores
        del ciclo. Así la prioridad reglamentaria queda garantizada sin usar
        una constante arbitrariamente enorme ni convertir WRONG_PASS_SIDE en
        una restricción hard.
        """
        forward = [candidate for candidate in candidates
                   if not self._is_reverse_candidate(candidate) and candidate.safe]
        correct = [candidate for candidate in forward
                   if candidate.future_pass_viable and not candidate.wrong_pass_side]
        wrong = [candidate for candidate in forward if candidate.wrong_pass_side]
        if not wrong:
            return
        priority_gap = 0.0
        if correct:
            raw_values = [candidate.raw_score for candidate in forward
                          if math.isfinite(candidate.raw_score)]
            if raw_values:
                priority_gap = max(
                    self.tuning.wrong_pass_side_penalty,
                    max(raw_values) - min(raw_values) + 1.0,
                )
        for candidate in wrong:
            candidate.pass_side_adjustment = priority_gap
            candidate.score = (
                candidate.raw_score
                - self.tuning.wrong_pass_side_penalty
                - priority_gap
            )
            candidate.final_score = candidate.score
            candidate.score_components["wrong_pass_side"] = -self.tuning.wrong_pass_side_penalty

    def _no_safe_detail(
        self, candidates: Sequence[CandidateTrajectory], budget_exhausted: bool,
        reverse_attempted: bool,
    ) -> str:
        """Clasifica el motivo dominante sin convertirlo en un soft score."""
        if budget_exhausted:
            return "CANDIDATE_BUDGET_EXHAUSTED"
        reasons = [
            candidate.diagnostic_rejection_reason
            for candidate in candidates
            if candidate.diagnostic_rejection_reason is not None
        ]
        if reasons and all(reason in {"PHYSICAL_COLLISION", "WALL_COLLISION"}
                           for reason in reasons):
            return "ALL_COLLISION"
        if reasons and all(reason == "OUT_OF_TRACK" for reason in reasons):
            return "OUT_OF_TRACK"
        if reasons and all(reason == "KINEMATIC_LIMIT" for reason in reasons):
            return "KINEMATIC_LIMIT"
        if reasons and all(reason == "CLEARANCE" for reason in reasons):
            return "CLEARANCE"
        reverse = [candidate for candidate in candidates if self._is_reverse_candidate(candidate)]
        if reverse_attempted and not any(candidate.safe for candidate in reverse):
            return "NO_REVERSE_RECOVERY"
        return "UNKNOWN"

    def _budget_available(self, started: float, evaluated: int) -> bool:
        return (
            self.tuning.planning_budget_mode != "time"
            or evaluated == 0
            or (time.perf_counter() - started) * 1000 < self.tuning.max_planning_time_ms
        )

    def _evaluate(
        self, initial: VehicleState, candidate: CandidateTrajectory, data: PlannerInput,
        cache: GeometryCache | None = None, *, already_simulated: bool = False,
    ) -> None:
        cache = cache or self._geometry_cache(data)
        if not already_simulated:
            self.simulate(initial, candidate, cache)
        self.validate(candidate, data, cache=cache)

    def _recovery_input(self, data: PlannerInput, state: VehicleState) -> PlannerInput:
        return PlannerInput(
            vehicle_state=state,
            visible_obstacles=data.visible_obstacles,
            visible_walls=data.visible_walls,
            drivable_boundary=data.drivable_boundary,
            track_direction=data.track_direction,
            desired_heading_rad=data.desired_heading_rad,
            timestamp_s=data.timestamp_s,
            route_centerline=data.route_centerline,
            tracked_obstacles=data.tracked_obstacles,
            active_target_id=data.active_target_id,
        )

    def revalidate_committed(
        self, data: PlannerInput, candidate: CandidateTrajectory
    ) -> PlannerResult | None:
        """Valida la parte restante de un commitment sin generar alternativas."""
        self._evaluate(data.vehicle_state, candidate, data)
        if not candidate.safe:
            return None
        command = self.command_for(candidate)
        diagnostics = PlannerDiagnostics(
            candidates_generated=1,
            candidates_evaluated=1,
            minimum_clearance_cm=candidate.minimum_clearance_cm,
            minimum_obstacle_clearance_cm=candidate.minimum_obstacle_clearance_cm,
            minimum_wall_clearance_cm=candidate.minimum_wall_clearance_cm,
            selected_candidate_id=candidate.candidate_id,
            committed_candidate_id=candidate.candidate_id,
            selected_angle_deg=command.steering_angle_deg,
            selected_speed_cm_s=command.target_speed_cm_s,
            selected_radius_cm=self.geometry.turning_radius_cm(command.steering_angle_deg),
            reason="committed_safe_trajectory",
            active_target_id=data.active_target_id,
            target_obstacle_id=candidate.target_obstacle_id,
            desired_pass_side=(
                candidate.desired_pass_side.value if candidate.desired_pass_side else None
            ),
            current_lateral_offset_cm=candidate.current_lateral_offset_cm,
            target_lateral_offset_cm=candidate.target_lateral_offset_cm,
            lateral_error_cm=candidate.lateral_error_cm,
            pass_side_feasible=candidate.pass_side_feasible,
        )
        state = (
            PlannerState.FOLLOW
            if candidate.primitives[0].kind is PrimitiveType.STRAIGHT
            else PlannerState.MANEUVERING
        )
        return PlannerResult(command, state, [candidate], candidate, diagnostics, diagnostics.reason)

    def plan(self, data: PlannerInput) -> PlannerResult:
        """GENERAR -> SIMULAR -> VALIDAR -> SCORE -> SELECCIONAR."""
        started = time.perf_counter()
        self._cycle_stats = {
            "simulation_calls": 0,
            "segment_simulations": 0,
            "clearance_evaluations": 0,
        }
        self._pruned_candidates = []
        cache = self._geometry_cache(data)
        candidates: list[CandidateTrajectory] = []
        budget_exhausted = False
        reverse_attempted = False
        best: CandidateTrajectory | None = None

        # Diagnóstico únicamente; no compite ni puede reemplazar STRAIGHT.
        projection = self._straight(self.tuning.forward_projection_cm)
        self._evaluate(data.vehicle_state, projection, data, cache)

        target = self._nearest(data)
        aligned = (
            data.desired_heading_rad is None
            or abs(math.degrees(normalize_angle(
                data.vehicle_state.heading_rad - data.desired_heading_rad,
            ))) <= self.tuning.route_alignment_tolerance_deg
        )
        fast_path = target is None and aligned and projection.safe
        if fast_path:
            fast_candidate = self._straight_beam_candidate(data, cache)
            self._evaluate(
                data.vehicle_state, fast_candidate, data, cache,
                already_simulated=True,
            )
            if fast_candidate.safe:
                forward_candidates = [fast_candidate]
            else:
                fast_path = False
                forward_candidates = self._forward_candidates(data, cache=cache)
        else:
            forward_candidates = self._forward_candidates(data, cache=cache)
        for candidate in forward_candidates:
            if len(candidates) >= self.tuning.max_candidates or not self._budget_available(started, len(candidates)):
                budget_exhausted = True
                break
            if not candidate.safe:
                self._evaluate(
                    data.vehicle_state, candidate, data, cache,
                    already_simulated=True,
                )
            candidates.append(candidate)

        forward_safe = [
            candidate for candidate in candidates
            if candidate.safe and not candidate.beam_pruned
            and not self._is_reverse_candidate(candidate)
        ]
        self._apply_pass_side_priority(forward_safe)
        if forward_safe:
            best = max(forward_safe, key=lambda candidate: candidate.score)
        else:
            reverse_attempted = True
            distance = self.tuning.reverse_step_cm
            while distance <= self.tuning.max_reverse_recovery_cm + 1e-9:
                recovery_candidates: list[CandidateTrajectory] = []
                for probe in self._reverse_probes(data):
                    probe.primitives = (replace(probe.primitives[0], distance_cm=distance),)
                    probe.candidate_id = probe.candidate_id.replace(
                        f"REVERSE_{self.tuning.reverse_step_cm:g}CM",
                        f"REVERSE_{distance:g}CM",
                    )
                    if len(candidates) >= self.tuning.max_candidates or not self._budget_available(started, len(candidates)):
                        budget_exhausted = True
                        break
                    self.simulate(data.vehicle_state, probe, cache)
                    self.validate(probe, data, enforce_pass_side=False, cache=cache)
                    candidates.append(probe)
                    if not probe.recovery_probe_valid:
                        continue
                    recovery_state = probe.final_state or data.vehicle_state
                    recovery_data = self._recovery_input(data, recovery_state)
                    for forward in self._forward_candidates(recovery_data, cache=cache):
                        if len(candidates) >= self.tuning.max_candidates or not self._budget_available(started, len(candidates)):
                            budget_exhausted = True
                            break
                        self._evaluate(
                            recovery_state, forward, recovery_data, cache,
                            already_simulated=True,
                        )
                        recovery = self._combine_recovery_candidate(probe, forward)
                        self.validate(recovery, data, cache=cache)
                        candidates.append(recovery)
                        recovery_candidates.append(recovery)
                    if budget_exhausted:
                        break
                self._apply_pass_side_priority(recovery_candidates)
                recovery_safe = [candidate for candidate in recovery_candidates if candidate.safe]
                if recovery_safe:
                    best = max(
                        recovery_safe,
                        key=lambda candidate: (
                            -self._reverse_distance(candidate), candidate.score,
                        ),
                    )
                    break
                if budget_exhausted:
                    break
                distance += self.tuning.reverse_step_cm

            if best is None:
                best = None

        # Las ramas podadas se exponen en diagnostics, pero nunca se mezclan
        # con los candidatos completos seleccionables.
        diagnostic_candidates = [*candidates, *self._pruned_candidates]
        command = self.command_for(best) if best else ControlCommand(0.0, 0.0)
        state = (
            PlannerState.NO_SAFE_TRAJECTORY if best is None else
            PlannerState.FOLLOW if all(
                primitive.kind is PrimitiveType.STRAIGHT
                for primitive in best.primitives
            ) else PlannerState.MANEUVERING
        )
        diagnostics = PlannerDiagnostics(
            calculation_time_ms=(time.perf_counter() - started) * 1000,
            candidates_generated=len(diagnostic_candidates),
            candidates_evaluated=len(candidates),
            rejected_collision=sum(candidate.rejection_reason == "collision" for candidate in candidates),
            rejected_clearance=sum(candidate.rejection_reason == "clearance" for candidate in candidates),
            rejected_kinematics=sum(candidate.rejection_reason in {"kinematics", "simulation_limit"} for candidate in candidates),
            minimum_clearance_cm=min((candidate.minimum_clearance_cm for candidate in diagnostic_candidates), default=math.inf),
            minimum_obstacle_clearance_cm=min((candidate.minimum_obstacle_clearance_cm for candidate in diagnostic_candidates), default=math.inf),
            minimum_wall_clearance_cm=min((candidate.minimum_wall_clearance_cm for candidate in diagnostic_candidates), default=math.inf),
            selected_candidate_id=best.candidate_id if best else None,
            selected_angle_deg=command.steering_angle_deg,
            selected_speed_cm_s=command.target_speed_cm_s,
            selected_radius_cm=self.geometry.turning_radius_cm(command.steering_angle_deg),
            straight_projection_safe=projection.safe,
            reason="forward_candidate" if best and not self._is_reverse_candidate(best)
            else "reverse_recovery" if best else "NO_SAFE_TRAJECTORY",
            budget_exhausted=budget_exhausted,
            budget_reason=(
                "max_planning_time_ms" if budget_exhausted and self.tuning.planning_budget_mode == "time"
                else "max_candidates" if budget_exhausted else None
            ),
            active_target_id=data.active_target_id,
            no_safe_reason="ALL_FORWARD_AND_REVERSE_INVALID" if best is None else None,
            no_safe_detail=self._no_safe_detail(diagnostic_candidates, budget_exhausted, reverse_attempted) if best is None else None,
            simulation_calls=self._cycle_stats["simulation_calls"],
            segment_simulations=self._cycle_stats["segment_simulations"],
            clearance_evaluations=self._cycle_stats["clearance_evaluations"],
            fast_path=fast_path,
            diagnostic_level=self.tuning.diagnostic_level,
        )
        forward_candidates = [candidate for candidate in diagnostic_candidates if not self._is_reverse_candidate(candidate)]
        reverse_candidates = [candidate for candidate in diagnostic_candidates if self._is_reverse_candidate(candidate)]
        diagnostics.forward_candidates_generated = len(forward_candidates)
        diagnostics.forward_candidates_valid = sum(candidate.safe for candidate in forward_candidates)
        diagnostics.reverse_candidates_generated = len(reverse_candidates)
        diagnostics.reverse_candidates_valid = sum(candidate.safe for candidate in reverse_candidates)
        diagnostics.reverse_recovery_attempted = reverse_attempted
        diagnostics.reverse_distance_cm = self._reverse_distance(best) if best and self._is_reverse_candidate(best) else 0.0
        if best is not None and self._is_reverse_candidate(best):
            diagnostics.reverse_steering_deg = best.primitives[0].steering_angle_deg
            diagnostics.recovery_min_clearance_cm = best.minimum_clearance_cm
            diagnostics.recovery_final_score = best.final_score
            diagnostics.forward_after_reverse_candidate_id = (
                best.candidate_id.split(":", 1)[1]
                if ":" in best.candidate_id else None
            )

        rejection_reasons: dict[str, int] = {}
        forward_rejections: dict[str, int] = {}
        reverse_rejections: dict[str, int] = {}
        for candidate in diagnostic_candidates:
            reason = candidate.diagnostic_rejection_reason
            if reason is None:
                continue
            rejection_reasons[reason] = rejection_reasons.get(reason, 0) + 1
            bucket = reverse_rejections if self._is_reverse_candidate(candidate) else forward_rejections
            bucket[reason] = bucket.get(reason, 0) + 1
        diagnostics.rejection_reasons = rejection_reasons
        diagnostics.forward_rejections = forward_rejections
        diagnostics.reverse_rejections = reverse_rejections
        if self.tuning.diagnostic_level != "off":
            diagnostics.candidate_diagnostics = [
                {
                    "candidate_id": candidate.candidate_id,
                    "primitive": "+".join(item.kind.value for item in candidate.primitives),
                    "valid": candidate.safe,
                    "physical_safe": candidate.physical_safe,
                    "safe": candidate.safe,
                    "horizon_cm": candidate.horizon_cm,
                    "recovery_probe_valid": candidate.recovery_probe_valid,
                    "rejection_reason": candidate.diagnostic_rejection_reason,
                    "collision_type": candidate.collision_type,
                    "collision_object_id": candidate.collision_object_id,
                    "target_obstacle_id": candidate.target_obstacle_id,
                    "desired_pass_side": candidate.desired_pass_side.value if candidate.desired_pass_side else None,
                    "actual_pass_side": candidate.actual_pass_side.value if candidate.actual_pass_side else None,
                    "current_pass_side_satisfied": candidate.current_pass_side_satisfied,
                    "future_pass_viable": candidate.future_pass_viable,
                    "target_passed": candidate.target_passed,
                    "target_passed_correctly": candidate.target_passed_correctly,
                    "wrong_pass_side": candidate.wrong_pass_side,
                    "initial_lateral_error_cm": candidate.initial_lateral_error_cm,
                    "final_lateral_error_cm": candidate.final_lateral_error_cm,
                    "pass_progress_cm": candidate.pass_progress_cm,
                    "minimum_clearance_cm": candidate.minimum_clearance_cm,
                    "raw_score": candidate.raw_score,
                    "final_score": candidate.final_score,
                    "score_components": dict(candidate.score_components),
                    "score_clearance": candidate.score_components.get("clearance", 0.0),
                    "score_progress": candidate.score_components.get("progress", 0.0),
                    "score_heading": candidate.score_components.get("heading", 0.0),
                    "score_steering": candidate.score_components.get("steering", 0.0),
                    "score_steering_changes": candidate.score_components.get("steering_changes", 0.0),
                    "score_length": candidate.score_components.get("length", 0.0),
                    "score_pass_progress": candidate.score_components.get("pass_progress", 0.0),
                    "score_wrong_pass_side": candidate.score_components.get("wrong_pass_side", 0.0),
                    "score_reverse": candidate.score_components.get("reverse", 0.0),
                    "pass_side_adjustment": candidate.pass_side_adjustment,
                    "beam_pruned": candidate.beam_pruned,
                    "beam_pruned_reason": candidate.beam_pruned_reason,
                    "current_lateral_offset_cm": candidate.current_lateral_offset_cm,
                    "target_lateral_offset_cm": candidate.target_lateral_offset_cm,
                    "lateral_error_cm": candidate.lateral_error_cm,
                }
                for candidate in diagnostic_candidates
            ]
        rejected = next((candidate for candidate in diagnostic_candidates if candidate.rejection_reason), None)
        if rejected is not None:
            first = rejected.primitives[0]
            diagnostics.representative_rejected_candidate = {
                "candidate_id": rejected.candidate_id,
                "primitive": first.kind.value,
                "steering_deg": first.steering_angle_deg,
                "predicted_min_clearance_cm": rejected.minimum_clearance_cm if math.isfinite(rejected.minimum_clearance_cm) else None,
                "predicted_collision": rejected.physical_collision,
                "rejection_reason": rejected.diagnostic_rejection_reason,
                "collision_type": rejected.collision_type,
                "collision_object_id": rejected.collision_object_id,
            }
        reference = best or next((candidate for candidate in diagnostic_candidates if candidate.target_obstacle_id), None)
        if reference is not None:
            diagnostics.target_obstacle_id = reference.target_obstacle_id
            diagnostics.desired_pass_side = reference.desired_pass_side.value if reference.desired_pass_side else None
            diagnostics.current_lateral_offset_cm = reference.current_lateral_offset_cm
            diagnostics.target_lateral_offset_cm = reference.target_lateral_offset_cm
            diagnostics.lateral_error_cm = reference.lateral_error_cm
            diagnostics.pass_side_feasible = reference.pass_side_feasible
        if self.tuning.diagnostic_level != "full":
            for candidate in diagnostic_candidates:
                candidate.points.clear()
                candidate.trajectory_points.clear()
        if self.tuning.diagnostic_level == "off":
            diagnostics.candidate_diagnostics = []
            diagnostics.representative_rejected_candidate = None
        return PlannerResult(command, state, candidates, best, diagnostics, diagnostics.reason)

    def collision_metrics(self,state:VehicleState,data:PlannerInput)->tuple[bool,float,float,float]:
        return self._clearance(self.geometry.footprint(state),data)


def timing_percentiles(values:Sequence[float])->dict[str,float]:
    if not values:return {k:0.0 for k in ("p50_ms","p90_ms","p95_ms","p99_ms","max_ms")}
    ordered=sorted(values)
    def pct(f:float)->float:
        i=(len(ordered)-1)*f;lo,hi=math.floor(i),math.ceil(i);return ordered[lo] if lo==hi else ordered[lo]+(ordered[hi]-ordered[lo])*(i-lo)
    return {"p50_ms":pct(.5),"p90_ms":pct(.9),"p95_ms":pct(.95),"p99_ms":pct(.99),"max_ms":max(ordered)}
