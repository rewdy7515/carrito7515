"""Planificador geométrico puro para un carro Ackermann; sin Pygame ni hardware."""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Sequence

try:
    from planner_rules import FIXED_RULES
    from planner_tuning import PlannerTuning
except ImportError:
    from simulator.planner_rules import FIXED_RULES
    from simulator.planner_tuning import PlannerTuning

Point = tuple[float, float]
Polygon = tuple[Point, ...]
NUMERICAL_CLEARANCE_TOLERANCE_CM = 0.15


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
    max_speed_cm_s: float = 32.0
    fixed_speed_cm_s: float = 24.0
    max_acceleration_cm_s2: float = 45.0
    max_deceleration_cm_s2: float = 70.0
    max_steering_rate_deg_s: float = 90.0

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


@dataclass(frozen=True)
class MotionPrimitive:
    kind: PrimitiveType
    distance_cm: float
    steering_angle_deg: float
    target_speed_cm_s: float

    def __post_init__(self) -> None:
        if self.distance_cm <= 0:
            raise ValueError("distance_cm debe ser positivo")
        if self.kind in (PrimitiveType.STRAIGHT, PrimitiveType.REVERSE) and abs(self.steering_angle_deg) > 1e-9:
            raise ValueError("STRAIGHT/REVERSE requieren steering cero")
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


@dataclass
class CandidateTrajectory:
    candidate_id: str
    profile: TrajectoryProfile | None
    primitives: tuple[MotionPrimitive, ...]
    points: list[Point] = field(default_factory=list)
    trajectory_points: list[TrajectoryPoint] = field(default_factory=list)
    safe: bool = False
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
    rejection_reason: str | None = None
    score: float = -math.inf


@dataclass(frozen=True)
class ControlCommand:
    target_speed_cm_s: float
    steering_angle_deg: float


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
    PROFILE_FRACTION = {TrajectoryProfile.CONSERVATIVE: .5, TrajectoryProfile.NOMINAL: .72, TrajectoryProfile.TIGHT: 1.0}
    PROFILE_LATERAL_FRACTION = {TrajectoryProfile.CONSERVATIVE: 1.0,
                                TrajectoryProfile.NOMINAL: 1.0,
                                TrajectoryProfile.TIGHT: 1.0}

    def __init__(self, geometry: VehicleGeometry | None = None, tuning: PlannerTuning | None = None) -> None:
        self.tuning = (tuning or PlannerTuning()).validate()
        self.geometry = geometry or VehicleGeometry(fixed_speed_cm_s=self.tuning.fixed_speed_cm_s,
            max_acceleration_cm_s2=self.tuning.max_acceleration_cm_s2,
            max_deceleration_cm_s2=self.tuning.max_deceleration_cm_s2,
            max_steering_rate_deg_s=self.tuning.max_steering_rate_deg_s)

    @property
    def planning_horizon_s(self) -> float: return self.tuning.planning_horizon_s
    @property
    def preview_horizon_s(self) -> float: return self.tuning.preview_horizon_s

    def _nearest(self, data: PlannerInput) -> VisibleObstacle | None:
        s = data.vehicle_state
        tangent = self._track_tangent(data)
        forward = (math.cos(tangent), math.sin(tangent))
        trigger=(max(self.geometry.minimum_left_radius_cm,self.geometry.minimum_right_radius_cm)
                 + self.geometry.length_cm/2 + self.tuning.safety_margins.hard_front_cm
                 + self.geometry.fixed_speed_cm_s*self.tuning.replanning_period_s
                 + min(self.geometry.minimum_left_radius_cm,
                       self.geometry.minimum_right_radius_cm))
        ahead = []
        for obstacle in data.visible_obstacles:
            dx, dy = obstacle.x_cm - s.x_cm, obstacle.y_cm - s.y_cm
            longitudinal = dx * forward[0] + dy * forward[1]
            if 0 < longitudinal <= trigger and math.hypot(dx, dy) <= trigger:
                ahead.append(obstacle)
        return min(ahead, key=lambda o: math.dist((s.x_cm,s.y_cm),(o.x_cm,o.y_cm)), default=None)

    def _side(self, data: PlannerInput, target: VisibleObstacle | None) -> int:
        if target and target.color.lower() == "green": return -1
        if target and target.color.lower() == "red": return 1
        if data.desired_heading_rad is not None:
            heading_error=normalize_angle(data.desired_heading_rad-data.vehicle_state.heading_rad)
            if abs(heading_error)>math.radians(FIXED_RULES.route_alignment_tolerance_deg):
                return 1 if heading_error>0 else -1
        return 1 if data.track_direction is TrackDirection.CLOCKWISE else -1

    def _track_tangent(self, data: PlannerInput) -> float:
        """Dirección longitudinal local inferida de los límites conocidos."""
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
        """Tangente de la recta actual o de la siguiente si está en esquina."""
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

    def _straight(self) -> CandidateTrajectory:
        hard = max(self.tuning.safety_margins.hard_front_cm,
                   self.tuning.safety_margins.hard_side_cm,
                   self.tuning.safety_margins.hard_rear_cm)
        maneuver_distance = (max(self.geometry.minimum_left_radius_cm,
                                 self.geometry.minimum_right_radius_cm)
                             + self.geometry.length_cm / 2 + hard)
        distance = max(FIXED_RULES.forward_projection_cm,
                       self.geometry.fixed_speed_cm_s*self.tuning.planning_horizon_s,
                       maneuver_distance + self.geometry.fixed_speed_cm_s*self.tuning.replanning_period_s + 1.0)
        return CandidateTrajectory("STRAIGHT", None, (MotionPrimitive(PrimitiveType.STRAIGHT, distance, 0, self.geometry.fixed_speed_cm_s),))

    def _corner_candidates(
        self,
        data: PlannerInput,
        target: VisibleObstacle,
        next_tangent: float,
        overshoot_deg: float = 20.0,
    ) -> list[CandidateTrajectory]:
        """Arcos para entrar en la siguiente de las cuatro rectas."""
        clockwise = data.track_direction is TrackDirection.CLOCKWISE
        turn_sign = 1 if clockwise else -1
        kind = PrimitiveType.ARC_RIGHT if clockwise else PrimitiveType.ARC_LEFT
        # Un pequeño sobrepaso deja espacio para el footprint en la salida y
        # se corrige en los replannings siguientes.
        exit_heading = normalize_angle(
            next_tangent + turn_sign * math.radians(overshoot_deg)
        )
        arc_angle = clamp(
            abs(normalize_angle(exit_heading - data.vehicle_state.heading_rad)),
            math.radians(10.0),
            math.radians(110.0),
        )
        hold = self.geometry.length_cm + target.length_cm + 2 * self.tuning.safety_margins.hard_side_cm
        candidates: list[CandidateTrajectory] = []
        for profile, fraction in reversed(list(self.PROFILE_FRACTION.items())):
            steering = turn_sign * self.geometry.steering_limit_deg(clockwise) * fraction
            radius = abs(self.geometry.turning_radius_cm(steering) or math.inf)
            candidates.append(CandidateTrajectory(
                f"CORNER:{profile.value}:{'RIGHT' if clockwise else 'LEFT'}",
                profile,
                (
                    MotionPrimitive(kind, radius * arc_angle, steering,
                                    self.geometry.fixed_speed_cm_s),
                    MotionPrimitive(PrimitiveType.STRAIGHT, hold, 0.0,
                                    self.geometry.fixed_speed_cm_s),
                ),
            ))
        return candidates

    def _pass_then_route_candidates(
        self,
        data: PlannerInput,
        target: VisibleObstacle,
    ) -> list[CandidateTrajectory]:
        """Mantiene el lado correcto y corrige la ruta después del paso."""
        tangent = self._target_tangent(data, target)
        forward = (math.cos(tangent), math.sin(tangent))
        target_forward = ((target.x_cm - data.vehicle_state.x_cm) * forward[0]
                          + (target.y_cm - data.vehicle_state.y_cm) * forward[1])
        approach = max(
            1.0,
            target_forward
            - self.geometry.length_cm / 2
            - target.length_cm / 2
            - self.tuning.safety_margins.hard_rear_cm,
        )
        desired = data.desired_heading_rad or tangent
        heading_error = normalize_angle(desired - data.vehicle_state.heading_rad)
        turn_sign = 1 if heading_error > 0 else -1
        kind = PrimitiveType.ARC_RIGHT if turn_sign > 0 else PrimitiveType.ARC_LEFT
        candidates: list[CandidateTrajectory] = []
        for profile, fraction in self.PROFILE_FRACTION.items():
            steering = turn_sign * self.geometry.steering_limit_deg(turn_sign > 0) * fraction
            radius = abs(self.geometry.turning_radius_cm(steering) or math.inf)
            candidates.append(CandidateTrajectory(
                f"PASS_THEN_ROUTE:{profile.value}:{'RIGHT' if turn_sign > 0 else 'LEFT'}",
                profile,
                (
                    MotionPrimitive(PrimitiveType.STRAIGHT, approach, 0.0,
                                    self.geometry.fixed_speed_cm_s),
                    MotionPrimitive(kind, radius * abs(heading_error), steering,
                                    self.geometry.fixed_speed_cm_s),
                    MotionPrimitive(PrimitiveType.STRAIGHT, 20.0, 0.0,
                                    self.geometry.fixed_speed_cm_s),
                ),
            ))
        return candidates

    def _local_candidates(
        self,
        data: PlannerInput,
        target: VisibleObstacle | None,
        route_priority: bool,
    ) -> list[CandidateTrajectory]:
        """Tres acciones locales baratas para garantizar replanning continuo."""
        side = self._side(data, None if route_priority else target)
        kind = PrimitiveType.ARC_RIGHT if side > 0 else PrimitiveType.ARC_LEFT
        distance = max(
            10.0,
            self.geometry.fixed_speed_cm_s * self.tuning.replanning_period_s * 2.0,
        )
        result: list[CandidateTrajectory] = []
        for profile, fraction in reversed(list(self.PROFILE_FRACTION.items())):
            steering = side * self.geometry.steering_limit_deg(side > 0) * fraction
            result.append(CandidateTrajectory(
                f"LOCAL:{profile.value}:{'RIGHT' if side > 0 else 'LEFT'}",
                profile,
                (MotionPrimitive(kind, distance, steering,
                                 self.geometry.fixed_speed_cm_s),),
            ))
        return result

    def _avoidance(self, data: PlannerInput) -> list[CandidateTrajectory]:
        target = self._nearest(data)
        hard = self.tuning.safety_margins.hard_side_cm
        side = self._side(data, target)
        if target is not None:
            track_tangent = self._track_tangent(data)
            target_tangent = self._target_tangent(data, target)
            if abs(normalize_angle(target_tangent - track_tangent)) > math.radians(45.0):
                return self._corner_candidates(data, target, target_tangent)
        clearance_shift = (self.geometry.width_cm/2
                           + self.tuning.safety_margins.preferred_side_cm
                           + (target.width_cm/2 if target else 0)
                           + NUMERICAL_CLEARANCE_TOLERANCE_CM)
        shift = clearance_shift
        hold = self.geometry.length_cm + 2*hard + (target.length_cm if target else 0)
        target_forward = 0.0
        if target:
            s = data.vehicle_state
            target_tangent = self._target_tangent(data, target)
            track_tangent = self._track_tangent(data)
            projection_heading = (
                s.heading_rad
                if abs(normalize_angle(target_tangent - track_tangent))
                    <= math.radians(45.0)
                else target_tangent
            )
            forward = (math.cos(projection_heading), math.sin(projection_heading))
            right=(-forward[1],forward[0])
            target_forward = (target.x_cm-s.x_cm)*forward[0] + (target.y_cm-s.y_cm)*forward[1]
            target_lateral = (target.x_cm-s.x_cm)*right[0] + (target.y_cm-s.y_cm)*right[1]
            shift = max(0.0, clearance_shift + side * target_lateral)
            if (shift <= 1.0
                    and abs(normalize_angle(target_tangent - s.heading_rad))
                    > math.radians(FIXED_RULES.route_alignment_tolerance_deg)):
                return self._corner_candidates(data, target, target_tangent, overshoot_deg=0.0)
        candidates = []
        profiles = list(self.PROFILE_FRACTION.items())
        if target is not None:
            # Cerca de un obstáculo se evalúa primero la maniobra que necesita
            # menos longitud longitudinal. Así el presupuesto siempre cubre al
            # menos el perfil físicamente más apto para un espacio reducido.
            profiles.reverse()
        for profile, fraction in profiles:
            steering = side*self.geometry.steering_limit_deg(side>0)*fraction
            radius = abs(self.geometry.turning_radius_cm(steering) or math.inf)
            out = PrimitiveType.ARC_RIGHT if side>0 else PrimitiveType.ARC_LEFT
            if target is None:
                heading_error=(normalize_angle(data.desired_heading_rad-data.vehicle_state.heading_rad)
                               if data.desired_heading_rad is not None else side*math.pi/2)
                arc_angle=clamp(abs(heading_error),math.radians(10),math.radians(90))
                arc_distance = radius * arc_angle
                candidates.append(CandidateTrajectory(
                    f"{profile.value}:{'RIGHT' if side>0 else 'LEFT'}", profile,
                    (MotionPrimitive(out, arc_distance, steering, self.geometry.fixed_speed_cm_s),
                     MotionPrimitive(PrimitiveType.STRAIGHT, 20.0, 0, self.geometry.fixed_speed_cm_s))))
                continue
            back = PrimitiveType.ARC_LEFT if side>0 else PrimitiveType.ARC_RIGHT
            back_steering = -side*self.geometry.steering_limit_deg(side<0)*fraction
            back_radius = abs(self.geometry.turning_radius_cm(back_steering) or math.inf)
            # ``shift`` ya representa cuánto falta para colocar el footprint
            # al lado exigido. En replanning no se debe repetir el
            # desplazamiento completo desde cero.
            profile_shift=max(0.5,shift*self.PROFILE_LATERAL_FRACTION[profile])
            arc_angle=min(math.acos(clamp(1-profile_shift/(radius+back_radius),-1,1)), math.radians(90))
            arc = max(4, radius*arc_angle)
            back_arc = max(4,back_radius*arc_angle)
            steering_transition_cm = (
                self.geometry.fixed_speed_cm_s
                * abs(back_steering - steering)
                / self.geometry.max_steering_rate_deg_s
            )
            approach=max(0,target_forward-self.geometry.length_cm/2
                         -(target.length_cm/2 if target else 0)-hard
                         -(radius+back_radius)*math.sin(arc_angle))
            parts = []
            if approach > 1: parts.append(MotionPrimitive(PrimitiveType.STRAIGHT, approach, 0, self.geometry.fixed_speed_cm_s))
            parts += [MotionPrimitive(out, arc, steering, self.geometry.fixed_speed_cm_s),
                      MotionPrimitive(back, back_arc + steering_transition_cm,
                                      back_steering, self.geometry.fixed_speed_cm_s),
                      MotionPrimitive(PrimitiveType.STRAIGHT, hold, 0, self.geometry.fixed_speed_cm_s),
                      MotionPrimitive(back, back_arc, back_steering, self.geometry.fixed_speed_cm_s),
                      MotionPrimitive(out, arc + steering_transition_cm,
                                      steering, self.geometry.fixed_speed_cm_s)]
            candidates.append(CandidateTrajectory(f"{profile.value}:{'RIGHT' if side>0 else 'LEFT'}", profile, tuple(parts)))
        steering = side*self.geometry.steering_limit_deg(side>0)
        arc_kind = PrimitiveType.ARC_RIGHT if side>0 else PrimitiveType.ARC_LEFT
        candidates.append(CandidateTrajectory(f"REVERSE_TIGHT:{'RIGHT' if side>0 else 'LEFT'}", TrajectoryProfile.TIGHT,
            (MotionPrimitive(PrimitiveType.REVERSE, 10, 0, -self.geometry.fixed_speed_cm_s/2),
             MotionPrimitive(arc_kind, 18, steering, self.geometry.fixed_speed_cm_s),
             MotionPrimitive(PrimitiveType.STRAIGHT, hold, 0, self.geometry.fixed_speed_cm_s))))
        return candidates

    def _primitive_valid(self, p: MotionPrimitive) -> bool:
        if abs(p.steering_angle_deg-self.geometry.clamp_steering(p.steering_angle_deg)) > 1e-9: return False
        radius = self.geometry.turning_radius_cm(p.steering_angle_deg)
        if p.kind is PrimitiveType.ARC_RIGHT: return radius is not None and radius >= self.geometry.minimum_right_radius_cm-1e-6
        if p.kind is PrimitiveType.ARC_LEFT: return radius is not None and abs(radius) >= self.geometry.minimum_left_radius_cm-1e-6
        return radius is None

    def simulate(self, initial: VehicleState, candidate: CandidateTrajectory) -> None:
        state, total = initial, 0.0
        candidate.points = [(state.x_cm,state.y_cm)]
        candidate.trajectory_points = [TrajectoryPoint(state,0,0,self.geometry.footprint(state))]
        for index, primitive in enumerate(candidate.primitives):
            if not self._primitive_valid(primitive): candidate.rejection_reason="kinematics"; return
            traveled=0.0
            for _ in range(2000):
                if traveled >= primitive.distance_cm: break
                old=state
                state=vehicle_step(state,ControlCommand(primitive.target_speed_cm_s,primitive.steering_angle_deg),self.tuning.simulation_dt_s,self.geometry)
                increment=math.dist((old.x_cm,old.y_cm),(state.x_cm,state.y_cm)); traveled+=increment; total+=increment
                candidate.points.append((state.x_cm,state.y_cm)); candidate.trajectory_points.append(TrajectoryPoint(state,index,total,self.geometry.footprint(state)))
            else: candidate.rejection_reason="simulation_limit"; return
        candidate.length_cm=total

    def _clearance(self, footprint: Polygon, data: PlannerInput) -> tuple[bool,float,float,float]:
        obstacle=math.inf; wall=math.inf; collision=False
        for item in data.visible_obstacles:
            distance=polygon_distance(footprint,item.polygon()); obstacle=min(obstacle,distance); collision |= distance<=1e-9
        for item in data.visible_walls:
            distance=polygon_segment_distance(footprint,item.start,item.end)-item.thickness_cm/2; wall=min(wall,distance); collision |= distance<=1e-9
        if data.drivable_boundary:
            collision |= not all(point_in_polygon(p,data.drivable_boundary) for p in footprint)
            wall=min(wall,min(point_segment_distance(p,a,b) for p in footprint for a,b in zip(data.drivable_boundary,data.drivable_boundary[1:]+data.drivable_boundary[:1])))
        return collision,min(obstacle,wall),obstacle,wall

    def _passed_target(
        self,
        candidate: CandidateTrajectory,
        data: PlannerInput,
        target: VisibleObstacle,
    ) -> bool:
        tangent=self._target_tangent(data,target)
        forward=(math.cos(tangent),math.sin(tangent))
        obstacle_front=max(
            p[0]*forward[0]+p[1]*forward[1] for p in target.polygon()
        )
        final_rear=min(
            p[0]*forward[0]+p[1]*forward[1]
            for p in candidate.trajectory_points[-1].footprint
        )
        return final_rear > obstacle_front + self.tuning.safety_margins.hard_rear_cm

    def _correct_side(self,candidate:CandidateTrajectory,data:PlannerInput)->bool:
        target=self._nearest(data)
        if not target or target.color.lower() not in {"red","green"}: return True
        tangent=self._target_tangent(data,target);right=(-math.sin(tangent),math.cos(tangent))
        if not self._passed_target(candidate, data, target):
            return True
        closest=min(candidate.trajectory_points,key=lambda p:math.dist((p.state.x_cm,p.state.y_cm),(target.x_cm,target.y_cm)))
        lateral=(closest.state.x_cm-target.x_cm)*right[0]+(closest.state.y_cm-target.y_cm)*right[1]
        return lateral>0 if target.color.lower()=="red" else lateral<0

    def validate(self,candidate:CandidateTrajectory,data:PlannerInput)->None:
        if candidate.rejection_reason:return
        # La separación medida entre dos footprints es lateral en los
        # corredores de paso. El margen frontal se usa para anticipar la
        # maniobra, no para estrechar artificialmente todo el corredor.
        hard=self.tuning.safety_margins.hard_side_cm
        for point in candidate.trajectory_points:
            collision,clearance,obstacle,wall=self._clearance(point.footprint,data)
            candidate.minimum_clearance_cm=min(candidate.minimum_clearance_cm,clearance); candidate.minimum_obstacle_clearance_cm=min(candidate.minimum_obstacle_clearance_cm,obstacle); candidate.minimum_wall_clearance_cm=min(candidate.minimum_wall_clearance_cm,wall)
            if collision: candidate.physical_collision=True; candidate.rejection_reason="collision"; return
            if clearance+NUMERICAL_CLEARANCE_TOLERANCE_CM<hard: candidate.rejection_reason="clearance"; return
        candidate.correct_pass_side=self._correct_side(candidate,data)
        if not candidate.correct_pass_side:candidate.rejection_reason="wrong_pass_side";return
        initial,final=data.vehicle_state,candidate.trajectory_points[-1].state
        desired=data.desired_heading_rad if data.desired_heading_rad is not None else initial.heading_rad
        candidate.progress_cm=(final.x_cm-initial.x_cm)*math.cos(desired)+(final.y_cm-initial.y_cm)*math.sin(desired)
        candidate.final_heading_error_deg=abs(math.degrees(normalize_angle(final.heading_rad-desired)))
        steer=[p.steering_angle_deg for p in candidate.primitives]
        candidate.steering_effort=sum(abs(p.steering_angle_deg)*p.distance_cm for p in candidate.primitives)
        candidate.steering_changes=sum(abs(b-a) for a,b in zip(steer,steer[1:]))
        preferred=max(self.tuning.safety_margins.preferred_front_cm,self.tuning.safety_margins.preferred_side_cm,self.tuning.safety_margins.preferred_rear_cm)
        candidate.score=min(candidate.minimum_clearance_cm,preferred)*12+candidate.progress_cm*2-candidate.final_heading_error_deg*2.5-candidate.steering_effort*.01-candidate.steering_changes*.3-candidate.length_cm*.05
        candidate.safe=True

    @staticmethod
    def command_for(candidate:CandidateTrajectory)->ControlCommand:
        primitive=candidate.primitives[0];return ControlCommand(primitive.target_speed_cm_s,primitive.steering_angle_deg)

    def plan(self,data:PlannerInput)->PlannerResult:
        started=time.perf_counter()
        target = self._nearest(data)
        straight=self._straight();self.simulate(data.vehicle_state,straight);self.validate(straight,data)
        if target is not None:
            straight.correct_pass_side = self._correct_side(straight, data)
        straight_projection_safe=straight.safe
        heading_error=(abs(math.degrees(normalize_angle(
            (data.desired_heading_rad if data.desired_heading_rad is not None else data.vehicle_state.heading_rad)
            - data.vehicle_state.heading_rad))))
        if (straight.safe and target is None
                and heading_error>FIXED_RULES.route_alignment_tolerance_deg):
            straight.safe=False;straight.rejection_reason="route_heading"
        if (straight.safe and target is not None
                and abs(normalize_angle(
                    self._target_tangent(data, target) - self._track_tangent(data)
                )) > math.radians(45.0)
                and not self._passed_target(straight, data, target)):
            straight.safe=False;straight.rejection_reason="corner_target_ahead"
        candidates=[straight]; generated_count=1; budget_exhausted=False
        if not straight.safe:
            obstacle_path_is_already_valid = (
                target is not None
                and abs(normalize_angle(
                    self._target_tangent(data, target) - self._track_tangent(data)
                )) <= math.radians(45.0)
                and straight.correct_pass_side
                and straight.minimum_obstacle_clearance_cm
                    + NUMERICAL_CLEARANCE_TOLERANCE_CM
                    >= self.tuning.safety_margins.hard_side_cm
            )
            primary=(
                self._pass_then_route_candidates(data, target)
                if obstacle_path_is_already_valid and target is not None
                else self._avoidance(data)
            )
            generated=(self._local_candidates(
                data, target, obstacle_path_is_already_valid
            ) + primary)[:max(0,self.tuning.max_candidates-1)]
            generated_count += len(generated)
            for candidate in generated:
                if ((time.perf_counter()-started)*1000 >= self.tuning.max_planning_time_ms
                        and len(candidates)>1):
                    budget_exhausted=True
                    break
                self.simulate(data.vehicle_state,candidate);self.validate(candidate,data);candidates.append(candidate)
        safe=[c for c in candidates if c.safe]
        forward_safe=[c for c in safe if not c.candidate_id.startswith("REVERSE_")]
        selectable=forward_safe or safe
        best=max(selectable,key=lambda c:c.score,default=None)
        command=self.command_for(best) if best else ControlCommand(0,0)
        state=PlannerState.FOLLOW if best is straight else PlannerState.MANEUVERING if best else PlannerState.NO_SAFE_TRAJECTORY
        diagnostics=PlannerDiagnostics((time.perf_counter()-started)*1000,generated_count,len(candidates),
            sum(c.rejection_reason=="collision" for c in candidates),sum(c.rejection_reason in {"clearance","wrong_pass_side"} for c in candidates),sum(c.rejection_reason in {"kinematics","simulation_limit"} for c in candidates),
            min((c.minimum_clearance_cm for c in candidates),default=math.inf),min((c.minimum_obstacle_clearance_cm for c in candidates),default=math.inf),min((c.minimum_wall_clearance_cm for c in candidates),default=math.inf),
            best.candidate_id if best else None,None,command.steering_angle_deg,command.target_speed_cm_s,self.geometry.turning_radius_cm(command.steering_angle_deg),straight_projection_safe,
            "straight_clear" if best is straight else "route_or_avoidance_candidate" if best else "no_safe_trajectory")
        diagnostics.budget_exhausted=budget_exhausted
        diagnostics.budget_reason="max_planning_time_ms" if budget_exhausted else None
        return PlannerResult(command,state,candidates,best,diagnostics,diagnostics.reason)

    def collision_metrics(self,state:VehicleState,data:PlannerInput)->tuple[bool,float,float,float]:
        return self._clearance(self.geometry.footprint(state),data)


def timing_percentiles(values:Sequence[float])->dict[str,float]:
    if not values:return {k:0.0 for k in ("p50_ms","p90_ms","p95_ms","p99_ms","max_ms")}
    ordered=sorted(values)
    def pct(f:float)->float:
        i=(len(ordered)-1)*f;lo,hi=math.floor(i),math.ceil(i);return ordered[lo] if lo==hi else ordered[lo]+(ordered[hi]-ordered[lo])*(i-lo)
    return {"p50_ms":pct(.5),"p90_ms":pct(.9),"p95_ms":pct(.95),"p99_ms":pct(.99),"max_ms":max(ordered)}
