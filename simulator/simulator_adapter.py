"""Adaptador entre el mundo 2D del simulador y el controller autónomo puro."""
from __future__ import annotations

import math
from enum import Enum, auto
from typing import Protocol

try:
    from autonomous_controller import AutonomousController
    from geometric_planner import (
        GeometricPlanner, PlannerInput, PlannerResult, PlannerState,
        TrackDirection, VehicleGeometry, VehicleState, VisibleObstacle, VisibleWall,
        normalize_angle, rectangle_polygon,
    )
    from planner_rules import FIXED_RULES
    from planner_tuning import PlannerTuning
except ImportError:
    from simulator.autonomous_controller import AutonomousController
    from simulator.geometric_planner import (
        GeometricPlanner, PlannerInput, PlannerResult, PlannerState,
        TrackDirection, VehicleGeometry, VehicleState, VisibleObstacle, VisibleWall,
        normalize_angle, rectangle_polygon,
    )
    from simulator.planner_rules import FIXED_RULES
    from simulator.planner_tuning import PlannerTuning


class AvoidState(Enum):
    FOLLOW = auto()
    EMERGENCY_STOP = auto()


class VehicleLike(Protocol):
    x: float; y: float; heading: float; speed_cm_s: float; steering_deg: float
    target_speed_cm_s: float; target_steering_deg: float


class ObstacleLike(Protocol):
    x: float; y: float; color: str; passed: bool


def _in_fov(vehicle: VehicleLike, point: tuple[float, float], fov_deg: float, range_cm: float = 120.0) -> bool:
    dx, dy = point[0]-vehicle.x, point[1]-vehicle.y
    return math.hypot(dx,dy) <= range_cm and abs(normalize_angle(math.atan2(dy,dx)-vehicle.heading)) <= math.radians(fov_deg)/2


def _rect_segments(rect: tuple[float,float,float,float], prefix: str) -> tuple[VisibleWall,...]:
    polygon=rectangle_polygon(rect)
    return tuple(VisibleWall(f"{prefix}-{index}",a,b) for index,(a,b) in enumerate(zip(polygon,polygon[1:]+polygon[:1])))


class SimulatorAutonomousAdapter:
    def __init__(self, tuning: PlannerTuning, fov_deg: float,
                 outer_rect: tuple[float,float,float,float], inner_rect: tuple[float,float,float,float],
                 obstacle_size_cm: float = FIXED_RULES.default_obstacle_width_cm) -> None:
        geometry=VehicleGeometry(fixed_speed_cm_s=tuning.fixed_speed_cm_s,
            max_acceleration_cm_s2=tuning.max_acceleration_cm_s2,
            max_deceleration_cm_s2=tuning.max_deceleration_cm_s2,
            max_steering_rate_deg_s=tuning.max_steering_rate_deg_s)
        self.controller=AutonomousController(GeometricPlanner(geometry,tuning))
        self.planner=self.controller.planner
        self.fov_deg=fov_deg; self.outer_rect=outer_rect; self.inner_rect=inner_rect
        self.obstacle_size_cm=obstacle_size_cm; self.replanning_period_s=tuning.replanning_period_s
        self.elapsed_s=0.0; self.next_replan_s=0.0; self.latest_result:PlannerResult|None=None
        self.route_points:list[tuple[float,float]]=[]; self.track_direction:TrackDirection|None=None
        self.state=AvoidState.FOLLOW; self.planner_state=PlannerState.FOLLOW.value
        self.planning_phase="FOLLOW"; self.side=0.0

    def reset(self)->None:
        self.controller.reset(); self.elapsed_s=self.next_replan_s=0.0; self.latest_result=None
        self.track_direction=None; self.route_points=[]
        self.state=AvoidState.FOLLOW; self.planner_state=PlannerState.FOLLOW.value; self.planning_phase="FOLLOW"; self.side=0.0

    def set_track_direction(self,direction:TrackDirection,route_points:list[tuple[float,float]])->None:
        self.track_direction=direction; self.route_points=route_points

    def _desired_heading(self,vehicle:VehicleLike)->float|None:
        if len(self.route_points)<2:return None
        index=min(range(len(self.route_points)),key=lambda i:math.dist((vehicle.x,vehicle.y),self.route_points[i]))
        target_index=index
        lookahead_cm=max(self.planner.geometry.minimum_left_radius_cm,
                         self.planner.geometry.minimum_right_radius_cm)
        traveled=0.0
        while target_index+1<len(self.route_points) and traveled<lookahead_cm:
            traveled+=math.dist(self.route_points[target_index],self.route_points[target_index+1])
            target_index+=1
        target=self.route_points[target_index]
        return math.atan2(target[1]-vehicle.y,target[0]-vehicle.x)

    def _input(self,vehicle:VehicleLike,obstacles:list[ObstacleLike])->PlannerInput:
        visible=tuple(VisibleObstacle(str(i),o.x,o.y,self.obstacle_size_cm,self.obstacle_size_cm,o.color)
                      for i,o in enumerate(obstacles,1) if not o.passed and _in_fov(vehicle,(o.x,o.y),self.fov_deg))
        all_walls=_rect_segments(self.outer_rect,"outer")+_rect_segments(self.inner_rect,"inner")
        walls=tuple(w for w in all_walls if _in_fov(vehicle,w.start,self.fov_deg) or _in_fov(vehicle,w.end,self.fov_deg)
                    or _in_fov(vehicle,((w.start[0]+w.end[0])/2,(w.start[1]+w.end[1])/2),self.fov_deg))
        return PlannerInput(VehicleState(vehicle.x,vehicle.y,vehicle.heading,vehicle.speed_cm_s,0,vehicle.steering_deg,self.elapsed_s),
            visible,walls,rectangle_polygon(self.outer_rect),self.track_direction,self._desired_heading(vehicle),self.elapsed_s)

    def update(self,vehicle:VehicleLike,obstacles:list[ObstacleLike],dt:float)->None:
        self.elapsed_s+=dt
        if self.latest_result is not None and self.elapsed_s<self.next_replan_s:
            command=self.latest_result.command
        else:
            self.latest_result=self.controller.plan(self._input(vehicle,obstacles)); command=self.latest_result.command
            self.next_replan_s=self.elapsed_s+self.replanning_period_s
        vehicle.target_speed_cm_s=command.target_speed_cm_s; vehicle.target_steering_deg=command.steering_angle_deg
        result=self.latest_result
        if result:
            self.planner_state=result.state.value; self.planning_phase=result.state.value
            selected=result.best_candidate.candidate_id if result.best_candidate else ""
            self.side=1.0 if selected.endswith("RIGHT") else -1.0 if selected.endswith("LEFT") else 0.0
            self.state=AvoidState.EMERGENCY_STOP if result.state is PlannerState.NO_SAFE_TRAJECTORY else AvoidState.FOLLOW

    def take_line_transition_event(self): return None
    def aligned_after_line(self,vehicle:VehicleLike)->bool:
        desired=self._desired_heading(vehicle)
        return desired is None or abs(normalize_angle(desired-vehicle.heading))<=math.radians(FIXED_RULES.route_alignment_tolerance_deg)
