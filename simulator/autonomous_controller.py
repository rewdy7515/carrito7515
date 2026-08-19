"""Política autónoma pura: PlannerInput -> PlannerResult.

Este módulo no conoce Pygame, cámaras, GPIO, motores, servos ni la pista del
simulador. El commitment se conserva por identificador de trayectoria.
"""
from __future__ import annotations

try:
    from geometric_planner import GeometricPlanner, PlannerInput, PlannerResult, PlannerState
except ImportError:
    from simulator.geometric_planner import GeometricPlanner, PlannerInput, PlannerResult, PlannerState


class AutonomousController:
    def __init__(self, planner: GeometricPlanner | None = None) -> None:
        self.planner = planner or GeometricPlanner()
        self.committed_candidate_id: str | None = None

    def reset(self) -> None:
        self.committed_candidate_id = None

    def plan(self, planner_input: PlannerInput) -> PlannerResult:
        result = self.planner.plan(planner_input)
        if result.state is PlannerState.FOLLOW:
            self.committed_candidate_id = None
            return result

        committed = next(
            (candidate for candidate in result.candidates
             if candidate.safe and candidate.candidate_id == self.committed_candidate_id),
            None,
        )
        if committed is not None:
            result.best_candidate = committed
            result.command = self.planner.command_for(committed)
            result.diagnostics.selected_candidate_id = committed.candidate_id
            result.diagnostics.selected_angle_deg = result.command.steering_angle_deg
            result.diagnostics.selected_speed_cm_s = result.command.target_speed_cm_s
            result.diagnostics.selected_radius_cm = self.planner.geometry.turning_radius_cm(
                result.command.steering_angle_deg
            )
            result.reason = "committed_safe_trajectory"
        elif result.best_candidate is not None:
            self.committed_candidate_id = result.best_candidate.candidate_id
        else:
            self.committed_candidate_id = None
        result.diagnostics.committed_candidate_id = self.committed_candidate_id
        return result
