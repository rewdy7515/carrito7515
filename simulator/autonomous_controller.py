"""Estado autónomo puro: objetivo activo y commitment de trayectoria."""
from __future__ import annotations

import math
from dataclasses import replace

try:
    from geometric_planner import (
        CandidateTrajectory, GeometricPlanner, MotionPrimitive, PlannerInput,
        PlannerResult, PlannerState, PrimitiveType, VehicleState,
        VisibleObstacle,
    )
except ImportError:
    from simulator.geometric_planner import (
        CandidateTrajectory, GeometricPlanner, MotionPrimitive, PlannerInput,
        PlannerResult, PlannerState, PrimitiveType, VehicleState,
        VisibleObstacle,
    )


class AutonomousController:
    """Gestiona un commitment flexible sobre planes completos de 50 cm.

    En cada llamada se valida el resto del plan comprometido y se genera un
    nuevo plan desde la pose actual. El plan actual se conserva salvo que deje
    de ser seguro o que el nuevo score lo supere por ``switch_margin``.
    El controller no inventa giros: solo compara y ejecuta primitivas del
    planner.
    """

    def __init__(self, planner: GeometricPlanner | None = None) -> None:
        self.planner = planner or GeometricPlanner()
        self.active_target_id: str | None = None
        self._committed_id: str | None = None
        self._committed_profile = None
        self._committed_target_id: str | None = None
        self._committed_pass_side = None
        self._remaining_primitives: list[MotionPrimitive] = []
        self._last_position: tuple[float, float] | None = None
        self._tracked_obstacles: dict[str, VisibleObstacle] = {}

    def reset(self) -> None:
        self.active_target_id = None
        self._tracked_obstacles.clear()
        self._clear_commitment()

    def _clear_commitment(self) -> None:
        self._committed_id = None
        self._committed_profile = None
        self._committed_target_id = None
        self._committed_pass_side = None
        self._remaining_primitives = []
        self._last_position = None

    def _remember_obstacles(self, planner_input: PlannerInput) -> PlannerInput:
        """Conserva las detecciones para no perder el orden de prioridad."""
        self._tracked_obstacles.update(
            {item.object_id: item for item in planner_input.visible_obstacles}
        )
        return replace(
            planner_input,
            tracked_obstacles=tuple(self._tracked_obstacles.values()),
        )

    def _update_active_target(self, planner_input: PlannerInput) -> PlannerInput:
        data = self._remember_obstacles(planner_input)
        if self.active_target_id is None:
            return data
        target = next(
            (obstacle for obstacle in data.tracked_obstacles
             if obstacle.object_id == self.active_target_id),
            None,
        )
        if target is not None and (
            target.already_passed
            or self.planner.obstacle_passed_now(data, target)
        ):
            self.active_target_id = None
            self._clear_commitment()
        return data

    def _advance_commitment(self, state: VehicleState) -> None:
        if self._last_position is None:
            self._last_position = (state.x_cm, state.y_cm)
            return
        distance = math.dist(self._last_position, (state.x_cm, state.y_cm))
        self._last_position = (state.x_cm, state.y_cm)
        while self._remaining_primitives and distance > 1e-9:
            current = self._remaining_primitives[0]
            if distance + 1e-9 < current.distance_cm:
                self._remaining_primitives[0] = replace(
                    current, distance_cm=current.distance_cm - distance,
                )
                return
            distance -= current.distance_cm
            self._remaining_primitives.pop(0)

    def _remaining_candidate(self) -> CandidateTrajectory | None:
        if not self._committed_id or not self._remaining_primitives:
            return None
        return CandidateTrajectory(
            self._committed_id,
            self._committed_profile,
            tuple(self._remaining_primitives),
            target_obstacle_id=self._committed_target_id,
            desired_pass_side=self._committed_pass_side,
        )

    def execution_horizon_cm(self, state: VehicleState) -> float:
        """Calcula cuánto se ejecuta antes de volver a comparar planes."""
        tuning = self.planner.tuning
        # Estos parámetros ya están expresados como distancia. No se deben
        # multiplicar por la velocidad: eso convertiría, por ejemplo, 6 cm a
        # 144 cm a 24 cm/s y forzaría siempre el máximo de 15 cm.
        return min(tuning.execution_horizon_min_cm, tuning.execution_horizon_max_cm)

    def execution_interval_s(
        self, state: VehicleState, command_speed_cm_s: float,
    ) -> float:
        """Convierte distancia y periodo mínimo en intervalo de simulación."""
        speed = max(abs(state.speed_cm_s), abs(command_speed_cm_s), 1e-9)
        distance_interval = self.execution_horizon_cm(state) / speed
        return max(self.planner.tuning.replanning_period_s, distance_interval)

    def _should_switch(
        self,
        current: CandidateTrajectory | None,
        new: CandidateTrajectory | None,
    ) -> bool:
        """Decide el cambio sin usar diferencias pequeñas de score."""
        if current is None:
            return new is not None
        if new is None or not new.safe:
            return False
        if not current.safe:
            return True
        return new.score > current.score + self.planner.tuning.switch_margin

    def _comparison_result(
        self,
        result: PlannerResult,
        current_result: PlannerResult | None,
        current: CandidateTrajectory | None,
        switched: bool,
        state: VehicleState,
    ) -> PlannerResult:
        """Expone en el resultado el plan retenido y la comparación realizada."""
        diagnostics = result.diagnostics
        diagnostics.commitment_mode = "flexible"
        diagnostics.current_plan_score = current.score if current else -math.inf
        diagnostics.new_plan_score = (
            result.best_candidate.score
            if result.best_candidate and result.best_candidate.safe
            else -math.inf
        )
        diagnostics.switch_margin = self.planner.tuning.switch_margin
        diagnostics.switched_plan = switched
        diagnostics.execution_horizon_cm = self.execution_horizon_cm(state)

        if current is not None:
            # Se muestran las alternativas nuevas junto con el resto del plan
            # comprometido para poder auditar por qué se conservó o cambió.
            result.candidates = [current, *result.candidates]
            result.diagnostics.candidates_generated += 1
            result.diagnostics.candidates_evaluated += 1

        if current_result is not None and not switched and current is not None:
            result.best_candidate = current
            result.command = self.planner.command_for(current)
            result.state = current_result.state
            result.reason = "flexible_commitment_kept"
            diagnostics.reason = result.reason
            diagnostics.selected_candidate_id = current.candidate_id
            diagnostics.committed_candidate_id = current.candidate_id
            diagnostics.selected_angle_deg = result.command.steering_angle_deg
            diagnostics.selected_speed_cm_s = result.command.target_speed_cm_s
            diagnostics.selected_radius_cm = self.planner.geometry.turning_radius_cm(
                result.command.steering_angle_deg,
            )
            diagnostics.no_safe_reason = None
            diagnostics.no_safe_detail = None
        elif switched:
            diagnostics.reason = "flexible_commitment_switched"
            result.reason = diagnostics.reason
        return result

    def _commit(self, result: PlannerResult, state: VehicleState) -> None:
        candidate = result.best_candidate
        if candidate is None:
            self._clear_commitment()
            return
        self._committed_id = candidate.candidate_id
        self._committed_profile = candidate.profile
        self._committed_target_id = candidate.target_obstacle_id
        self._committed_pass_side = candidate.desired_pass_side
        self._remaining_primitives = list(candidate.primitives)
        self._last_position = (state.x_cm, state.y_cm)
        result.diagnostics.committed_candidate_id = candidate.candidate_id

    def plan(self, planner_input: PlannerInput) -> PlannerResult:
        data = self._update_active_target(planner_input)
        data = replace(data, active_target_id=self.active_target_id)

        self._advance_commitment(data.vehicle_state)
        remaining = self._remaining_candidate()
        committed = (
            self.planner.revalidate_committed(data, remaining)
            if remaining is not None else None
        )
        current = committed.best_candidate if committed else None

        # Aunque el commitment siga siendo seguro, se calcula una nueva
        # trayectoria completa para poder compararla contra su parte restante.
        result = self.planner.plan(data)
        if self.active_target_id is None and result.diagnostics.target_obstacle_id:
            self.active_target_id = result.diagnostics.target_obstacle_id
        result.diagnostics.active_target_id = self.active_target_id
        new = result.best_candidate if result.best_candidate and result.best_candidate.safe else None
        switched = self._should_switch(current, new)

        if current is None or switched:
            if current is not None:
                self._clear_commitment()
            self._commit(result, data.vehicle_state)
        else:
            # El nuevo plan no supera el margen: conservar el resto del plan
            # evita oscilaciones aun cuando aparezca una alternativa cercana.
            result.diagnostics.active_target_id = self.active_target_id

        return self._comparison_result(
            result, committed, current, switched, data.vehicle_state,
        )
