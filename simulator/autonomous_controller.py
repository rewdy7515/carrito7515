"""Estado autónomo puro: objetivo activo y commitment de trayectoria."""
from __future__ import annotations

import math
from dataclasses import replace

try:
    from geometric_planner import (
        CandidateTrajectory, GeometricPlanner, MotionPrimitive, ObstaclePassState, PlannerInput,
        PlannerResult, PlannerState, PrimitiveType, VehicleState,
        VisibleObstacle,
    )
except ImportError:
    from simulator.geometric_planner import (
        CandidateTrajectory, GeometricPlanner, MotionPrimitive, ObstaclePassState, PlannerInput,
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
        self._last_result: PlannerResult | None = None
        self._tracked_obstacles: dict[str, VisibleObstacle] = {}
        self._tracked_seen_at: dict[str, float] = {}
        self._obstacle_states: dict[str, ObstaclePassState] = {}
        self._last_switch_reason: str | None = None

    def reset(self) -> None:
        self.active_target_id = None
        self._tracked_obstacles.clear()
        self._tracked_seen_at.clear()
        self._obstacle_states.clear()
        self._last_switch_reason = None
        self._last_result = None
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
        for item in planner_input.visible_obstacles:
            state = self._obstacle_states.get(item.object_id, item.pass_state)
            terminal = state in {
                ObstaclePassState.PASSED_CORRECT,
                ObstaclePassState.PASSED_WRONG,
            }
            self._tracked_obstacles[item.object_id] = replace(
                item, pass_state=state, already_passed=item.already_passed or terminal,
            )
        for item in planner_input.visible_obstacles:
            self._tracked_seen_at[item.object_id] = planner_input.timestamp_s
        timeout = self.planner.tuning.memory_timeout_s
        expired = [
            object_id for object_id, last_seen in self._tracked_seen_at.items()
            if object_id != self.active_target_id
            and planner_input.timestamp_s - last_seen > timeout
        ]
        for object_id in expired:
            self._tracked_seen_at.pop(object_id, None)
            self._tracked_obstacles.pop(object_id, None)
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
        if target is not None and target.already_passed:
            self._obstacle_states[target.object_id] = target.pass_state
            terminal = replace(
                target, already_passed=True, pass_state=target.pass_state,
            )
            self.active_target_id = None
            self._clear_commitment()
            return self._replace_obstacle(data, terminal)
        elif target is not None and self.planner.obstacle_passed_now(data, target):
            side = self.planner.obstacle_pass_side_now(data, target)
            required = "RIGHT" if target.color.lower() == "red" else "LEFT"
            state = (
                ObstaclePassState.PASSED_CORRECT
                if side.value == required else ObstaclePassState.PASSED_WRONG
            )
            self._obstacle_states[target.object_id] = state
            self._tracked_obstacles[target.object_id] = replace(
                target, already_passed=True, pass_state=state,
            )
            self.active_target_id = None
            self._clear_commitment()
            return self._replace_obstacle(
                data, self._tracked_obstacles[target.object_id],
            )
        return data

    def _replace_obstacle(
        self, data: PlannerInput, replacement: VisibleObstacle,
    ) -> PlannerInput:
        """Publica el estado terminal en visible y tracked en este ciclo."""
        visible = tuple(
            replacement if item.object_id == replacement.object_id else item
            for item in data.visible_obstacles
        )
        tracked = tuple(
            replacement if item.object_id == replacement.object_id else item
            for item in data.tracked_obstacles
        )
        return replace(data, visible_obstacles=visible, tracked_obstacles=tracked)

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
        # El intervalo temporal de replanning es independiente. Aquí solo se
        # decide la distancia que se ejecuta antes de comparar otra vez.
        result = self._last_result
        diagnostics = result.diagnostics if result else None
        has_target = bool(self.active_target_id or self._committed_target_id)
        if not has_target:
            return tuning.execution_horizon_max_cm
        clearance = diagnostics.minimum_clearance_cm if diagnostics else math.inf
        preferred = max(tuning.preferred_clearance_cm, 1e-9)
        risk = 0.0 if not math.isfinite(clearance) else max(
            0.0, min(1.0, (preferred - clearance) / preferred)
        )
        return tuning.execution_horizon_max_cm - risk * (
            tuning.execution_horizon_max_cm - tuning.execution_horizon_min_cm
        )

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
        self._last_switch_reason = None
        if current is None:
            self._last_switch_reason = "no_committed_plan"
            return new is not None
        if new is None or not new.safe:
            self._last_switch_reason = "new_plan_not_safe"
            return False
        if not current.safe:
            self._last_switch_reason = "committed_not_safe"
            return True
        if not current.future_pass_viable and new.future_pass_viable:
            self._last_switch_reason = "committed_future_pass_not_viable"
            return True
        current_horizon = max(current.horizon_cm or current.length_cm, 1.0)
        new_horizon = max(new.horizon_cm or new.length_cm, 1.0)
        current_normalized = current.score / current_horizon
        new_normalized = new.score / new_horizon
        normalized_margin = self.planner.tuning.switch_margin / current_horizon
        if new_normalized > current_normalized + normalized_margin:
            self._last_switch_reason = "new_plan_exceeds_normalized_switch_margin"
            return True
        self._last_switch_reason = "switch_margin_kept"
        return False

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
        diagnostics.switch_reason = self._last_switch_reason
        diagnostics.execution_horizon_cm = self.execution_horizon_cm(state)
        diagnostics.committed_horizon_cm = (
            current.horizon_cm if current else 0.0
        )
        diagnostics.new_horizon_cm = (
            result.best_candidate.horizon_cm
            if result.best_candidate else 0.0
        )
        diagnostics.committed_future_pass_viable = (
            current.future_pass_viable if current else None
        )
        diagnostics.new_future_pass_viable = (
            result.best_candidate.future_pass_viable
            if result.best_candidate else None
        )

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
            self._obstacle_states.setdefault(
                self.active_target_id, ObstaclePassState.APPROACHING,
            )
        result.diagnostics.active_target_id = self.active_target_id
        new = result.best_candidate if result.best_candidate and result.best_candidate.safe else None
        if self.active_target_id is not None and new is not None:
            state = new.pass_state
            if state in {
                ObstaclePassState.APPROACHING,
                ObstaclePassState.PASSING,
            }:
                self._obstacle_states[self.active_target_id] = state
        switched = self._should_switch(current, new)

        if current is None or switched:
            if current is not None:
                self._clear_commitment()
            self._commit(result, data.vehicle_state)
        else:
            # El nuevo plan no supera el margen: conservar el resto del plan
            # evita oscilaciones aun cuando aparezca una alternativa cercana.
            result.diagnostics.active_target_id = self.active_target_id

        compared = self._comparison_result(
            result, committed, current, switched, data.vehicle_state,
        )
        self._last_result = compared
        return compared
