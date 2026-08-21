"""Puntuación de trayectorias, separada de la geometría del planner."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class TrajectoryScoreWeights:
    """Pesos ajustables para comparar candidatos ya validados."""

    clearance_reward: float = 12.0
    progress_reward: float = 2.0
    heading_error_penalty: float = 2.5
    steering_effort_penalty: float = 0.01
    steering_changes_penalty: float = 0.3
    length_penalty: float = 0.05
    reverse_distance_penalty: float = 2.0
    collision_penalty: float = 1_000_000.0
    # Factores separados para poder ajustar muros y obstáculos sin perder
    # ``clearance_reward`` como peso base histórico.
    wall_clearance_weight: float = 0.5
    obstacle_clearance_weight: float = 0.5
    clearance_violation_penalty: float = 25.0


DEFAULT_SCORE_WEIGHTS = TrajectoryScoreWeights()


@dataclass(frozen=True)
class TrajectoryScoreBreakdown:
    """Desglose auditable del score soft de una trayectoria."""

    raw_score: float
    final_score: float
    score_clearance: float
    score_wall_clearance: float
    score_obstacle_clearance: float
    score_progress: float
    score_heading: float
    score_steering: float
    score_steering_changes: float
    score_length: float
    score_pass_progress: float
    score_wrong_pass_side: float
    score_wrong_pass_side_base: float
    score_wrong_pass_side_priority_adjustment: float
    score_reverse: float
    min_wall_clearance: float
    min_obstacle_clearance: float
    safety_margin_violation: float
    pass_side_adjustment: float = 0.0


def score_trajectory(
    *,
    minimum_clearance_cm: float,
    preferred_clearance_cm: float,
    minimum_wall_clearance_cm: float | None = None,
    minimum_obstacle_clearance_cm: float | None = None,
    wall_hard_clearance_cm: float = 0.0,
    obstacle_hard_clearance_cm: float = 0.0,
    wall_target_clearance_cm: float | None = None,
    obstacle_target_clearance_cm: float | None = None,
    progress_cm: float,
    final_heading_error_deg: float,
    steering_effort: float,
    steering_changes: float,
    length_cm: float,
    reverse_distance_cm: float,
    physical_collision: bool,
    allow_collisions: bool = False,
    pass_progress_cm: float = 0.0,
    pass_progress_ratio: float | None = None,
    pass_progress_reward: float = 0.0,
    wrong_pass_side: bool = False,
    wrong_pass_side_penalty: float = 0.0,
    pass_side_adjustment: float = 0.0,
    weights: TrajectoryScoreWeights = DEFAULT_SCORE_WEIGHTS,
) -> float:
    """Devuelve el soft score de una trayectoria ya validada.

    Las restricciones duras deben invalidarse antes de llamar a esta función.
    Una colisión predicha contra un obstáculo solo permanece seleccionable
    cuando ``allow_collisions=True``, reservado para pruebas y diagnóstico;
    el planner invalida antes cualquier choque con muro o salida de pista.
    ``wrong_pass_side`` solo resta el penalty soft; la prioridad geométrica
    adicional se aplica una vez sobre el conjunto de candidatos.
    """
    return score_trajectory_breakdown(
        minimum_clearance_cm=minimum_clearance_cm,
        preferred_clearance_cm=preferred_clearance_cm,
        minimum_wall_clearance_cm=minimum_wall_clearance_cm,
        minimum_obstacle_clearance_cm=minimum_obstacle_clearance_cm,
        wall_hard_clearance_cm=wall_hard_clearance_cm,
        obstacle_hard_clearance_cm=obstacle_hard_clearance_cm,
        wall_target_clearance_cm=wall_target_clearance_cm,
        obstacle_target_clearance_cm=obstacle_target_clearance_cm,
        progress_cm=progress_cm,
        final_heading_error_deg=final_heading_error_deg,
        steering_effort=steering_effort,
        steering_changes=steering_changes,
        length_cm=length_cm,
        reverse_distance_cm=reverse_distance_cm,
        physical_collision=physical_collision,
        allow_collisions=allow_collisions,
        pass_progress_cm=pass_progress_cm,
        pass_progress_ratio=pass_progress_ratio,
        pass_progress_reward=pass_progress_reward,
        wrong_pass_side=wrong_pass_side,
        wrong_pass_side_penalty=wrong_pass_side_penalty,
        pass_side_adjustment=pass_side_adjustment,
        weights=weights,
    ).final_score


def score_trajectory_breakdown(
    *,
    minimum_clearance_cm: float,
    preferred_clearance_cm: float,
    minimum_wall_clearance_cm: float | None = None,
    minimum_obstacle_clearance_cm: float | None = None,
    wall_hard_clearance_cm: float = 0.0,
    obstacle_hard_clearance_cm: float = 0.0,
    wall_target_clearance_cm: float | None = None,
    obstacle_target_clearance_cm: float | None = None,
    progress_cm: float,
    final_heading_error_deg: float,
    steering_effort: float,
    steering_changes: float,
    length_cm: float,
    reverse_distance_cm: float,
    physical_collision: bool,
    allow_collisions: bool = False,
    pass_progress_cm: float = 0.0,
    pass_progress_ratio: float | None = None,
    pass_progress_reward: float = 0.0,
    wrong_pass_side: bool = False,
    wrong_pass_side_penalty: float = 0.0,
    pass_side_adjustment: float = 0.0,
    weights: TrajectoryScoreWeights = DEFAULT_SCORE_WEIGHTS,
) -> TrajectoryScoreBreakdown:
    """Calcula raw/final score; las restricciones hard se validan antes.

    ``wrong_pass_side`` es una penalización soft base y no invalida una
    trayectoria físicamente segura. ``pass_progress_ratio`` está acotado a
    ``[-1, 1]`` y representa progreso firmado hacia la frontera lateral.

    ``pass_side_adjustment`` permite que el planner garantice la prioridad
    reglamentaria después de conocer el conjunto completo de candidatos. No
    elimina candidatos: solo modifica su score final.
    """
    wall_clearance = (
        minimum_clearance_cm
        if minimum_wall_clearance_cm is None
        else minimum_wall_clearance_cm
    )
    obstacle_clearance = (
        minimum_clearance_cm
        if minimum_obstacle_clearance_cm is None
        else minimum_obstacle_clearance_cm
    )
    wall_target = (
        preferred_clearance_cm
        if wall_target_clearance_cm is None
        else wall_target_clearance_cm
    )
    obstacle_target = (
        preferred_clearance_cm
        if obstacle_target_clearance_cm is None
        else obstacle_target_clearance_cm
    )

    def clearance_component(
        minimum: float,
        hard: float,
        target: float,
        factor: float,
    ) -> tuple[float, float]:
        if not math.isfinite(minimum):
            return 0.0, 0.0
        violation = max(0.0, hard - minimum)
        span = max(0.0, target - hard)
        rewarded_distance = min(max(0.0, minimum - hard), span)
        reward = rewarded_distance * weights.clearance_reward * factor
        penalty = violation * weights.clearance_violation_penalty
        return reward - penalty, violation

    score_wall_clearance, wall_violation = clearance_component(
        wall_clearance, wall_hard_clearance_cm, wall_target,
        weights.wall_clearance_weight,
    )
    score_obstacle_clearance, obstacle_violation = clearance_component(
        obstacle_clearance, obstacle_hard_clearance_cm, obstacle_target,
        weights.obstacle_clearance_weight,
    )
    safety_margin_violation = wall_violation + obstacle_violation

    if physical_collision and not allow_collisions:
        return TrajectoryScoreBreakdown(
            raw_score=float("-inf"), final_score=float("-inf"),
            score_clearance=score_wall_clearance + score_obstacle_clearance,
            score_wall_clearance=score_wall_clearance,
            score_obstacle_clearance=score_obstacle_clearance,
            score_heading=0.0, score_steering=0.0,
            score_steering_changes=0.0, score_length=0.0,
            score_pass_progress=0.0, score_wrong_pass_side=0.0,
            score_wrong_pass_side_base=0.0,
            score_wrong_pass_side_priority_adjustment=0.0,
            score_reverse=0.0,
            min_wall_clearance=wall_clearance,
            min_obstacle_clearance=obstacle_clearance,
            safety_margin_violation=safety_margin_violation,
        )

    ratio = pass_progress_ratio
    if ratio is None:
        ratio = max(-1.0, min(1.0, pass_progress_cm))
    ratio = max(-1.0, min(1.0, ratio))
    wrong_pass_side_base = -wrong_pass_side_penalty if wrong_pass_side else 0.0
    wrong_pass_side_priority = -pass_side_adjustment

    components = {
        "clearance": score_wall_clearance + score_obstacle_clearance,
        "progress": progress_cm * weights.progress_reward,
        "heading": -final_heading_error_deg * weights.heading_error_penalty,
        "steering": -steering_effort * weights.steering_effort_penalty,
        "steering_changes": -steering_changes * weights.steering_changes_penalty,
        "length": -length_cm * weights.length_penalty,
        "pass_progress": ratio * pass_progress_reward,
        "wrong_pass_side_base": wrong_pass_side_base,
        "wrong_pass_side_priority": wrong_pass_side_priority,
        "reverse": -reverse_distance_cm * weights.reverse_distance_penalty,
    }
    raw_score = sum(components.values()) - components["wrong_pass_side_base"] - components["wrong_pass_side_priority"]
    final_score = raw_score + components["wrong_pass_side_base"] + components["wrong_pass_side_priority"]
    if physical_collision and allow_collisions:
        final_score -= weights.collision_penalty
    return TrajectoryScoreBreakdown(
        raw_score=raw_score,
        final_score=final_score,
        score_clearance=components["clearance"],
        score_wall_clearance=score_wall_clearance,
        score_obstacle_clearance=score_obstacle_clearance,
        score_progress=components["progress"],
        score_heading=components["heading"],
        score_steering=components["steering"],
        score_steering_changes=components["steering_changes"],
        score_length=components["length"],
        score_pass_progress=components["pass_progress"],
        score_wrong_pass_side=(
            components["wrong_pass_side_base"]
            + components["wrong_pass_side_priority"]
        ),
        score_wrong_pass_side_base=components["wrong_pass_side_base"],
        score_wrong_pass_side_priority_adjustment=components["wrong_pass_side_priority"],
        score_reverse=components["reverse"],
        min_wall_clearance=wall_clearance,
        min_obstacle_clearance=obstacle_clearance,
        safety_margin_violation=safety_margin_violation,
        pass_side_adjustment=pass_side_adjustment,
    )
