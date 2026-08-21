"""Puntuación de trayectorias, separada de la geometría del planner."""

from __future__ import annotations

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


DEFAULT_SCORE_WEIGHTS = TrajectoryScoreWeights()


@dataclass(frozen=True)
class TrajectoryScoreBreakdown:
    """Desglose auditable del score soft de una trayectoria."""

    raw_score: float
    final_score: float
    score_clearance: float
    score_progress: float
    score_heading: float
    score_steering: float
    score_steering_changes: float
    score_length: float
    score_pass_progress: float
    score_wrong_pass_side: float
    score_reverse: float
    pass_side_adjustment: float = 0.0


def score_trajectory(
    *,
    minimum_clearance_cm: float,
    preferred_clearance_cm: float,
    progress_cm: float,
    final_heading_error_deg: float,
    steering_effort: float,
    steering_changes: float,
    length_cm: float,
    reverse_distance_cm: float,
    physical_collision: bool,
    allow_collisions: bool = False,
    pass_progress_cm: float = 0.0,
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
    """
    return score_trajectory_breakdown(
        minimum_clearance_cm=minimum_clearance_cm,
        preferred_clearance_cm=preferred_clearance_cm,
        progress_cm=progress_cm,
        final_heading_error_deg=final_heading_error_deg,
        steering_effort=steering_effort,
        steering_changes=steering_changes,
        length_cm=length_cm,
        reverse_distance_cm=reverse_distance_cm,
        physical_collision=physical_collision,
        allow_collisions=allow_collisions,
        pass_progress_cm=pass_progress_cm,
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
    progress_cm: float,
    final_heading_error_deg: float,
    steering_effort: float,
    steering_changes: float,
    length_cm: float,
    reverse_distance_cm: float,
    physical_collision: bool,
    allow_collisions: bool = False,
    pass_progress_cm: float = 0.0,
    pass_progress_reward: float = 0.0,
    wrong_pass_side: bool = False,
    wrong_pass_side_penalty: float = 0.0,
    pass_side_adjustment: float = 0.0,
    weights: TrajectoryScoreWeights = DEFAULT_SCORE_WEIGHTS,
) -> TrajectoryScoreBreakdown:
    """Calcula raw/final score; las restricciones hard se validan antes.

    ``pass_side_adjustment`` permite que el planner garantice la prioridad
    reglamentaria después de conocer el conjunto completo de candidatos. No
    elimina candidatos: solo modifica su score final.
    """
    if physical_collision and not allow_collisions:
        return TrajectoryScoreBreakdown(
            float("-inf"), float("-inf"), 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0,
        )

    components = {
        "clearance": min(minimum_clearance_cm, preferred_clearance_cm) * weights.clearance_reward,
        "progress": progress_cm * weights.progress_reward,
        "heading": -final_heading_error_deg * weights.heading_error_penalty,
        "steering": -steering_effort * weights.steering_effort_penalty,
        "steering_changes": -steering_changes * weights.steering_changes_penalty,
        "length": -length_cm * weights.length_penalty,
        "pass_progress": max(0.0, pass_progress_cm) * pass_progress_reward,
        "wrong_pass_side": -wrong_pass_side_penalty if wrong_pass_side else 0.0,
        "reverse": -reverse_distance_cm * weights.reverse_distance_penalty,
    }
    raw_score = sum(components.values()) - components["wrong_pass_side"]
    final_score = raw_score + components["wrong_pass_side"] - pass_side_adjustment
    if physical_collision and allow_collisions:
        final_score -= weights.collision_penalty
    return TrajectoryScoreBreakdown(
        raw_score=raw_score,
        final_score=final_score,
        score_clearance=components["clearance"],
        score_progress=components["progress"],
        score_heading=components["heading"],
        score_steering=components["steering"],
        score_steering_changes=components["steering_changes"],
        score_length=components["length"],
        score_pass_progress=components["pass_progress"],
        score_wrong_pass_side=components["wrong_pass_side"],
        score_reverse=components["reverse"],
        pass_side_adjustment=pass_side_adjustment,
    )
