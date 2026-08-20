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
    weights: TrajectoryScoreWeights = DEFAULT_SCORE_WEIGHTS,
) -> float:
    """Devuelve el soft score de una trayectoria ya validada.

    Las restricciones duras deben invalidarse antes de llamar a esta función.
    Una colisión predicha contra un obstáculo solo permanece seleccionable
    cuando ``allow_collisions=True``, reservado para pruebas y diagnóstico;
    el planner invalida antes cualquier choque con muro o salida de pista.
    """
    if physical_collision and not allow_collisions:
        return float("-inf")

    score = (
        min(minimum_clearance_cm, preferred_clearance_cm)
        * weights.clearance_reward
        + progress_cm * weights.progress_reward
        - final_heading_error_deg * weights.heading_error_penalty
        - steering_effort * weights.steering_effort_penalty
        - steering_changes * weights.steering_changes_penalty
        - length_cm * weights.length_penalty
        - reverse_distance_cm * weights.reverse_distance_penalty
    )
    if physical_collision and allow_collisions:
        score -= weights.collision_penalty
    return score
