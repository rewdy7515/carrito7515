"""Parámetros que solo cambian la decisión de trayectoria.

``planner_rules.py`` contiene medidas, dinámica, clearance hard y reglas
fijas. Este módulo contiene búsqueda, compromiso, memoria, clearance preferred
y pesos soft. ``PlannerTuning`` queda listo para ser modificado por un
optimizador sin tocar la física del vehículo.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, fields, replace
from pathlib import Path
from typing import Any

try:
    from planner_rules import FIXED_RULES
    from trajectory_scorer import DEFAULT_SCORE_WEIGHTS, TrajectoryScoreWeights
except ImportError:
    from simulator.planner_rules import FIXED_RULES
    from simulator.trajectory_scorer import DEFAULT_SCORE_WEIGHTS, TrajectoryScoreWeights


DEFAULT_TUNING_FILE = Path(__file__).resolve().parents[1] / "config" / "simulator_planner_tuning.json"


@dataclass(frozen=True)
class PreferredSafetyMargins:
    """Clearance preferido: solo modifica la puntuación."""

    front_cm: float = 12.0
    side_cm: float = 5.0
    rear_cm: float = 5.0

    def validate(self) -> "PreferredSafetyMargins":
        if any(value < 0.0 for value in (self.front_cm, self.side_cm, self.rear_cm)):
            raise ValueError("Los margenes preferred no pueden ser negativos")
        return self


@dataclass(frozen=True)
class PlannerTuning:
    """Únicamente parámetros TUNABLE que afectan la elección del trayecto."""

    planning_horizon_cm: float = 50.0
    prediction_segments: int = 3
    beam_width: int = 4
    # Fracciones del límite físico usadas por la única familia de generación.
    steering_fractions: tuple[float, ...] = (0.5, 1.0)
    post_pass_margin_cm: float = 10.0
    pass_progress_reward: float = 8.0
    wrong_pass_side_penalty: float = 100.0
    reverse_step_cm: float = 4.0
    max_reverse_recovery_cm: float = 16.0
    forward_projection_cm: float = 30.0
    route_alignment_tolerance_deg: float = 8.0
    replanning_period_s: float = 0.20
    preview_horizon_s: float = 5.0
    execution_horizon_min_cm: float = 6.0
    execution_horizon_max_cm: float = 15.0
    switch_margin: float = 8.0
    memory_timeout_s: float = 2.0
    preferred_safety_margins: PreferredSafetyMargins = PreferredSafetyMargins()
    # Pueden cambiar la decisión si se agotan; no son redundantes del beam.
    planning_budget_mode: str = "time"
    max_candidates: int = 256
    max_planning_time_ms: float = 20.0
    diagnostic_level: str = "full"
    score_weights: TrajectoryScoreWeights = DEFAULT_SCORE_WEIGHTS

    # Modos operativos explícitos de pruebas/diagnóstico; no se serializan
    # como tuning optimizable.
    allow_physical_collisions: bool = False
    disable_hard_safety_margins: bool = False

    def validate(self) -> "PlannerTuning":
        self.preferred_safety_margins.validate()
        if self.planning_horizon_cm <= 0.0:
            raise ValueError("planning_horizon_cm debe ser positivo")
        if self.prediction_segments <= 0 or self.beam_width <= 0:
            raise ValueError("prediction_segments y beam_width deben ser positivos")
        if not self.steering_fractions or any(
            fraction <= 0.0 or fraction > 1.0
            for fraction in self.steering_fractions
        ):
            raise ValueError("steering_fractions debe estar entre 0 y 1")
        if self.post_pass_margin_cm < 0.0:
            raise ValueError("post_pass_margin_cm no puede ser negativo")
        if self.pass_progress_reward < 0.0 or self.wrong_pass_side_penalty < 0.0:
            raise ValueError("Los pesos de pass side no pueden ser negativos")
        if self.reverse_step_cm <= 0.0 or self.max_reverse_recovery_cm < self.reverse_step_cm:
            raise ValueError("La recuperación reverse incremental es inválida")
        if self.forward_projection_cm <= 0.0 or self.route_alignment_tolerance_deg < 0.0:
            raise ValueError("Los parámetros de ayuda de ruta son inválidos")
        if self.replanning_period_s <= 0.0:
            raise ValueError("replanning_period_s debe ser positivo")
        if self.preview_horizon_s <= 0.0:
            raise ValueError("preview_horizon_s debe ser positivo")
        if self.execution_horizon_min_cm <= 0.0:
            raise ValueError("execution_horizon_min_cm debe ser positivo")
        if self.execution_horizon_max_cm < self.execution_horizon_min_cm:
            raise ValueError("execution_horizon_max_cm debe ser >= execution_horizon_min_cm")
        if self.switch_margin < 0.0 or self.memory_timeout_s < 0.0:
            raise ValueError("switch_margin y memory_timeout_s no pueden ser negativos")
        if self.diagnostic_level not in {"full", "summary", "off"}:
            raise ValueError("diagnostic_level debe ser 'full', 'summary' u 'off'")
        if self.max_candidates <= 0 or self.max_planning_time_ms <= 0.0:
            raise ValueError("Presupuesto de planificación inválido")
        if self.planning_budget_mode not in {"time", "candidate_count"}:
            raise ValueError("planning_budget_mode debe ser 'time' o 'candidate_count'")
        return self

    @property
    def preferred_clearance_cm(self) -> float:
        """Alias de reporte; no es un parámetro almacenado adicional."""
        return max(
            self.preferred_safety_margins.front_cm,
            self.preferred_safety_margins.side_cm,
            self.preferred_safety_margins.rear_cm,
        )

    @property
    def mandatory_clearance_cm(self) -> float:
        """Alias de reporte para la mayor regla hard fija."""
        return max(
            FIXED_RULES.hard_front_clearance_cm,
            FIXED_RULES.hard_side_clearance_cm,
            FIXED_RULES.hard_rear_clearance_cm,
        )

    def with_overrides(self, **values: Any) -> "PlannerTuning":
        """Devuelve una variante validada para un experimento.

        Los nombres físicos antiguos se rechazan para evitar que un sweep
        vuelva a optimizar hardware por accidente.
        """
        forbidden = {
            "fixed_speed_cm_s", "max_speed_cm_s", "max_steering_deg",
            "max_steering_rate_deg_s", "max_acceleration_cm_s2",
            "max_deceleration_cm_s2", "simulation_dt_s",
            "mandatory_clearance_cm",
            "desired_clearance_cm", "safety_margins",
        }
        invalid = sorted(forbidden.intersection(values))
        if invalid:
            raise ValueError(
                "Parámetros FIXED no se pueden modificar desde PlannerTuning: "
                + ", ".join(invalid)
            )
        return replace(self, **values).validate()


def _tuple_values(raw: Any, name: str) -> tuple[float, ...]:
    if not isinstance(raw, (list, tuple)):
        raise ValueError(f"{name} debe ser una lista")
    return tuple(float(value) for value in raw)


def load_planner_tuning(path: str | Path | None = None) -> PlannerTuning:
    """Carga únicamente claves TUNABLE del JSON indicado."""
    tuning_path = Path(path) if path else DEFAULT_TUNING_FILE
    if not tuning_path.exists():
        return PlannerTuning().validate()
    payload = json.loads(tuning_path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("El archivo de tuning debe contener un objeto JSON")

    values = {
        key: value for key, value in payload.items()
        if not str(key).startswith("_")
    }
    # Claves retiradas: se ignoran, nunca vuelven a controlar decisiones.
    for key in (
        "fixed_speed_cm_s", "max_steering_deg", "simulation_dt_s",
        "max_steering_rate_deg_s", "max_acceleration_cm_s2",
        "max_deceleration_cm_s2",
        "mandatory_clearance_cm", "desired_clearance_cm", "safety_margins",
        "phase_transitions", "planning_horizon_s",
    ):
        values.pop(key, None)

    raw_preferred = values.pop("preferred_safety_margins", None)
    if isinstance(raw_preferred, dict):
        nested = raw_preferred.get("preferred", raw_preferred)
        if not isinstance(nested, dict):
            raise ValueError("preferred_safety_margins debe ser un objeto JSON")
        values["preferred_safety_margins"] = PreferredSafetyMargins(
            front_cm=float(nested.get("front_cm", 12.0)),
            side_cm=float(nested.get("side_cm", 5.0)),
            rear_cm=float(nested.get("rear_cm", 5.0)),
        ).validate()
    elif raw_preferred is not None:
        raise ValueError("preferred_safety_margins debe ser un objeto JSON")

    raw_weights = values.get("score_weights")
    if isinstance(raw_weights, dict):
        values["score_weights"] = TrajectoryScoreWeights(**{
            key: float(value)
            for key, value in raw_weights.items()
            if key in TrajectoryScoreWeights.__dataclass_fields__
        })
    elif raw_weights is not None:
        raise ValueError("score_weights debe ser un objeto JSON")

    known = {item.name for item in fields(PlannerTuning)}
    values = {key: value for key, value in values.items() if key in known}
    for key in ("steering_fractions",):
        if key in values:
            values[key] = _tuple_values(values[key], key)
    return PlannerTuning(**values).validate()


def save_planner_tuning(tuning: PlannerTuning, path: str | Path | None = None) -> Path:
    """Guarda solo la configuración TUNABLE y conserva comentarios JSON."""
    tuning.validate()
    tuning_path = Path(path) if path else DEFAULT_TUNING_FILE
    tuning_path.parent.mkdir(parents=True, exist_ok=True)
    payload = asdict(tuning)
    payload.pop("allow_physical_collisions", None)
    payload.pop("disable_hard_safety_margins", None)
    if tuning_path.exists():
        try:
            previous = json.loads(tuning_path.read_text(encoding="utf-8"))
            if isinstance(previous, dict) and isinstance(previous.get("_comments"), dict):
                payload = {"_comments": previous["_comments"], **payload}
        except json.JSONDecodeError:
            pass
    tuning_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    return tuning_path
