"""Parámetros ajustables del planner y carga desde JSON."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, fields, replace
from pathlib import Path
from typing import Any

try:
    from planner_rules import FIXED_RULES
except ImportError:
    from simulator.planner_rules import FIXED_RULES


DEFAULT_TUNING_FILE = Path(__file__).resolve().parents[1] / "config" / "simulator_planner_tuning.json"


@dataclass(frozen=True)
class SafetyMargins:
    """Margenes geometricos por eje del rectangulo completo del carro."""

    hard_front_cm: float = 6.0
    hard_side_cm: float = 3.0
    hard_rear_cm: float = 3.0
    preferred_front_cm: float = 12.0
    preferred_side_cm: float = 5.0
    preferred_rear_cm: float = 5.0

    def validate(self) -> "SafetyMargins":
        values = (
            self.hard_front_cm, self.hard_side_cm, self.hard_rear_cm,
            self.preferred_front_cm, self.preferred_side_cm, self.preferred_rear_cm,
        )
        if any(value < 0.0 for value in values):
            raise ValueError("Los margenes de seguridad no pueden ser negativos")
        if self.preferred_front_cm < self.hard_front_cm:
            raise ValueError("preferred_front_cm debe ser >= hard_front_cm")
        if self.preferred_side_cm < self.hard_side_cm:
            raise ValueError("preferred_side_cm debe ser >= hard_side_cm")
        if self.preferred_rear_cm < self.hard_rear_cm:
            raise ValueError("preferred_rear_cm debe ser >= hard_rear_cm")
        return self


@dataclass(frozen=True)
class PlannerTuning:
    """Valores que se pueden calibrar sin cambiar las reglas de seguridad."""

    fixed_speed_cm_s: float = 24.0
    max_steering_deg: float = FIXED_RULES.maximum_physical_steering_deg
    safety_margins: SafetyMargins = SafetyMargins()
    simulation_dt_s: float = FIXED_RULES.simulation_dt_s
    replanning_period_s: float = 0.20
    planning_horizon_s: float = 2.0
    preview_horizon_s: float = 5.0
    memory_timeout_s: float = 2.0
    max_candidates: int = 256
    max_planning_time_ms: float = 20.0
    max_steering_rate_deg_s: float = 90.0
    max_acceleration_cm_s2: float = 45.0
    max_deceleration_cm_s2: float = 70.0
    turn_angles_deg: tuple[float, ...] = (15.0, 10.0, 5.0, 0.0)
    counter_steer_angles_deg: tuple[float, ...] = (15.0, 10.0, 5.0, 0.0)
    def validate(self) -> "PlannerTuning":
        self.safety_margins.validate()
        if not 0.0 < self.fixed_speed_cm_s <= 32.0:
            raise ValueError("fixed_speed_cm_s debe estar entre 0 y 32")
        if not 0.0 < self.max_steering_deg < 89.0:
            raise ValueError("max_steering_deg inválido")
        if self.replanning_period_s <= 0.0 or self.planning_horizon_s <= 0.0:
            raise ValueError("Los periodos del planner deben ser positivos")
        if self.preview_horizon_s < self.planning_horizon_s:
            raise ValueError("preview_horizon_s debe ser >= planning_horizon_s")
        if self.memory_timeout_s < 0.0 or self.max_candidates <= 0 or self.max_planning_time_ms <= 0.0:
            raise ValueError("Presupuesto o memoria inválidos")
        if self.max_steering_rate_deg_s <= 0.0 or self.max_acceleration_cm_s2 <= 0.0 or self.max_deceleration_cm_s2 <= 0.0:
            raise ValueError("Límites dinámicos inválidos")
        if not self.turn_angles_deg or not self.counter_steer_angles_deg:
            raise ValueError("Debe existir al menos un ángulo candidato")
        return self

    @property
    def mandatory_clearance_cm(self) -> float:
        """Alias legacy: representa el mayor margen hard para reportes antiguos."""
        return max(
            self.safety_margins.hard_front_cm,
            self.safety_margins.hard_side_cm,
            self.safety_margins.hard_rear_cm,
        )

    @property
    def desired_clearance_cm(self) -> float:
        """Alias legacy: representa el mayor margen preferred para reportes antiguos."""
        return max(
            self.safety_margins.preferred_front_cm,
            self.safety_margins.preferred_side_cm,
            self.safety_margins.preferred_rear_cm,
        )

    def with_overrides(self, **values: Any) -> "PlannerTuning":
        legacy_hard = values.pop("mandatory_clearance_cm", None)
        legacy_preferred = values.pop("desired_clearance_cm", None)
        margins = self.safety_margins
        if legacy_hard is not None:
            legacy_hard = float(legacy_hard)
            margins = replace(
                margins,
                hard_front_cm=legacy_hard,
                hard_side_cm=legacy_hard,
                hard_rear_cm=legacy_hard,
            )
        if legacy_preferred is not None:
            legacy_preferred = float(legacy_preferred)
            margins = replace(
                margins,
                preferred_front_cm=legacy_preferred,
                preferred_side_cm=legacy_preferred,
                preferred_rear_cm=legacy_preferred,
            )
        return replace(self, safety_margins=margins, **values).validate()


def _tuple_values(raw: Any, name: str) -> tuple[float, ...]:
    if not isinstance(raw, (list, tuple)):
        raise ValueError(f"{name} debe ser una lista")
    return tuple(float(value) for value in raw)


def load_planner_tuning(path: str | Path | None = None) -> PlannerTuning:
    """Carga tuning editable; si no existe, usa los defaults del simulador."""
    tuning_path = Path(path) if path else DEFAULT_TUNING_FILE
    if not tuning_path.exists():
        return PlannerTuning().validate()
    payload = json.loads(tuning_path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("El archivo de tuning debe contener un objeto JSON")
    # Las claves privadas permiten documentar el JSON sin usar comentarios
    # no estándar como // o /* */. No forman parte de PlannerTuning.
    values = {
        key: value for key, value in payload.items()
        if not str(key).startswith("_")
    }
    # Los campos retirados se ignoran para que un archivo anterior siga
    # siendo legible, pero ya no pueden controlar la duracion de una fase.
    for key in (
        "phase_durations_s", "pass_hold_steering_fraction",
        "pass_hold_residual_fraction", "stabilizing_pass_steering_deg",
        "stabilizing_counter_steering_deg", "stabilizing_steering_deg",
    ):
        values.pop(key, None)
    legacy_hard = values.pop("mandatory_clearance_cm", None)
    legacy_preferred = values.pop("desired_clearance_cm", None)
    raw_margins = values.get("safety_margins")
    if isinstance(raw_margins, dict):
        # Acepta la forma plana guardada por versiones anteriores y la forma
        # agrupada que se usa en la documentacion:
        # {"hard": {"front_cm": ...}, "preferred": {...}}.
        margin_values: dict[str, Any] = {}
        for group, prefix in (("hard", "hard"), ("preferred", "preferred")):
            nested = raw_margins.get(group, {})
            if nested is not None and not isinstance(nested, dict):
                raise ValueError(f"safety_margins.{group} debe ser un objeto JSON")
            for axis in ("front", "side", "rear"):
                flat_key = f"{prefix}_{axis}_cm"
                if flat_key in raw_margins:
                    margin_values[flat_key] = raw_margins[flat_key]
                elif axis + "_cm" in nested:
                    margin_values[flat_key] = nested[axis + "_cm"]
        values["safety_margins"] = SafetyMargins(**margin_values).validate()
    elif raw_margins is not None:
        raise ValueError("safety_margins debe ser un objeto JSON")
    margins = values.get("safety_margins", SafetyMargins())
    if legacy_hard is not None:
        margins = replace(
            margins,
            hard_front_cm=float(legacy_hard),
            hard_side_cm=float(legacy_hard),
            hard_rear_cm=float(legacy_hard),
        )
    if legacy_preferred is not None:
        margins = replace(
            margins,
            preferred_front_cm=float(legacy_preferred),
            preferred_side_cm=float(legacy_preferred),
            preferred_rear_cm=float(legacy_preferred),
        )
    values["safety_margins"] = margins
    known = {item.name for item in fields(PlannerTuning)}
    values = {key: value for key, value in values.items() if key in known}
    for key in ("turn_angles_deg", "counter_steer_angles_deg"):
        if key in values:
            values[key] = _tuple_values(values[key], key)
    return PlannerTuning(**values).validate()


def save_planner_tuning(tuning: PlannerTuning, path: str | Path | None = None) -> Path:
    tuning.validate()
    tuning_path = Path(path) if path else DEFAULT_TUNING_FILE
    tuning_path.parent.mkdir(parents=True, exist_ok=True)
    payload = asdict(tuning)
    margins = payload.get("safety_margins")
    if isinstance(margins, dict):
        payload["safety_margins"] = {
            "hard": {
                "front_cm": margins["hard_front_cm"],
                "side_cm": margins["hard_side_cm"],
                "rear_cm": margins["hard_rear_cm"],
            },
            "preferred": {
                "front_cm": margins["preferred_front_cm"],
                "side_cm": margins["preferred_side_cm"],
                "rear_cm": margins["preferred_rear_cm"],
            },
        }
    if tuning_path.exists():
        try:
            previous = json.loads(tuning_path.read_text(encoding="utf-8"))
            if isinstance(previous, dict) and isinstance(previous.get("_comments"), dict):
                payload = {"_comments": previous["_comments"], **payload}
        except json.JSONDecodeError:
            pass
    tuning_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    return tuning_path
