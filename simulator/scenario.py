"""Escenarios compartidos por Pygame y las pruebas headless."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

try:
    from planner_rules import FIXED_RULES
    from track_config import SIGN_SEAT_CENTERS, START_POSE
except ImportError:
    from simulator.planner_rules import FIXED_RULES
    from simulator.track_config import SIGN_SEAT_CENTERS, START_POSE


@dataclass(frozen=True)
class ScenarioObject:
    object_id: str
    x_cm: float
    y_cm: float
    color: str
    width_cm: float = FIXED_RULES.default_obstacle_width_cm
    length_cm: float = FIXED_RULES.default_obstacle_length_cm


@dataclass(frozen=True)
class Scenario:
    scenario_id: int
    objects: tuple[ScenarioObject, ...]
    description: str


def seat_slot(seat: tuple[float, float]) -> tuple[str, int]:
    x, y = seat
    if y in (40.0, 60.0):
        return "top", int((x - 100.0) / 50.0)
    if y in (240.0, 260.0):
        return "bottom", int((x - 100.0) / 50.0)
    if x in (40.0, 60.0):
        return "left", int((y - 100.0) / 50.0)
    return "right", int((y - 100.0) / 50.0)


def default_scenario(scenario_id: int) -> Scenario:
    if scenario_id == 1:
        return Scenario(1, (), "sin obstáculos")
    objects = tuple(
        ScenarioObject(str(index), x, y, color)
        for index, (x, y, color) in enumerate((
            (100.0, 40.0, "red"), (200.0, 60.0, "green"),
            (240.0, 100.0, "red"), (260.0, 200.0, "green"),
            (100.0, 240.0, "red"), (200.0, 260.0, "green"),
            (40.0, 100.0, "red"), (60.0, 200.0, "green"),
        ), start=1)
    )
    return Scenario(2, objects, "con obstáculos")


def generate_scenario(rng: random.Random, scenario_index: int) -> Scenario:
    count = scenario_index % 5
    if count == 0:
        return Scenario(scenario_index, (), "sin obstáculos")
    candidates = list(SIGN_SEAT_CENTERS)
    rng.shuffle(candidates)
    selected: list[ScenarioObject] = []
    used_slots: set[tuple[str, int]] = set()
    minimum_start_distance = math.hypot(
        FIXED_RULES.vehicle_length_cm / 2,
        FIXED_RULES.vehicle_width_cm / 2,
    ) + max(FIXED_RULES.default_obstacle_length_cm, FIXED_RULES.default_obstacle_width_cm) / 2 + 10.0
    for seat in candidates:
        if math.hypot(seat[0] - START_POSE[0], seat[1] - START_POSE[1]) < minimum_start_distance:
            continue
        slot = seat_slot(seat)
        if slot in used_slots:
            continue
        used_slots.add(slot)
        color = "red" if rng.randrange(2) == 0 else "green"
        selected.append(ScenarioObject(str(len(selected) + 1), seat[0], seat[1], color))
        if len(selected) >= count:
            break
    return Scenario(scenario_index, tuple(selected), "escenario reproducible")


def generate_objects(rng: random.Random, scenario_index: int) -> list[ScenarioObject]:
    """Compatibility helper para el runner existente."""
    return list(generate_scenario(rng, scenario_index).objects)
