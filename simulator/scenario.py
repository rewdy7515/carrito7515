"""Escenarios compartidos por Pygame y las pruebas headless."""

from __future__ import annotations

import random
from dataclasses import dataclass

try:
    from planner_rules import FIXED_RULES
except ImportError:
    from simulator.planner_rules import FIXED_RULES


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
    single_obstacle_straight: str | None = None


# Cada recta tiene una matriz de 3 filas x 2 columnas. Cada tupla está en
# orden fila-major: fila 1 izquierda/derecha, fila 2 izquierda/derecha,
# fila 3 izquierda/derecha. La columna 0 es la más cercana al muro exterior.
STRAIGHT_SEATS: dict[str, tuple[tuple[float, float], ...]] = {
    "bottom": ((100.0, 260.0), (100.0, 240.0), (150.0, 260.0),
                (150.0, 240.0), (200.0, 260.0), (200.0, 240.0)),
    "left": ((40.0, 100.0), (60.0, 100.0), (40.0, 150.0),
              (60.0, 150.0), (40.0, 200.0), (60.0, 200.0)),
    "top": ((100.0, 40.0), (100.0, 60.0), (150.0, 40.0),
             (150.0, 60.0), (200.0, 40.0), (200.0, 60.0)),
    "right": ((260.0, 100.0), (240.0, 100.0), (260.0, 150.0),
               (240.0, 150.0), (260.0, 200.0), (240.0, 200.0)),
}

STRAIGHT_NAMES = ("bottom", "left", "top", "right")


def seat_slot(seat: tuple[float, float]) -> tuple[str, int]:
    x, y = seat
    if y in (40.0, 60.0):
        return "top", int((x - 100.0) / 50.0) * 2 + (0 if y == 40.0 else 1)
    if y in (240.0, 260.0):
        return "bottom", int((x - 100.0) / 50.0) * 2 + (0 if y == 260.0 else 1)
    if x in (40.0, 60.0):
        return "left", int((y - 100.0) / 50.0) * 2 + (0 if x == 40.0 else 1)
    return "right", int((y - 100.0) / 50.0) * 2 + (0 if x == 260.0 else 1)


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
    """Genera un escenario completo de obstáculos de forma reproducible.

    ``scenario_index`` solo identifica el escenario. La cantidad, posiciones y
    colores dependen exclusivamente del estado recibido en ``rng``.
    """
    single_straight = rng.choice(STRAIGHT_NAMES)
    selected: list[ScenarioObject] = []

    # Obstáculo único: fila central, columna exterior.
    single_seat = STRAIGHT_SEATS[single_straight][2]
    selected.append(ScenarioObject(
        "1", single_seat[0], single_seat[1], rng.choice(("red", "green"))
    ))

    # En las otras rectas se seleccionan primero cantidad y posiciones; cada
    # color se sortea después, sin alterar las posiciones seleccionadas.
    used_configurations: set[tuple[tuple[int, str], ...]] = set()
    for straight in STRAIGHT_NAMES:
        if straight == single_straight:
            continue
        while True:
            rows = rng.sample(range(3), rng.choice((1, 2)))
            indexes = sorted(row * 2 + rng.randrange(2) for row in rows)
            colors = tuple(rng.choice(("red", "green")) for _ in indexes)
            configuration = tuple(zip(indexes, colors))
            if configuration not in used_configurations:
                used_configurations.add(configuration)
                break
        for index, color in configuration:
            seat = STRAIGHT_SEATS[straight][index]
            selected.append(ScenarioObject(
                str(len(selected) + 1), seat[0], seat[1], color
            ))

    return Scenario(
        scenario_index,
        tuple(selected),
        "escenario reproducible con obstáculos",
        single_straight,
    )


def generate_objects(rng: random.Random, scenario_index: int) -> list[ScenarioObject]:
    """Compatibility helper para el runner existente."""
    return list(generate_scenario(rng, scenario_index).objects)
