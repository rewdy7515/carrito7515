"""Barrido reproducible de parámetros del planificador geométrico."""

from __future__ import annotations

import argparse
import csv
import itertools
import json
from dataclasses import asdict
from pathlib import Path
from typing import Iterable

try:
    from planner_test_runner import PlannerConfig, SensorModel, run_scenario
    from planner_tuning import PreferredSafetyMargins, load_planner_tuning
except ImportError:
    from simulator.planner_test_runner import PlannerConfig, SensorModel, run_scenario
    from simulator.planner_tuning import PreferredSafetyMargins, load_planner_tuning


def parse_values(raw: str, cast=float) -> list[float]:
    values = [cast(item.strip()) for item in raw.split(",") if item.strip()]
    if not values:
        raise argparse.ArgumentTypeError("La lista de parámetros no puede estar vacía.")
    return values


def aggregate_configuration(
    config_id: int,
    config: PlannerConfig,
    summaries: list[dict[str, object]],
) -> dict[str, object]:
    count = max(1, len(summaries))
    collisions = sum(bool(item["collision"]) for item in summaries)
    maneuvers = sum(bool(item["maneuver_completed"]) for item in summaries)
    laps = sum(bool(item["lap_completed"]) for item in summaries)
    next_straight = sum(bool(item["next_straight_reached"]) for item in summaries)
    route_valid = sum(bool(item["route_progress_valid"]) for item in summaries)
    correct_side = sum(bool(item["correct_pass_side"]) for item in summaries)
    no_safe_cycles = sum(int(item["no_safe_trajectory_cycles"]) for item in summaries)
    p95_values = [float(item["timing"]["p95_ms"]) for item in summaries]
    p95_ms = max(p95_values, default=0.0)
    collision_rate = collisions / count
    route_rate = route_valid / count
    maneuver_rate = maneuvers / count
    lap_rate = laps / count
    next_rate = next_straight / count
    side_rate = correct_side / count
    # La seguridad es una restricción dura. Entre configuraciones sin
    # colisiones se prioriza progreso válido, maniobras y lado correcto; el
    # tiempo y NO_SAFE desempatan sin permitir que oculten una colisión.
    score = (
        -1_000_000.0 * collision_rate
        + 100_000.0 * route_rate
        + 50_000.0 * maneuver_rate
        + 150_000.0 * lap_rate
        + 20_000.0 * next_rate
        + 20_000.0 * side_rate
        - 10.0 * no_safe_cycles
        - p95_ms
    )
    return {
        "configuration_id": config_id,
        "planning_horizon_cm": config.planning_horizon_cm,
        "beam_width": config.beam_width,
        "execution_horizon_min_cm": config.execution_horizon_min_cm,
        "execution_horizon_max_cm": config.execution_horizon_max_cm,
        "switch_margin": config.switch_margin,
        "preferred_clearance_cm": config.preferred_clearance_cm,
        "parameters": asdict(config),
        "scenarios": count,
        "collisions": collisions,
        "collision_rate": collision_rate,
        "maneuvers_completed": maneuvers,
        "maneuver_rate": maneuver_rate,
        "laps_completed": laps,
        "lap_rate": lap_rate,
        "scenarios_reaching_next_straight": next_straight,
        "next_straight_rate": next_rate,
        "route_progress_valid": route_valid,
        "route_progress_rate": route_rate,
        "correct_pass_side": correct_side,
        "correct_pass_side_rate": side_rate,
        "no_safe_trajectory_cycles": no_safe_cycles,
        "timing_p95_ms": p95_ms,
        "score": score,
    }


def configurations_from_grid(
    preferred: Iterable[float],
    horizon: Iterable[float],
    beam_width: Iterable[int],
    execution_min: Iterable[float],
    execution_max: Iterable[float],
    switch_margin: Iterable[float],
    base_tuning: PlannerConfig | None = None,
) -> list[PlannerConfig]:
    configurations: list[PlannerConfig] = []
    base = base_tuning or PlannerConfig()
    for values in itertools.product(
        preferred, horizon, beam_width, execution_min, execution_max, switch_margin,
    ):
        preferred_cm, horizon_cm, width, minimum_cm, maximum_cm, margin = values
        if maximum_cm < minimum_cm:
            continue
        configurations.append(base.with_overrides(
            preferred_safety_margins=PreferredSafetyMargins(
                preferred_cm, preferred_cm, preferred_cm,
            ),
            planning_horizon_cm=horizon_cm,
            beam_width=int(width),
            execution_horizon_min_cm=minimum_cm,
            execution_horizon_max_cm=maximum_cm,
            switch_margin=margin,
        ))
    return configurations


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenarios", type=int, default=20)
    parser.add_argument("--seed", type=int, default=20260815)
    parser.add_argument("--duration-s", type=float, default=20.0)
    parser.add_argument("--output-dir", type=Path, default=Path("/tmp/wro_planner_sweep"))
    parser.add_argument("--planner-config", type=Path, default=None,
                        help="Archivo JSON base de PlannerTuning.")
    parser.add_argument("--preferred-clearance-cm", default="13,15",
                        help="Valores preferred comparados en los tres ejes.")
    parser.add_argument("--planning-horizon-cm", default="50")
    parser.add_argument("--beam-width", default="4")
    parser.add_argument("--execution-horizon-min-cm", default="6")
    parser.add_argument("--execution-horizon-max-cm", default="15")
    parser.add_argument("--switch-margin", default="8")
    parser.add_argument("--noise-position-cm", type=float, default=0.0)
    parser.add_argument("--noise-heading-deg", type=float, default=0.0)
    parser.add_argument("--latency-s", type=float, default=0.0)
    parser.add_argument("--dropout-probability", type=float, default=0.0)
    args = parser.parse_args()
    if args.scenarios <= 0 or args.duration_s <= 0:
        parser.error("scenarios y duration deben ser positivos")
    if not 0 <= args.dropout_probability <= 1:
        parser.error("dropout-probability debe estar entre 0 y 1")

    base_tuning = load_planner_tuning(args.planner_config)
    configs = configurations_from_grid(
        parse_values(args.preferred_clearance_cm),
        parse_values(args.planning_horizon_cm),
        parse_values(args.beam_width, int),
        parse_values(args.execution_horizon_min_cm),
        parse_values(args.execution_horizon_max_cm),
        parse_values(args.switch_margin),
        base_tuning,
    )
    if not configs:
        parser.error("No hay combinaciones válidas: desired-clearance debe ser >= mandatory-clearance")

    sensor = SensorModel(
        args.noise_position_cm,
        args.noise_heading_deg,
        args.latency_s,
        args.dropout_probability,
    )
    results: list[dict[str, object]] = []
    detailed: list[dict[str, object]] = []
    for config_id, config in enumerate(configs, start=1):
        print(
            f"Configuracion {config_id}/{len(configs)}: "
            f"preferred={config.preferred_clearance_cm:g} cm, "
            f"horizon={config.planning_horizon_cm:g} cm, "
            f"beam={config.beam_width}, "
            f"execute={config.execution_horizon_min_cm:g}-{config.execution_horizon_max_cm:g} cm, "
            f"switch_margin={config.switch_margin:g}",
            flush=True,
        )
        summaries: list[dict[str, object]] = []
        for scenario_index in range(args.scenarios):
            summaries.append(run_scenario(
                args.seed + scenario_index,
                scenario_index,
                sensor,
                args.duration_s,
                config,
            )[0])
            print(
                f"\r  Escenarios: {scenario_index + 1}/{args.scenarios} completados",
                end="",
                flush=True,
            )
        print()
        aggregate = aggregate_configuration(config_id, config, summaries)
        results.append(aggregate)
        detailed.append({"aggregate": aggregate, "scenario_summaries": summaries})
        provisional_best = max(results, key=lambda item: float(item["score"]))
        print(
            f"  Resultado: colisiones={aggregate['collisions']}, "
            f"progreso={aggregate['route_progress_rate']:.1%}, "
            f"score={aggregate['score']:.2f}; "
            f"mejor provisional=#{provisional_best['configuration_id']}",
            flush=True,
        )

    results.sort(key=lambda item: float(item["score"]), reverse=True)
    best = results[0]
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "sweep_results.json").write_text(
        json.dumps({
            "seed": args.seed,
            "scenarios": args.scenarios,
            "duration_s": args.duration_s,
            "tunable_only": True,
            "best": best,
            "configurations": results,
        }, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    with (args.output_dir / "sweep_results.csv").open("w", newline="", encoding="utf-8") as handle:
        columns = [
            "configuration_id", "planning_horizon_cm", "beam_width",
            "execution_horizon_min_cm", "execution_horizon_max_cm",
            "switch_margin", "preferred_clearance_cm", "score", "collisions", "collision_rate",
            "maneuver_rate", "next_straight_rate", "route_progress_rate",
            "lap_rate",
            "correct_pass_side_rate", "no_safe_trajectory_cycles", "timing_p95_ms",
        ]
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        writer.writerows({column: item[column] for column in columns} for item in results)
    (args.output_dir / "sweep_scenario_summaries.json").write_text(
        json.dumps(detailed, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print("Barrido completado.")
    print(json.dumps({"best": best, "tested_configurations": len(results)}, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
