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
    from planner_tuning import load_planner_tuning
except ImportError:
    from simulator.planner_test_runner import PlannerConfig, SensorModel, run_scenario
    from simulator.planner_tuning import load_planner_tuning


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
        "fixed_speed_cm_s": config.fixed_speed_cm_s,
        "mandatory_clearance_cm": config.mandatory_clearance_cm,
        "desired_clearance_cm": config.desired_clearance_cm,
        "replanning_period_s": config.replanning_period_s,
        "planning_horizon_s": config.planning_horizon_s,
        "max_steering_rate_deg_s": config.max_steering_rate_deg_s,
        "max_acceleration_cm_s2": config.max_acceleration_cm_s2,
        "max_deceleration_cm_s2": config.max_deceleration_cm_s2,
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
    fixed_speed: Iterable[float],
    mandatory: Iterable[float],
    desired: Iterable[float],
    replanning: Iterable[float],
    horizon: Iterable[float],
    steering_rate: Iterable[float],
    acceleration: Iterable[float],
    deceleration: Iterable[float],
    base_tuning: PlannerConfig | None = None,
) -> list[PlannerConfig]:
    configurations: list[PlannerConfig] = []
    base = base_tuning or PlannerConfig()
    for values in itertools.product(
        fixed_speed, mandatory, desired, replanning, horizon, steering_rate,
        acceleration, deceleration,
    ):
        speed_cm_s, mandatory_cm, desired_cm, period_s, horizon_s, rate, accel, decel = values
        if desired_cm < mandatory_cm:
            continue
        configurations.append(base.with_overrides(
            fixed_speed_cm_s=speed_cm_s,
            mandatory_clearance_cm=mandatory_cm,
            desired_clearance_cm=desired_cm,
            replanning_period_s=period_s,
            planning_horizon_s=horizon_s,
            max_steering_rate_deg_s=rate,
            max_acceleration_cm_s2=accel,
            max_deceleration_cm_s2=decel,
        ))
    return configurations


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenarios", type=int, default=20)
    parser.add_argument("--seed", type=int, default=20260815)
    parser.add_argument("--duration-s", type=float, default=20.0)
    parser.add_argument("--fixed-speed-cm-s", default="24",
                        help="Una velocidad o lista separada por coma para comparar; ejemplo: 15,18,21")
    parser.add_argument("--output-dir", type=Path, default=Path("/tmp/wro_planner_sweep"))
    parser.add_argument("--planner-config", type=Path, default=None,
                        help="Archivo JSON base de PlannerTuning.")
    parser.add_argument("--mandatory-clearance-cm", default="9,10,11",
                        help="Valores separados por coma; ejemplo: 8,10,12")
    parser.add_argument("--desired-clearance-cm", default="13,15",
                        help="Valores separados por coma; siempre >= mandatory")
    parser.add_argument("--replanning-period-s", default="0.15,0.20,0.25")
    parser.add_argument("--planning-horizon-s", default="1.5,2.0,2.5")
    parser.add_argument("--steering-rate-deg-s", default="90")
    parser.add_argument("--acceleration-cm-s2", default="45",
                        help="Valores separados por coma; ejemplo: 35,45,55")
    parser.add_argument("--deceleration-cm-s2", default="70",
                        help="Valores separados por coma; ejemplo: 55,70,85")
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
        parse_values(args.fixed_speed_cm_s),
        parse_values(args.mandatory_clearance_cm),
        parse_values(args.desired_clearance_cm),
        parse_values(args.replanning_period_s),
        parse_values(args.planning_horizon_s),
        parse_values(args.steering_rate_deg_s),
        parse_values(args.acceleration_cm_s2),
        parse_values(args.deceleration_cm_s2),
        base_tuning,
    )
    if any(speed <= 0 or speed > 32 for speed in parse_values(args.fixed_speed_cm_s)):
        parser.error("fixed-speed-cm-s debe estar entre 0 y 32 cm/s")
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
            f"speed={config.fixed_speed_cm_s:g} cm/s, "
            f"clearance={config.mandatory_clearance_cm:g}/"
            f"{config.desired_clearance_cm:g} cm, "
            f"replan={config.replanning_period_s:g} s, "
            f"horizon={config.planning_horizon_s:g} s, "
            f"steering_rate={config.max_steering_rate_deg_s:g} deg/s, "
            f"accel/decel={config.max_acceleration_cm_s2:g}/{config.max_deceleration_cm_s2:g} cm/s2",
            flush=True,
        )
        summaries: list[dict[str, object]] = []
        for scenario_index in range(args.scenarios):
            summaries.append(run_scenario(
                args.seed + scenario_index,
                scenario_index,
                sensor,
                args.duration_s,
                config.fixed_speed_cm_s,
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
            "fixed_speed_cm_s": args.fixed_speed_cm_s,
            "best": best,
            "configurations": results,
        }, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    with (args.output_dir / "sweep_results.csv").open("w", newline="", encoding="utf-8") as handle:
        columns = [
            "configuration_id", "fixed_speed_cm_s", "mandatory_clearance_cm",
            "desired_clearance_cm", "replanning_period_s", "planning_horizon_s",
            "max_steering_rate_deg_s", "max_acceleration_cm_s2",
            "max_deceleration_cm_s2", "score", "collisions", "collision_rate",
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
