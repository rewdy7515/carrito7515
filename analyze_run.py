import argparse
import csv
import json
from pathlib import Path


def load_json(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def load_csv_rows(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def safe_int(value, default=0):
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def safe_float(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def infer_base_path(run_path):
    path = Path(run_path)
    if path.suffix == ".json":
        return path.with_suffix("")
    if path.suffix == ".csv" and path.name.endswith(".timestamps.csv"):
        return path.with_name(path.name[: -len(".timestamps.csv")])
    if path.suffix == ".mp4":
        return path.with_suffix("")
    return path


def build_paths(base_path):
    return {
        "base": base_path,
        "json": base_path.with_suffix(".json"),
        "csv": base_path.with_suffix(".timestamps.csv"),
        "mp4": base_path.with_suffix(".mp4"),
    }


def check_files(paths):
    missing = [str(path) for path in paths.values() if path != paths["base"] and not path.exists()]
    if missing:
        raise FileNotFoundError("Faltan archivos requeridos:\n" + "\n".join(missing))


def summarize_run(metadata, rows):
    print("== Resumen de corrida ==")
    print(f"Video: {metadata.get('video_path')}")
    print(f"Timestamps: {metadata.get('timestamps_path')}")
    print(f"JSON: {metadata.get('json_path')}")
    print(f"Resolucion: {metadata.get('width')}x{metadata.get('height')}")
    print(f"FPS metadata: {metadata.get('fps')}")
    print(f"Inicio: {metadata.get('started_at_local')}")
    print(f"Fin: {metadata.get('ended_at_local')}")
    print(f"Duracion: {metadata.get('duration_seconds')} s")
    print(f"Frames JSON: {metadata.get('frames_written')}")
    print(f"Filas CSV: {len(rows)}")
    print(f"Marcadores JSON: {len(metadata.get('markers', []))}")
    print()


def analyze_markers(metadata, rows, context):
    markers = metadata.get("markers", [])
    if not markers:
        print("No hay marcadores en el JSON.")
        return

    print("== Marcadores ==")
    for idx, marker in enumerate(markers, start=1):
        frame_index = safe_int(marker.get("frame_index"))
        start = max(0, frame_index - context)
        end = min(len(rows) - 1, frame_index + context) if rows else -1

        print(
            f"[{idx}] frame={frame_index} t={marker.get('elapsed_seconds')}s "
            f"label={marker.get('label')} speed={marker.get('speed')} angle={marker.get('angle')}"
        )

        if not rows:
            print("  Sin filas CSV para cruzar.")
            continue

        if frame_index >= len(rows):
            print(f"  El frame marcado no existe en el CSV. Ultimo frame CSV: {len(rows) - 1}")
            continue

        row = rows[frame_index]
        print(
            "  Fila exacta CSV: "
            f"frame={row['frame_index']} elapsed={row['elapsed_seconds']} "
            f"speed={row['current_speed']} angle={row['current_angle']} marker={row['marker']!r}"
        )

        print(f"  Contexto CSV: frames {start}..{end}")
        for csv_idx in range(start, end + 1):
            csv_row = rows[csv_idx]
            prefix = "->" if csv_idx == frame_index else "  "
            print(
                f"{prefix} frame={csv_row['frame_index']:>4} "
                f"t={csv_row['elapsed_seconds']:>9} "
                f"speed={csv_row['current_speed']:>3} "
                f"angle={csv_row['current_angle']:>3} "
                f"marker={csv_row['marker']!r}"
            )
        print()


def summarize_ranges(rows):
    if not rows:
        return

    speeds = [safe_int(row.get("current_speed")) for row in rows]
    angles = [safe_int(row.get("current_angle")) for row in rows]
    elapsed = [safe_float(row.get("elapsed_seconds")) for row in rows]

    print("== Rango de control ==")
    print(f"Velocidad min/max: {min(speeds)} / {max(speeds)}")
    print(f"Angulo min/max: {min(angles)} / {max(angles)}")
    print(f"Tiempo CSV min/max: {min(elapsed):.3f} / {max(elapsed):.3f}")
    print()


def compare_json_csv(metadata, rows):
    issues = []

    if metadata.get("frames_written") != len(rows):
        issues.append(
            f"frames_written JSON={metadata.get('frames_written')} distinto a filas CSV={len(rows)}"
        )

    if rows:
        last_index = safe_int(rows[-1].get("frame_index"), -1)
        if last_index != len(rows) - 1:
            issues.append(
                f"ultimo frame_index CSV={last_index} distinto a len(rows)-1={len(rows) - 1}"
            )

    print("== Consistencia ==")
    if issues:
        for issue in issues:
            print(f"- {issue}")
    else:
        print("Sin inconsistencias estructurales entre JSON y CSV.")
    print()


def main():
    parser = argparse.ArgumentParser(
        description="Analiza una corrida grabada del bot usando .json + .timestamps.csv + .mp4."
    )
    parser.add_argument(
        "run_path",
        help="Ruta base de la corrida o ruta a uno de sus archivos (.json, .timestamps.csv o .mp4).",
    )
    parser.add_argument(
        "--context",
        type=int,
        default=5,
        help="Cantidad de frames antes y despues de cada marcador para mostrar en el reporte.",
    )
    args = parser.parse_args()

    base_path = infer_base_path(args.run_path)
    paths = build_paths(base_path)
    check_files(paths)

    metadata = load_json(paths["json"])
    rows = load_csv_rows(paths["csv"])

    summarize_run(metadata, rows)
    compare_json_csv(metadata, rows)
    summarize_ranges(rows)
    analyze_markers(metadata, rows, max(0, args.context))


if __name__ == "__main__":
    main()
