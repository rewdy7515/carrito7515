"""Registro inicial de medidas físicas y calibración manual del robot.

Este archivo no se conecta al Arduino, no mueve motores y no implementa
navegación. Solo permite introducir o confirmar datos físicos para guardarlos
en un archivo JSON que podrá utilizar el software futuro.
"""

from __future__ import annotations

import json
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
OUTPUT_FILE = PROJECT_ROOT / "config" / "physical_measurements.json"

DEFAULTS = {
    "overall_front_to_rear_cm": 21.15,
    "wheel_center_to_center_cm": 14.8,
    "wheel_diameter_cm": 6.3,
    "front_axle_wheel_center_distance_cm": 14.6,
    "camera_vertical_angle_deg": None,
    "servo_center_deg": 86.0,
    "servo_safe_min_deg": 60.0,
    "servo_safe_max_deg": 120.0,
}


def ask_number(label: str, default: float | None) -> float | None:
    """Ask for a number while allowing the current value to be preserved."""
    suffix = "" if default is None else f" [{default}]"
    raw = input(f"{label}{suffix}: ").strip()

    if not raw:
        return default

    if raw.lower() in {"n", "ninguno", "pendiente"}:
        return None

    try:
        return float(raw.replace(",", "."))
    except ValueError:
        print("Valor inválido; se conservará el valor anterior.")
        return default


def collect_measurements() -> dict[str, object]:
    """Collect physical measurements and manual servo settings."""
    print("Registro físico del robot")
    print("Enter conserva el valor mostrado. 'pendiente' deja un valor sin confirmar.\n")

    measurements = {
        "overall_front_to_rear_cm": ask_number(
            "Distancia entre extremos delantero y trasero (cm)",
            DEFAULTS["overall_front_to_rear_cm"],
        ),
        "wheel_center_to_center_cm": ask_number(
            "Distancia entre centros de rueda delantera y trasera (cm)",
            DEFAULTS["wheel_center_to_center_cm"],
        ),
        "wheel_diameter_cm": ask_number(
            "Diámetro de las ruedas (cm)", DEFAULTS["wheel_diameter_cm"]
        ),
        "front_axle_wheel_center_distance_cm": ask_number(
            "Distancia entre centros de las ruedas delanteras (cm)",
            DEFAULTS["front_axle_wheel_center_distance_cm"],
        ),
        "camera_vertical_angle_deg": ask_number(
            "Ángulo vertical de la cámara (grados; ajustable)",
            DEFAULTS["camera_vertical_angle_deg"],
        ),
        "servo_center_deg": ask_number(
            "Centro provisional del servo (grados)", DEFAULTS["servo_center_deg"]
        ),
        "servo_safe_min_deg": ask_number(
            "Límite mínimo seguro del servo (grados)",
            DEFAULTS["servo_safe_min_deg"],
        ),
        "servo_safe_max_deg": ask_number(
            "Límite máximo seguro del servo (grados)",
            DEFAULTS["servo_safe_max_deg"],
        ),
    }

    return {
        "units": {"distance": "cm", "angle": "degrees"},
        "measurements": measurements,
        "notes": [
            "Las medidas fueron proporcionadas por el usuario y deben verificarse físicamente.",
            "El ángulo vertical de la cámara es ajustable y no tiene aún un valor definitivo.",
            "El centro y los límites del servo son provisionales hasta probar el mecanismo.",
        ],
    }


def main() -> None:
    data = collect_measurements()
    OUTPUT_FILE.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_FILE.write_text(
        json.dumps(data, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(f"\nDatos guardados en: {OUTPUT_FILE}")


if __name__ == "__main__":
    main()

