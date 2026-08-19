"""Registro inicial de medidas físicas y calibración manual del robot.

Este archivo no se conecta al Arduino, no mueve motores y no implementa
navegación. Solo permite introducir o confirmar datos físicos para guardarlos
en un archivo JSON que podrá utilizar el software futuro.
"""

from __future__ import annotations

import json
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
OUTPUT_FILE = PROJECT_ROOT / "config" / "physical_measurements.json"

DEFAULTS = {
    "overall_front_to_rear_cm": 21.15,
    "wheelbase_cm": 15.0,
    "front_wheel_center_track_cm": 14.9,
    "wheel_width_cm": 2.3,
    "wheel_diameter_cm": 6.8,
    "turn_radius_right_cm": 32.2,
    "turn_radius_left_cm": 43.0,
    "camera_vertical_angle_deg": None,
    "servo_center_deg": 92.0,
    "servo_safe_min_deg": 60.0,
    "servo_safe_max_deg": 120.0,
}

STEERING_MEASUREMENTS = {
    "reference_servo_deg": 90.0,
    "description": (
        "Ángulos de cada rueda delantera respecto a la recta observada con el "
        "servo en 90°. No se reducen a un único ángulo de bicicleta hasta medir "
        "el radio real del carro."
    ),
    "wheel_angles_deg": [
        {
            "servo_command_deg": 60.0,
            "turn": "right",
            "left_wheel_deg": 22.75,
            "right_wheel_deg": 27.97,
        },
        {
            "servo_command_deg": 120.0,
            "turn": "left",
            "left_wheel_deg": 21.45,
            "right_wheel_deg": 17.58,
        },
    ],
    "front_wheel_tip_lateral_displacements_cm": [
        {
            "servo_command_deg": 140.0,
            "left_wheel_cm": 2.4,
            "right_wheel_cm": 3.9,
        },
        {
            "servo_command_deg": 50.0,
            "left_wheel_cm": 2.3,
            "right_wheel_cm": 2.2,
        },
    ],
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
        "wheelbase_cm": ask_number(
            "Distancia entre ejes delantero y trasero (cm)",
            DEFAULTS["wheelbase_cm"],
        ),
        "wheel_diameter_cm": ask_number(
            "Diámetro de las ruedas (cm)", DEFAULTS["wheel_diameter_cm"]
        ),
        "turn_radius_right_cm": ask_number(
            "Radio de giro medido a la derecha (cm)",
            DEFAULTS["turn_radius_right_cm"],
        ),
        "turn_radius_left_cm": ask_number(
            "Radio de giro medido a la izquierda (cm)",
            DEFAULTS["turn_radius_left_cm"],
        ),
        "front_wheel_center_track_cm": ask_number(
            "Distancia entre centros de las ruedas delanteras (cm)",
            DEFAULTS["front_wheel_center_track_cm"],
        ),
        "wheel_width_cm": ask_number(
            "Ancho de cada rueda (cm)", DEFAULTS["wheel_width_cm"]
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
    track_cm = measurements["front_wheel_center_track_cm"]
    wheel_width_cm = measurements["wheel_width_cm"]
    measurements["wheel_outer_envelope_width_cm"] = (
        track_cm + wheel_width_cm
        if track_cm is not None and wheel_width_cm is not None
        else None
    )

    return {
        "units": {"distance": "cm", "angle": "degrees"},
        "measurements": measurements,
        "steering_measurements": STEERING_MEASUREMENTS,
        "notes": [
            "Las medidas fueron proporcionadas por el usuario y deben verificarse físicamente.",
            "La envolvente lateral de ruedas se deriva de la separación entre centros más el ancho de rueda.",
            "Los ángulos por rueda requieren medir el radio real de giro antes de sustituir el modelo de bicicleta.",
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
