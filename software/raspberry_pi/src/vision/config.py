"""Configuración persistente y constantes de visión."""

from __future__ import annotations

import json
import threading
from pathlib import Path

TRACK_MAT_SIZE_MM = (3200, 3200)
TRACK_INTERIOR_SIZE_MM = (3000, 3000)
OUTER_AND_INNER_WALL_HEIGHT_MM = 100
TRACK_LINE_WIDTH_MM = 20
START_ZONE_SIZE_MM = (200, 500)
SIGN_SEAT_SIZE_MM = (50, 50)
OBSTACLE_DIMENSIONS_MM = (50, 50, 100)
MAX_OBSTACLES_PER_COLOR = 7
PARKING_DELIMITER_DIMENSIONS_MM = (200, 20, 100)
TUNING_PATH = Path(__file__).resolve().parents[4] / "config" / "vision_tuning.json"
MODEL_PATH = (
    Path(__file__).resolve().parents[4]
    / "yolo/obstacles/models/wro_obstacle_detector_ncnn_model"
)
CAMERA_CALIBRATION_PATH = Path(__file__).resolve().parents[4] / "config" / "camera_calibration.json"

DEFAULT_TUNING = {
    "safe_distance_mm": 200,
    "safe_zone_top_ratio": 0.68,
    "black_wall_threshold": 65,
    "colors": {
        "red": {"hue_tolerance": 10, "minimum_saturation": 80, "minimum_value": 80},
        "green": {"hue_tolerance": 35, "minimum_saturation": 20, "minimum_value": 15},
        "blue": {"hue_tolerance": 18, "minimum_saturation": 60, "minimum_value": 50},
        "orange": {"hue_tolerance": 15, "minimum_saturation": 70, "minimum_value": 50},
        "magenta": {"hue_tolerance": 15, "minimum_saturation": 80, "minimum_value": 70},
    },
}


def copy_tuning(value: dict) -> dict:
    return json.loads(json.dumps(value))


class TuningState:
    """Valores ajustables desde el panel web, persistidos en JSON."""

    def __init__(self, path: Path) -> None:
        self.path, self.lock = path, threading.RLock()
        self.values = copy_tuning(DEFAULT_TUNING)
        self._load()

    def _load(self) -> None:
        try:
            with self.path.open(encoding="utf-8") as file:
                self._merge(json.load(file))
        except (FileNotFoundError, json.JSONDecodeError, OSError, TypeError):
            pass

    def _merge(self, loaded: dict) -> None:
        if not isinstance(loaded, dict):
            return
        for key in DEFAULT_TUNING:
            if key != "colors" and isinstance(loaded.get(key), (int, float)):
                self.values[key] = loaded[key]
        for color, defaults in DEFAULT_TUNING["colors"].items():
            supplied = loaded.get("colors", {}).get(color, {})
            if isinstance(supplied, dict):
                for key in defaults:
                    if isinstance(supplied.get(key), (int, float)):
                        self.values["colors"][color][key] = supplied[key]
        self.values = self._clamp(self.values)

    @staticmethod
    def _clamp(values: dict) -> dict:
        result = copy_tuning(DEFAULT_TUNING)
        for key in ("safe_distance_mm", "black_wall_threshold"):
            result[key] = int(max(1, min(5000, values.get(key, result[key]))))
        for key in ("safe_zone_top_ratio",):
            result[key] = max(0.05, min(0.98, float(values.get(key, result[key]))))
        for color, defaults in result["colors"].items():
            source = values.get("colors", {}).get(color, {})
            defaults["hue_tolerance"] = int(max(1, min(60, source.get("hue_tolerance", defaults["hue_tolerance"]))))
            defaults["minimum_saturation"] = int(max(0, min(255, source.get("minimum_saturation", defaults["minimum_saturation"]))))
            defaults["minimum_value"] = int(max(0, min(255, source.get("minimum_value", defaults["minimum_value"]))))
        return result

    def snapshot(self) -> dict:
        with self.lock:
            return copy_tuning(self.values)

    def update(self, values: dict) -> dict:
        with self.lock:
            merged = self.snapshot()
            for key in DEFAULT_TUNING:
                if key != "colors" and key in values:
                    merged[key] = values[key]
            for color, supplied in values.get("colors", {}).items():
                if color in merged["colors"] and isinstance(supplied, dict):
                    merged["colors"][color].update(supplied)
            self.values = self._clamp(merged)
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self.path.write_text(json.dumps(self.values, indent=2) + "\n", encoding="utf-8")
            return copy_tuning(self.values)
